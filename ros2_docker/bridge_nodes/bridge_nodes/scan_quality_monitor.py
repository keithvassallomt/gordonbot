#!/usr/bin/env python3
"""
Scan Quality Monitor Node - Monitors LIDAR scan quality and reports metrics.

Subscribes to /scan topic and analyzes:
- Point count and density
- Angular coverage and gaps
- Range validity
- Scan rate consistency
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import asyncio
import websockets
import json
import threading
from datetime import datetime
import math

# Import the quality analyzer (would need to be accessible in Docker)
# For now, we'll implement a simplified version inline
from dataclasses import dataclass, field
from typing import List, Dict, Any
from enum import Enum
import numpy as np


class QualityLevel(Enum):
    """Quality level indicators."""
    EXCELLENT = "excellent"
    GOOD = "good"
    FAIR = "fair"
    POOR = "poor"
    CRITICAL = "critical"


@dataclass
class ScanQualityMetrics:
    """Simplified scan quality metrics for ROS2 node."""
    point_count: int = 0
    scan_density: float = 0.0
    angular_coverage: float = 0.0
    max_gap_size: float = 0.0
    valid_range_ratio: float = 0.0
    mean_range: float = 0.0
    scan_rate: float = 0.0
    quality_score: float = 1.0
    quality_level: str = "good"
    issues: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    timestamp: float = 0.0


class ScanQualityMonitor(Node):
    def __init__(self):
        super().__init__('scan_quality_monitor')

        # Declare parameters
        self.declare_parameter('ws_port', 9002)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('min_point_count', 100)
        self.declare_parameter('min_scan_density', 0.5)
        self.declare_parameter('min_angular_coverage', 0.7)
        self.declare_parameter('max_allowed_gap', 45.0)
        self.declare_parameter('expected_scan_rate', 10.0)
        self.declare_parameter('report_rate_hz', 1.0)
        self.declare_parameter('enable_monitoring', False)

        # Get parameters
        self.ws_port = self.get_parameter('ws_port').value
        scan_topic = self.get_parameter('scan_topic').value
        self.min_point_count = self.get_parameter('min_point_count').value
        self.min_scan_density = self.get_parameter('min_scan_density').value
        self.min_angular_coverage = self.get_parameter('min_angular_coverage').value
        self.max_allowed_gap = self.get_parameter('max_allowed_gap').value
        self.expected_scan_rate = self.get_parameter('expected_scan_rate').value
        self.report_rate = self.get_parameter('report_rate_hz').value
        self.enable_monitoring = self.get_parameter('enable_monitoring').value

        # Add parameter callback for runtime toggling
        self.add_on_set_parameters_callback(self._parameter_callback)

        # Subscribe to scan topic
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self._scan_callback,
            10
        )

        # Timer for periodic quality reports
        self.report_timer = self.create_timer(
            1.0 / self.report_rate,
            self._publish_quality_report
        )

        # Quality tracking
        self.latest_metrics = None
        self.metrics_history = []
        self.max_history = 100
        self.last_scan_time = 0.0
        self.metrics_lock = threading.Lock()

        # Connected WebSocket clients
        self._ws_clients = set()

        self.get_logger().info(f'Scan quality monitor starting on port {self.ws_port}')
        self.get_logger().info(f'Monitoring topic: {scan_topic}')
        self.get_logger().info(f'Quality thresholds: points>={self.min_point_count}, '
                             f'coverage>={self.min_angular_coverage:.1%}, '
                             f'gap<={self.max_allowed_gap}°')

        # Start WebSocket server in separate thread
        self.ws_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
        self.ws_thread.start()

    def _parameter_callback(self, params):
        """Handle parameter changes at runtime."""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'enable_monitoring':
                old_value = self.enable_monitoring
                self.enable_monitoring = param.value
                if old_value != self.enable_monitoring:
                    self.get_logger().info(
                        f'Monitoring {"enabled" if self.enable_monitoring else "disabled"}'
                    )

        return SetParametersResult(successful=True)

    def _scan_callback(self, msg: LaserScan):
        """Callback for scan topic - analyze quality."""
        # Skip analysis if monitoring is disabled
        if not self.enable_monitoring:
            return

        current_time = datetime.now().timestamp()

        # Extract scan data
        ranges = list(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(ranges))]

        # Analyze quality
        metrics = self._analyze_scan_quality(
            ranges=ranges,
            angles=angles,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            range_min=msg.range_min,
            range_max=msg.range_max,
            current_time=current_time
        )

        with self.metrics_lock:
            self.latest_metrics = metrics
            self.metrics_history.append(metrics)
            if len(self.metrics_history) > self.max_history:
                self.metrics_history.pop(0)

        # Log quality issues
        if metrics.quality_level in ["poor", "critical"]:
            self.get_logger().warn(
                f'Scan quality {metrics.quality_level.upper()}: score={metrics.quality_score:.2f}, '
                f'points={metrics.point_count}, coverage={metrics.angular_coverage:.1%}'
            )
            for issue in metrics.issues:
                self.get_logger().warn(f'  Issue: {issue}')

    def _analyze_scan_quality(
        self,
        ranges: List[float],
        angles: List[float],
        angle_min: float,
        angle_max: float,
        range_min: float,
        range_max: float,
        current_time: float
    ) -> ScanQualityMetrics:
        """Analyze scan quality and return metrics."""
        metrics = ScanQualityMetrics()
        metrics.timestamp = current_time

        if not ranges:
            metrics.quality_level = "critical"
            metrics.quality_score = 0.0
            metrics.issues.append("No scan data received")
            return metrics

        # Convert to numpy for efficient computation
        ranges_arr = np.array(ranges)
        angles_arr = np.array(angles)

        # Filter valid ranges (finite and within bounds)
        valid_mask = (ranges_arr >= range_min) & (ranges_arr <= range_max) & np.isfinite(ranges_arr)
        valid_ranges = ranges_arr[valid_mask]
        valid_angles = angles_arr[valid_mask]

        # Point count metrics
        metrics.point_count = len(valid_ranges)
        expected_points = len(ranges)

        if metrics.point_count < self.min_point_count:
            metrics.issues.append(
                f"Low point count: {metrics.point_count} < {self.min_point_count}"
            )
        elif metrics.point_count < expected_points * 0.5:
            metrics.warnings.append(
                f"Point count below 50%: {metrics.point_count}/{expected_points}"
            )

        # Scan density metrics
        if len(valid_angles) > 0:
            angular_span = float(np.max(valid_angles) - np.min(valid_angles))
            if angular_span > 0:
                metrics.scan_density = len(valid_angles) / np.degrees(angular_span)

            if metrics.scan_density < self.min_scan_density:
                metrics.issues.append(
                    f"Low scan density: {metrics.scan_density:.2f} < {self.min_scan_density}"
                )

        # Angular coverage analysis (360 degree bins)
        num_bins = 360
        bin_size = 2 * np.pi / num_bins
        bins = np.zeros(num_bins, dtype=bool)

        for angle in valid_angles:
            # Normalize angle to [0, 2π)
            normalized_angle = (angle - angle_min) % (2 * np.pi)
            bin_idx = int(normalized_angle / bin_size) % num_bins
            bins[bin_idx] = True

        metrics.angular_coverage = float(np.sum(bins)) / num_bins

        # Find largest gap
        max_gap = 0
        current_gap = 0
        for i in range(num_bins * 2):  # Check twice to handle wraparound
            if not bins[i % num_bins]:
                current_gap += 1
                max_gap = max(max_gap, current_gap)
            else:
                current_gap = 0

        metrics.max_gap_size = max_gap * (360.0 / num_bins)

        if metrics.angular_coverage < self.min_angular_coverage:
            metrics.issues.append(
                f"Low coverage: {metrics.angular_coverage:.1%} < {self.min_angular_coverage:.1%}"
            )

        if metrics.max_gap_size > self.max_allowed_gap:
            metrics.warnings.append(
                f"Large gap: {metrics.max_gap_size:.1f}° > {self.max_allowed_gap}°"
            )

        # Range quality metrics
        if len(valid_ranges) > 0:
            metrics.mean_range = float(np.mean(valid_ranges))
            metrics.valid_range_ratio = len(valid_ranges) / len(ranges)

            if metrics.valid_range_ratio < 0.5:
                metrics.warnings.append(
                    f"Low valid range ratio: {metrics.valid_range_ratio:.1%}"
                )

        # Scan rate
        if self.last_scan_time > 0:
            time_delta = current_time - self.last_scan_time
            if time_delta > 0:
                metrics.scan_rate = 1.0 / time_delta

                rate_error = abs(metrics.scan_rate - self.expected_scan_rate) / self.expected_scan_rate
                if rate_error > 0.2:  # 20% tolerance
                    metrics.warnings.append(
                        f"Scan rate deviation: {metrics.scan_rate:.1f}Hz "
                        f"(expected {self.expected_scan_rate}Hz)"
                    )

        self.last_scan_time = current_time

        # Calculate overall quality score
        score = 1.0

        # Point count (30%)
        point_score = min(1.0, metrics.point_count / (expected_points * 0.8))
        # Coverage (30%)
        coverage_score = min(1.0, metrics.angular_coverage / 0.95)
        # Density (20%)
        density_score = min(1.0, metrics.scan_density / 1.0)
        # Valid range ratio (20%)
        range_score = metrics.valid_range_ratio

        score = 0.30 * point_score + 0.30 * coverage_score + 0.20 * density_score + 0.20 * range_score

        # Apply penalties
        if len(metrics.issues) > 0:
            score *= 0.8
        if len(metrics.issues) > 2:
            score *= 0.7

        metrics.quality_score = score

        # Determine quality level
        if score >= 0.9:
            metrics.quality_level = "excellent"
        elif score >= 0.75:
            metrics.quality_level = "good"
        elif score >= 0.5:
            metrics.quality_level = "fair"
        elif score >= 0.3:
            metrics.quality_level = "poor"
        else:
            metrics.quality_level = "critical"

        return metrics

    def _publish_quality_report(self):
        """Publish periodic quality report via WebSocket."""
        with self.metrics_lock:
            if not self.latest_metrics:
                return

            report = self._generate_quality_report()

        # Broadcast to all connected clients
        asyncio.run(self._broadcast_report(report))

    def _generate_quality_report(self) -> Dict[str, Any]:
        """Generate quality report from latest metrics."""
        if not self.latest_metrics:
            return {
                "type": "scan_quality",
                "status": "no_data",
                "timestamp": datetime.now().timestamp()
            }

        m = self.latest_metrics

        # Calculate averages from recent history
        recent = self.metrics_history[-10:] if len(self.metrics_history) >= 10 else self.metrics_history
        avg_score = sum(r.quality_score for r in recent) / len(recent) if recent else 0.0
        avg_points = sum(r.point_count for r in recent) / len(recent) if recent else 0.0

        return {
            "type": "scan_quality",
            "status": "ok",
            "timestamp": m.timestamp,
            "latest": {
                "quality_level": m.quality_level,
                "quality_score": round(m.quality_score, 3),
                "point_count": m.point_count,
                "scan_density": round(m.scan_density, 3),
                "angular_coverage": round(m.angular_coverage, 3),
                "max_gap_size": round(m.max_gap_size, 1),
                "valid_range_ratio": round(m.valid_range_ratio, 3),
                "mean_range": round(m.mean_range, 2),
                "scan_rate": round(m.scan_rate, 1),
                "issues": m.issues,
                "warnings": m.warnings
            },
            "averages": {
                "quality_score": round(avg_score, 3),
                "point_count": round(avg_points, 1)
            },
            "config": {
                "min_point_count": self.min_point_count,
                "min_scan_density": self.min_scan_density,
                "min_angular_coverage": self.min_angular_coverage,
                "max_allowed_gap": self.max_allowed_gap,
                "expected_scan_rate": self.expected_scan_rate
            }
        }

    async def _broadcast_report(self, report: Dict[str, Any]):
        """Broadcast quality report to all clients."""
        if not self._ws_clients:
            return

        message = json.dumps(report)
        disconnected = set()

        for client in self._ws_clients:
            try:
                await client.send(message)
            except:
                disconnected.add(client)

        self._ws_clients -= disconnected

    def _run_websocket_server(self):
        """Run WebSocket server."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def handler(websocket):
            self._ws_clients.add(websocket)
            self.get_logger().info(f'Quality monitor client connected: {len(self._ws_clients)} total')

            try:
                # Send latest report immediately
                with self.metrics_lock:
                    if self.latest_metrics:
                        report = self._generate_quality_report()
                        await websocket.send(json.dumps(report))

                # Keep connection alive
                async for message in websocket:
                    try:
                        msg = json.loads(message)
                        if msg.get('type') == 'ping':
                            await websocket.send(json.dumps({
                                'type': 'pong',
                                'ts': msg.get('ts', 0)
                            }))
                    except:
                        pass

            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                self._ws_clients.discard(websocket)
                self.get_logger().info(f'Quality monitor client disconnected: {len(self._ws_clients)} total')

        async def start_server():
            async with websockets.serve(handler, '0.0.0.0', self.ws_port):
                await asyncio.Future()

        loop.run_until_complete(start_server())


def main(args=None):
    rclpy.init(args=args)
    node = ScanQualityMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
