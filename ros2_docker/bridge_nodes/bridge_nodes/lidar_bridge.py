#!/usr/bin/env python3
"""
LIDAR Bridge Node - Subscribes to GordonBot LIDAR WebSocket and publishes to ROS2 /scan topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import asyncio
import websockets
import json
import math
import threading
from datetime import datetime


class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge')

        # Declare parameters
        self.declare_parameter('backend_url', 'ws://localhost:8000')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('angle_min', 0.0)
        self.declare_parameter('angle_max', 2.0 * math.pi)

        # Get parameters
        backend_url = self.get_parameter('backend_url').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # WebSocket URL
        self.ws_url = f"{backend_url}/ws/lidar"

        self.get_logger().info(f'LIDAR bridge starting, connecting to {self.ws_url}')

        # Start WebSocket client in separate thread
        self.ws_thread = threading.Thread(target=self._run_websocket_client, daemon=True)
        self.ws_thread.start()

    def _run_websocket_client(self):
        """Run WebSocket client in asyncio event loop."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._websocket_client())

    async def _websocket_client(self):
        """WebSocket client that receives LIDAR scans and publishes to ROS2."""
        while rclpy.ok():
            try:
                async with websockets.connect(self.ws_url) as websocket:
                    self.get_logger().info('Connected to LIDAR WebSocket')

                    while rclpy.ok():
                        try:
                            # Receive scan data
                            data = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                            msg = json.loads(data)

                            if msg.get('type') == 'scan':
                                self._publish_scan(msg)

                        except asyncio.TimeoutError:
                            # Send ping to keep connection alive
                            await websocket.send(json.dumps({
                                'type': 'ping',
                                'ts': int(datetime.now().timestamp() * 1000)
                            }))

            except Exception as e:
                self.get_logger().warn(f'WebSocket error: {e}, reconnecting in 2s...')
                await asyncio.sleep(2.0)

    def _publish_scan(self, msg):
        """Convert LIDAR scan message to LaserScan and publish."""
        points = msg.get('points', [])

        if not points:
            return

        # Create LaserScan message
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        # RPLIDAR C1 specs
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = (2.0 * math.pi) / 360.0  # 1 degree resolution
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / msg.get('scan_rate_hz', 10.0)
        scan.range_min = 0.15  # 15cm minimum
        scan.range_max = 12.0  # 12m maximum

        # Initialize ranges and intensities arrays (360 points)
        num_points = 360
        ranges = [float('inf')] * num_points
        intensities = [0.0] * num_points

        # Fill in the data from points
        for point in points:
            angle_deg = point.get('angle', 0.0)
            distance_mm = point.get('distance_mm', 0.0)
            quality = point.get('quality', 0)

            # Convert to index (0-359)
            index = int(angle_deg) % 360

            # Convert distance mm to meters
            distance_m = distance_mm / 1000.0

            # Only use valid measurements
            if scan.range_min <= distance_m <= scan.range_max:
                ranges[index] = distance_m
                intensities[index] = float(quality)

        scan.ranges = ranges
        scan.intensities = intensities

        # Publish
        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
