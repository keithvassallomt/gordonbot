from __future__ import annotations

import asyncio
import logging
import time
from typing import Optional
from dataclasses import dataclass

log = logging.getLogger(__name__)

try:
    from rplidarc1 import RPLidar
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False
    log.warning("rplidarc1 library not available; LIDAR service will be disabled")


@dataclass
class LidarPoint:
    """Single LIDAR measurement point."""
    angle: float  # degrees (0-360)
    distance_mm: float  # millimeters
    quality: int  # signal quality (0-63 for C1)


@dataclass
class LidarScan:
    """Complete 360° LIDAR scan."""
    ts: int  # timestamp in milliseconds
    points: list[LidarPoint]
    scan_rate_hz: float


class LidarService:
    """
    Service for managing RPLIDAR C1 connection and scan data.

    This service connects to the RPLIDAR C1 via serial port and continuously
    reads scan data. It maintains a buffer of the most recent complete scan
    for consumers (REST API, WebSocket).
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 460800,
        timeout: float = 0.2,
    ):
        """
        Initialize LIDAR service.

        Args:
            port: Serial port path (e.g., '/dev/ttyUSB0')
            baudrate: Serial baudrate (default 460800 for C1)
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self._lidar: Optional[RPLidar] = None
        self._connected = False
        self._running = False
        self._scan_task: Optional[asyncio.Task] = None
        self._queue_task: Optional[asyncio.Task] = None

        # Most recent complete scan
        self._latest_scan: Optional[LidarScan] = None
        self._scan_lock = asyncio.Lock()

        # Scan rate tracking
        self._last_scan_time: float = 0.0
        self._scan_rate_hz: float = 0.0

        # Diagnostic stats
        self._total_points_received: int = 0
        self._points_filtered_angle: int = 0
        self._points_filtered_distance: int = 0
        self._points_filtered_quality: int = 0
        self._last_error: Optional[str] = None

    @property
    def connected(self) -> bool:
        """Check if LIDAR is connected."""
        return self._connected

    @property
    def running(self) -> bool:
        """Check if scan loop is running."""
        return self._running

    async def connect(self) -> bool:
        """
        Connect to LIDAR device.

        Returns:
            True if connection successful, False otherwise
        """
        if not RPLIDAR_AVAILABLE:
            log.error("rplidarc1 library not available")
            return False

        if self._connected:
            log.warning("LIDAR already connected")
            return True

        try:
            log.info(f"Connecting to RPLIDAR C1 on {self.port} at {self.baudrate} baud")

            # Create LIDAR instance
            self._lidar = RPLidar(self.port, self.baudrate, self.timeout)
            log.info("LIDAR instance created successfully")

            # Note: get_info() and healthcheck() are not fully implemented in rplidarc1 library
            # The device connection will be verified when we start scanning

            self._connected = True
            return True

        except Exception as e:
            log.error(f"Failed to connect to LIDAR: {e}")
            self._lidar = None
            self._connected = False
            return False

    async def disconnect(self) -> None:
        """Disconnect from LIDAR device."""
        if self._running:
            await self.stop_scan()

        if self._lidar is not None:
            try:
                await asyncio.to_thread(self._lidar.shutdown)
                log.info("LIDAR disconnected")
            except Exception as e:
                log.error(f"Error disconnecting LIDAR: {e}")
            finally:
                self._lidar = None
                self._connected = False

    async def start_scan(self) -> bool:
        """
        Start LIDAR scanning.

        Returns:
            True if scan started successfully, False otherwise
        """
        if not self._connected or self._lidar is None:
            log.error("LIDAR not connected; cannot start scan")
            return False

        if self._running:
            log.warning("LIDAR scan already running")
            return True

        try:
            self._running = True

            # Start the scan coroutine
            self._scan_task = asyncio.create_task(
                self._lidar.simple_scan(make_return_dict=True)
            )

            # Start queue processing task
            self._queue_task = asyncio.create_task(self._process_queue())

            log.info("LIDAR scan started")
            return True

        except Exception as e:
            log.error(f"Failed to start LIDAR scan: {e}")
            self._running = False
            return False

    async def stop_scan(self) -> None:
        """Stop LIDAR scanning."""
        if not self._running:
            return

        self._running = False

        # Signal stop event
        if self._lidar is not None:
            self._lidar.stop_event.set()

        # Cancel tasks
        if self._queue_task is not None:
            self._queue_task.cancel()
            try:
                await self._queue_task
            except asyncio.CancelledError:
                pass
            self._queue_task = None

        if self._scan_task is not None:
            try:
                await self._scan_task
            except asyncio.CancelledError:
                pass
            self._scan_task = None

        log.info("LIDAR scan stopped")

    async def _process_queue(self) -> None:
        """
        Process scan data from the LIDAR output queue.

        Accumulates points into complete scans and updates latest_scan.
        """
        if self._lidar is None:
            return

        try:
            log.info("Starting LIDAR queue processor")

            # Buffer for accumulating points in current scan
            current_scan_points: list[LidarPoint] = []
            scan_start_time = time.time()
            last_angle = 0.0
            last_publish_time = time.time()

            while self._running:
                try:
                    # Get point from queue with timeout
                    point = await asyncio.wait_for(
                        self._lidar.output_queue.get(),
                        timeout=1.0
                    )

                    # Point format: {"a_deg": angle, "d_mm": distance, "q": quality}
                    angle = point.get("a_deg", 0.0)
                    distance = point.get("d_mm", 0.0)
                    quality = point.get("q", 0)

                    self._total_points_received += 1

                    # Filter out invalid points (library protocol parsing errors)
                    # Note: angles from 0 to <360 are valid; 360.0+ are errors
                    # Handle None values from library
                    if angle is None or angle < 0 or angle >= 360:
                        self._points_filtered_angle += 1
                        continue  # Skip invalid angles
                    if distance is None or distance <= 0 or distance > 12000:  # C1 max range is 12m
                        self._points_filtered_distance += 1
                        continue  # Skip invalid distances
                    if quality is None or quality <= 0:
                        self._points_filtered_quality += 1
                        continue  # Skip zero-quality points

                    # Detect scan completion:
                    # Method 1: Angle wrap-around (preferred)
                    # Method 2: Time-based fallback (if stuck)
                    current_time = time.time()
                    scan_duration = current_time - scan_start_time

                    wrap_detected = (current_scan_points and angle < 10 and last_angle > 350)
                    timeout_reached = (scan_duration > 0.15 and len(current_scan_points) > 50)  # ~150ms for 10Hz

                    if wrap_detected or timeout_reached:
                        # Complete scan detected
                        scan_end_time = time.time()
                        scan_duration = scan_end_time - scan_start_time
                        self._scan_rate_hz = 1.0 / scan_duration if scan_duration > 0 else 0.0

                        # Store completed scan
                        async with self._scan_lock:
                            self._latest_scan = LidarScan(
                                ts=int(scan_end_time * 1000),
                                points=current_scan_points.copy(),
                                scan_rate_hz=self._scan_rate_hz,
                            )

                        # Reset for next scan
                        current_scan_points = []
                        scan_start_time = scan_end_time

                    # Add point to current scan
                    current_scan_points.append(
                        LidarPoint(
                            angle=angle,
                            distance_mm=distance,
                            quality=quality,
                        )
                    )
                    last_angle = angle

                except asyncio.TimeoutError:
                    # No data in queue, continue waiting
                    # If we have partial scan and haven't published in a while, publish it
                    if current_scan_points and (time.time() - last_publish_time) > 0.5:
                        log.warning(f"Publishing partial scan ({len(current_scan_points)} points) due to timeout")
                        async with self._scan_lock:
                            self._latest_scan = LidarScan(
                                ts=int(time.time() * 1000),
                                points=current_scan_points.copy(),
                                scan_rate_hz=self._scan_rate_hz,
                            )
                        last_publish_time = time.time()
                    continue

        except asyncio.CancelledError:
            log.info("LIDAR queue processor cancelled")
            raise
        except Exception as e:
            error_msg = f"Error in LIDAR queue processor: {e}"
            log.error(error_msg, exc_info=True)
            self._last_error = error_msg
            self._running = False
            # Don't raise - let the service stay connected for restart

    async def get_latest_scan(self) -> Optional[LidarScan]:
        """
        Get the most recent complete LIDAR scan.

        Returns:
            Latest scan data, or None if no scan available
        """
        async with self._scan_lock:
            return self._latest_scan

    async def get_status(self) -> dict:
        """
        Get LIDAR status information.

        Returns:
            Dictionary with status information
        """
        status = {
            "connected": self._connected,
            "running": self._running,
            "scan_rate_hz": self._scan_rate_hz,
            "port": self.port,
        }

        if self._lidar is not None and self._connected:
            # Note: healthcheck() not fully implemented in rplidarc1, so report based on connection state
            status["health"] = "Connected" if self._running else "Idle"
        else:
            status["health"] = "Disconnected"

        return status

    async def get_diagnostics(self) -> dict:
        """
        Get detailed diagnostic information for debugging.

        Returns:
            Dictionary with diagnostic stats
        """
        total_filtered = (
            self._points_filtered_angle +
            self._points_filtered_distance +
            self._points_filtered_quality
        )

        valid_points = self._total_points_received - total_filtered

        return {
            "total_points_received": self._total_points_received,
            "valid_points": valid_points,
            "filtered_points": {
                "total": total_filtered,
                "angle": self._points_filtered_angle,
                "distance": self._points_filtered_distance,
                "quality": self._points_filtered_quality,
            },
            "filter_rate_percent": (
                (total_filtered / self._total_points_received * 100)
                if self._total_points_received > 0 else 0
            ),
            "last_error": self._last_error,
            "scan_task_alive": self._scan_task is not None and not self._scan_task.done(),
            "queue_task_alive": self._queue_task is not None and not self._queue_task.done(),
            "latest_scan_point_count": len(self._latest_scan.points) if self._latest_scan else 0,
        }


# Global LIDAR service instance
_lidar_service: Optional[LidarService] = None


def get_lidar_service() -> Optional[LidarService]:
    """Get the global LIDAR service instance."""
    return _lidar_service


def set_lidar_service(service: Optional[LidarService]) -> None:
    """Set the global LIDAR service instance."""
    global _lidar_service
    _lidar_service = service
