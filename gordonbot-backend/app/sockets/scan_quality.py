from __future__ import annotations

import asyncio
import logging
import json
import subprocess
import time
from typing import Optional, Set

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import websockets

log = logging.getLogger(__name__)

router = APIRouter()

# Store connected clients
clients: Set[WebSocket] = set()

# Latest quality data from ROS2 scan_quality_monitor
latest_quality: Optional[dict] = None

# Import slam socket to get map quality
_slam_socket_imported = False

# Connection to ROS2 scan_quality_monitor WebSocket
quality_monitor_ws: Optional[websockets.WebSocketClientProtocol] = None
quality_monitor_task: Optional[asyncio.Task] = None

# Monitoring control state
monitoring_enabled: bool = False
manual_override: bool = False
last_disconnect_time: Optional[float] = None
DEBOUNCE_SECONDS = 30  # Wait 30 seconds after last client disconnects before stopping

# Background task for debounce checking
debounce_task: Optional[asyncio.Task] = None


async def set_ros2_monitoring_parameter(enabled: bool) -> bool:
    """
    Set the enable_monitoring parameter on the ROS2 scan_quality_monitor node.

    Args:
        enabled: True to enable monitoring, False to disable

    Returns:
        True if successful, False otherwise
    """
    try:
        # Call ros2 param set command
        # Note: This requires the ROS2 container to be accessible
        cmd = [
            "docker", "exec", "gordonbot-ros2-slam",
            "bash", "-c",
            f"source /opt/ros/humble/setup.bash && "
            f"ros2 param set /scan_quality_monitor enable_monitoring {str(enabled).lower()}"
        ]

        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode == 0:
            log.info(f"ROS2 parameter set successfully: enable_monitoring={enabled}")
            return True
        else:
            log.warning(f"Failed to set ROS2 parameter: {result.stderr}")
            return False

    except subprocess.TimeoutExpired:
        log.error("Timeout setting ROS2 parameter")
        return False
    except Exception as e:
        log.error(f"Error setting ROS2 parameter: {e}")
        return False


async def enable_monitoring_internal():
    """Internal function to actually start the monitoring connection."""
    global monitoring_enabled, quality_monitor_task

    if monitoring_enabled:
        log.info("Monitoring already enabled")
        return True

    monitoring_enabled = True
    log.info("Enabling scan quality monitoring")

    # Set ROS2 parameter to enable monitoring
    await set_ros2_monitoring_parameter(True)

    # Start the connection task if not already running
    if quality_monitor_task is None or quality_monitor_task.done():
        loop = asyncio.get_event_loop()
        quality_monitor_task = loop.create_task(connect_to_quality_monitor())
        log.info("Started scan_quality_monitor connection task")

    return True


async def disable_monitoring_internal():
    """Internal function to actually stop the monitoring connection."""
    global monitoring_enabled, quality_monitor_task, quality_monitor_ws

    if not monitoring_enabled:
        log.info("Monitoring already disabled")
        return True

    monitoring_enabled = False
    log.info("Disabling scan quality monitoring")

    # Set ROS2 parameter to disable monitoring
    await set_ros2_monitoring_parameter(False)

    # Close WebSocket connection to ROS2 node
    if quality_monitor_ws:
        try:
            await quality_monitor_ws.close()
        except Exception as e:
            log.debug(f"Error closing quality monitor websocket: {e}")
        quality_monitor_ws = None

    # Cancel the connection task
    if quality_monitor_task and not quality_monitor_task.done():
        quality_monitor_task.cancel()
        try:
            await quality_monitor_task
        except asyncio.CancelledError:
            pass
        quality_monitor_task = None

    return True


async def enable_monitoring(manual: bool = False) -> bool:
    """
    Enable scan quality monitoring.

    Args:
        manual: If True, sets manual override flag (persists until manually disabled)

    Returns:
        True if successful
    """
    global manual_override, last_disconnect_time

    if manual:
        manual_override = True
        log.info("Manual override enabled")

    last_disconnect_time = None  # Clear debounce timer
    return await enable_monitoring_internal()


async def disable_monitoring(manual: bool = False) -> bool:
    """
    Disable scan quality monitoring.

    Args:
        manual: If True from manual request, clears manual override

    Returns:
        True if successful
    """
    global manual_override

    if manual:
        manual_override = False
        log.info("Manual override disabled")

    # If there are still clients or manual override is active, don't actually disable
    if len(clients) > 0 and not manual:
        log.info("Not disabling monitoring: clients still connected")
        return False

    if manual_override and not manual:
        log.info("Not disabling monitoring: manual override active")
        return False

    return await disable_monitoring_internal()


async def check_debounce_timer():
    """Background task to check if we should disable monitoring after debounce period."""
    global last_disconnect_time

    while True:
        await asyncio.sleep(5)  # Check every 5 seconds

        if last_disconnect_time is not None:
            elapsed = time.time() - last_disconnect_time
            if elapsed >= DEBOUNCE_SECONDS:
                log.info(f"Debounce period ({DEBOUNCE_SECONDS}s) elapsed with no clients, stopping monitoring")
                last_disconnect_time = None
                await disable_monitoring()


def get_monitoring_status() -> dict:
    """Get current monitoring status."""
    return {
        "enabled": monitoring_enabled,
        "client_count": len(clients),
        "manual_override": manual_override,
    }


async def connect_to_quality_monitor():
    """Connect to ROS2 scan_quality_monitor WebSocket and relay data."""
    global latest_quality, quality_monitor_ws

    monitor_url = "ws://localhost:9002"

    while True:
        try:
            log.info(f"Connecting to scan_quality_monitor at {monitor_url}...")
            async with websockets.connect(monitor_url) as websocket:
                quality_monitor_ws = websocket
                log.info("Connected to scan_quality_monitor WebSocket")

                while True:
                    try:
                        # Receive data from quality monitor
                        data = await asyncio.wait_for(websocket.recv(), timeout=30.0)
                        msg = json.loads(data)

                        if msg.get("type") == "scan_quality":
                            latest_quality = msg
                            # Broadcast to all connected clients
                            await broadcast_to_clients(msg)

                    except asyncio.TimeoutError:
                        # Send ping to keep connection alive
                        await websocket.send(json.dumps({"type": "ping"}))

        except Exception as e:
            log.error(f"scan_quality_monitor connection error: {e}, reconnecting in 5s...")
            quality_monitor_ws = None
            await asyncio.sleep(5)


async def broadcast_to_clients(message: dict):
    """Broadcast message to all connected WebSocket clients."""
    if not clients:
        return

    disconnected = set()
    message_str = json.dumps(message)

    for client in clients:
        try:
            await client.send_text(message_str)
        except Exception:
            disconnected.add(client)

    # Remove disconnected clients
    clients.difference_update(disconnected)


@router.websocket("/ws/scan-quality")
async def scan_quality_websocket(websocket: WebSocket) -> None:
    """
    WebSocket endpoint for streaming scan quality data.

    Relays data from ROS2 scan_quality_monitor to frontend clients.

    Message format:
    {
      "type": "scan_quality",
      "status": "ok",
      "timestamp": <float>,
      "latest": {
        "quality_level": "good" | "excellent" | "fair" | "poor" | "critical",
        "quality_score": <float>,
        "point_count": <int>,
        "scan_density": <float>,
        "angular_coverage": <float>,
        "max_gap_size": <float>,
        "valid_range_ratio": <float>,
        "mean_range": <float>,
        "scan_rate": <float>,
        "issues": [<string>, ...],
        "warnings": [<string>, ...]
      },
      "averages": {
        "quality_score": <float>,
        "point_count": <float>
      },
      "config": {
        "min_point_count": <int>,
        "min_scan_density": <float>,
        "min_angular_coverage": <float>,
        "max_allowed_gap": <float>,
        "expected_scan_rate": <float>
      }
    }

    Client can send ping: {"type": "ping", "ts": <ms>}
    Server responds: {"type": "pong", "ts": <same>}
    """
    await websocket.accept()
    clients.add(websocket)
    log.info(f"Scan quality WebSocket client connected, total clients: {len(clients)}")

    # Auto-enable monitoring when first client connects
    if len(clients) == 1:
        log.info("First client connected, auto-enabling monitoring")
        await enable_monitoring()

    try:
        # Send latest data immediately on connect
        if latest_quality:
            await websocket.send_json(latest_quality)

        # Listen for client messages (ping/pong)
        while True:
            data = await websocket.receive_text()

            try:
                msg = json.loads(data)

                # Handle ping/pong
                if msg.get("type") == "ping":
                    await websocket.send_json({
                        "type": "pong",
                        "ts": msg.get("ts", 0),
                    })

            except json.JSONDecodeError:
                log.warning(f"Invalid JSON received: {data}")

    except WebSocketDisconnect:
        log.info("Scan quality WebSocket client disconnected")
    except Exception as e:
        log.error(f"Scan quality WebSocket error: {e}", exc_info=True)
    finally:
        global last_disconnect_time
        clients.discard(websocket)
        log.info(f"Scan quality WebSocket client removed, total clients: {len(clients)}")

        # Start debounce timer when last client disconnects
        if len(clients) == 0 and not manual_override:
            log.info(f"Last client disconnected, starting {DEBOUNCE_SECONDS}s debounce timer")
            last_disconnect_time = time.time()


def start_quality_monitor_connection():
    """
    Start background task to connect to scan_quality_monitor.

    NOTE: This function is now deprecated. The monitor connection starts automatically
    when the first WebSocket client connects. This function remains for backward compatibility
    but does nothing.
    """
    global debounce_task

    # Start the debounce checker task
    if debounce_task is None or debounce_task.done():
        loop = asyncio.get_event_loop()
        debounce_task = loop.create_task(check_debounce_timer())
        log.info("Started debounce checker task")

    # Do NOT auto-start monitoring - it will start when clients connect
    log.info("Scan quality monitoring configured (auto-start on client connection)")


def get_latest_quality() -> Optional[dict]:
    """Return the most recent scan quality data, if available."""
    global _slam_socket_imported

    if latest_quality is None:
        return None

    result = latest_quality.copy()

    # Add map quality if available
    try:
        if not _slam_socket_imported:
            from app.sockets import slam as slam_socket
            _slam_socket_imported = True
        else:
            import app.sockets.slam as slam_socket

        map_quality = slam_socket.get_latest_map_quality()
        if map_quality:
            result["map_quality"] = map_quality
    except Exception as e:
        log.debug(f"Could not get map quality: {e}")

    return result
