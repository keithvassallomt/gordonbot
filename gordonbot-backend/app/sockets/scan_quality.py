from __future__ import annotations

import asyncio
import logging
import json
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
        clients.discard(websocket)
        log.info(f"Scan quality WebSocket client removed, total clients: {len(clients)}")


def start_quality_monitor_connection():
    """Start background task to connect to scan_quality_monitor."""
    global quality_monitor_task

    if quality_monitor_task is None or quality_monitor_task.done():
        loop = asyncio.get_event_loop()
        quality_monitor_task = loop.create_task(connect_to_quality_monitor())
        log.info("Started scan_quality_monitor connection task")


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
