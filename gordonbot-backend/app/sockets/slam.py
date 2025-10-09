from __future__ import annotations

import asyncio
import logging
import json
from typing import Optional, Set

from pydantic import ValidationError

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import websockets

log = logging.getLogger(__name__)

from app.schemas import SlamMapMessage, SlamPoseMessage
router = APIRouter()

# Store connected clients
clients: Set[WebSocket] = set()

# Latest data from map_bridge
latest_map: Optional[dict] = None
latest_pose: Optional[dict] = None
latest_map_obj: Optional[SlamMapMessage] = None
latest_pose_obj: Optional[SlamPoseMessage] = None

# Connection to map_bridge WebSocket
map_bridge_ws: Optional[websockets.WebSocketClientProtocol] = None
map_bridge_task: Optional[asyncio.Task] = None


async def connect_to_map_bridge():
    """Connect to ROS2 map_bridge WebSocket and relay data."""
    global latest_map, latest_pose, latest_map_obj, latest_pose_obj, map_bridge_ws

    map_bridge_url = "ws://localhost:9001"

    while True:
        try:
            log.info(f"Connecting to map_bridge at {map_bridge_url}...")
            async with websockets.connect(map_bridge_url) as websocket:
                map_bridge_ws = websocket
                log.info("Connected to map_bridge WebSocket")

                while True:
                    try:
                        # Receive data from map_bridge
                        data = await asyncio.wait_for(websocket.recv(), timeout=30.0)
                        msg = json.loads(data)

                        msg_type = msg.get("type")

                        if msg_type == "map":
                            latest_map = msg
                            try:
                                latest_map_obj = SlamMapMessage.model_validate(msg)
                            except ValidationError as exc:
                                log.debug("Failed to validate SLAM map message: %s", exc)
                                latest_map_obj = None
                            # Broadcast to all connected clients
                            await broadcast_to_clients(msg)

                        elif msg_type == "pose":
                            latest_pose = msg
                            try:
                                latest_pose_obj = SlamPoseMessage.model_validate(msg)
                            except ValidationError as exc:
                                log.debug("Failed to validate SLAM pose message: %s", exc)
                                latest_pose_obj = None
                            # Broadcast to all connected clients
                            await broadcast_to_clients(msg)

                    except asyncio.TimeoutError:
                        # Send ping to keep connection alive
                        await websocket.send(json.dumps({"type": "ping"}))

        except Exception as e:
            log.error(f"map_bridge connection error: {e}, reconnecting in 5s...")
            map_bridge_ws = None
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


@router.websocket("/ws/slam")
async def slam_websocket(websocket: WebSocket) -> None:
    """
    WebSocket endpoint for streaming SLAM data (map + pose).

    Relays data from ROS2 map_bridge to frontend clients.

    Message formats:
    - Map: {"type": "map", "ts": <ms>, "width": <int>, "height": <int>,
            "resolution": <float>, "origin": {...}, "data": [int, ...]}
    - Pose: {"type": "pose", "ts": <ms>, "x": <float>, "y": <float>,
             "theta": <float>, "frame_id": "map"}

    Client can send ping: {"type": "ping", "ts": <ms>}
    Server responds: {"type": "pong", "ts": <same>}
    """
    await websocket.accept()
    clients.add(websocket)
    log.info(f"SLAM WebSocket client connected, total clients: {len(clients)}")

    try:
        # Send latest data immediately on connect
        if latest_map:
            await websocket.send_json(latest_map)
        if latest_pose:
            await websocket.send_json(latest_pose)

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
        log.info("SLAM WebSocket client disconnected")
    except Exception as e:
        log.error(f"SLAM WebSocket error: {e}", exc_info=True)
    finally:
        clients.discard(websocket)
        log.info(f"SLAM WebSocket client removed, total clients: {len(clients)}")


def start_map_bridge_connection():
    """Start background task to connect to map_bridge."""
    global map_bridge_task

    if map_bridge_task is None or map_bridge_task.done():
        loop = asyncio.get_event_loop()
        map_bridge_task = loop.create_task(connect_to_map_bridge())
        log.info("Started map_bridge connection task")


def get_latest_pose() -> Optional[SlamPoseMessage]:
    """Return the most recent SLAM pose message, if available."""
    if latest_pose_obj is None:
        return None
    return latest_pose_obj.model_copy()


def get_latest_map() -> Optional[SlamMapMessage]:
    """Return the most recent SLAM map message, if available."""
    if latest_map_obj is None:
        return None
    return latest_map_obj.model_copy()
