from __future__ import annotations

import asyncio
import logging
import json
from typing import Optional, Set

from pydantic import ValidationError

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import websockets

log = logging.getLogger(__name__)

from app.schemas import SlamMapMessage, SlamPoseMessage, LoopClosureEvent
from app.services.map_quality_analyzer import default_analyzer
import math
import time

router = APIRouter()

# Store connected clients
clients: Set[WebSocket] = set()

# Latest data from map_bridge
latest_map: Optional[dict] = None
latest_pose: Optional[dict] = None
latest_map_obj: Optional[SlamMapMessage] = None
latest_pose_obj: Optional[SlamPoseMessage] = None

# Map quality analysis
latest_map_quality: Optional[dict] = None

# Loop closure detection
previous_pose: Optional[SlamPoseMessage] = None
previous_pose_time: Optional[float] = None
loop_closure_events: list[dict] = []  # Recent loop closure events
MAX_LOOP_EVENTS = 50  # Keep last 50 events

# Connection to map_bridge WebSocket
map_bridge_ws: Optional[websockets.WebSocketClientProtocol] = None
map_bridge_task: Optional[asyncio.Task] = None


def detect_loop_closure(current_pose: SlamPoseMessage) -> Optional[LoopClosureEvent]:
    """
    Detect potential loop closure by analyzing pose corrections.

    Loop closures cause the robot's estimated position to "jump" when SLAM
    corrects accumulated drift. We detect this by comparing expected pose
    (based on smooth motion) vs actual pose.

    Returns LoopClosureEvent if a correction is detected, None otherwise.
    """
    global previous_pose, previous_pose_time

    current_time = time.time()

    # Need previous pose for comparison
    if previous_pose is None or previous_pose_time is None:
        previous_pose = current_pose
        previous_pose_time = current_time
        return None

    # Calculate time delta
    dt = current_time - previous_pose_time

    # Ignore if updates are too close together (< 50ms) or too far apart (> 5s)
    if dt < 0.05 or dt > 5.0:
        previous_pose = current_pose
        previous_pose_time = current_time
        return None

    # Calculate position change
    dx = current_pose.x - previous_pose.x
    dy = current_pose.y - previous_pose.y
    distance = math.sqrt(dx**2 + dy**2)

    # Calculate angular change
    dtheta = current_pose.theta - previous_pose.theta
    # Normalize to [-pi, pi]
    while dtheta > math.pi:
        dtheta -= 2 * math.pi
    while dtheta < -math.pi:
        dtheta += 2 * math.pi

    # Expected maximum movement based on robot capabilities
    # GordonBot max speed ~0.3 m/s, so in dt seconds:
    max_expected_distance = 0.35 * dt  # Add 15% margin
    max_expected_rotation = 2.0 * dt  # ~115 deg/s max

    # Detect anomalous jumps (likely loop closure corrections)
    is_position_jump = distance > max_expected_distance and distance > 0.05  # At least 5cm
    is_rotation_jump = abs(dtheta) > max_expected_rotation and abs(dtheta) > 0.1  # At least ~6 degrees

    # Update previous pose
    previous_pose = current_pose
    previous_pose_time = current_time

    # If either position or rotation jumped significantly
    if is_position_jump or is_rotation_jump:
        # Determine confidence based on magnitude
        confidence = "low"
        if distance > 0.2 or abs(dtheta) > 0.3:  # 20cm or 17 degrees
            confidence = "medium"
        if distance > 0.5 or abs(dtheta) > 0.5:  # 50cm or 29 degrees
            confidence = "high"

        event = LoopClosureEvent(
            ts=current_pose.ts,
            x=current_pose.x,
            y=current_pose.y,
            correction_distance=distance,
            correction_angle=abs(dtheta),
            confidence=confidence
        )

        log.info(f"Loop closure detected: {distance:.3f}m, {abs(dtheta):.3f}rad ({confidence} confidence)")
        return event

    return None


async def connect_to_map_bridge():
    """Connect to ROS2 map_bridge WebSocket and relay data."""
    global latest_map, latest_pose, latest_map_obj, latest_pose_obj, latest_map_quality, map_bridge_ws

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

                            # Analyze map structure quality (Phase 2)
                            try:
                                map_metrics = default_analyzer.analyze_map_structure(msg)
                                latest_map_quality = {
                                    "explored_ratio": map_metrics.explored_ratio,
                                    "occupied_ratio": map_metrics.occupied_ratio,
                                    "entropy": map_metrics.entropy,
                                    "noise_score": map_metrics.noise_score,
                                    "wall_sharpness": map_metrics.wall_sharpness,
                                    "feature_density": map_metrics.feature_density,
                                    "corner_count": map_metrics.corner_count,
                                    "edge_count": map_metrics.edge_count,
                                    "quality_level": map_metrics.quality_level.value,
                                    "quality_score": map_metrics.quality_score,
                                    "issues": map_metrics.issues,
                                    "warnings": map_metrics.warnings,
                                    "has_walls": map_metrics.has_walls,
                                    "is_empty": map_metrics.is_empty
                                }
                            except Exception as e:
                                log.debug(f"Map quality analysis failed: {e}")
                                latest_map_quality = None

                            # Broadcast to all connected clients
                            await broadcast_to_clients(msg)

                        elif msg_type == "pose":
                            latest_pose = msg
                            try:
                                latest_pose_obj = SlamPoseMessage.model_validate(msg)
                            except ValidationError as exc:
                                log.debug("Failed to validate SLAM pose message: %s", exc)
                                latest_pose_obj = None

                            # Detect loop closures from pose corrections
                            if latest_pose_obj is not None:
                                loop_event = detect_loop_closure(latest_pose_obj)
                                if loop_event is not None:
                                    # Add to event history
                                    global loop_closure_events
                                    event_dict = loop_event.model_dump()
                                    loop_closure_events.append(event_dict)
                                    # Keep only recent events
                                    if len(loop_closure_events) > MAX_LOOP_EVENTS:
                                        loop_closure_events = loop_closure_events[-MAX_LOOP_EVENTS:]
                                    # Broadcast loop closure event
                                    await broadcast_to_clients(event_dict)

                            # Broadcast pose to all connected clients
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

        # Send recent loop closure events
        for event in loop_closure_events:
            await websocket.send_json(event)

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


def get_latest_map_quality() -> Optional[dict]:
    """Return the most recent map quality analysis, if available."""
    if latest_map_quality is None:
        return None
    return latest_map_quality.copy()
