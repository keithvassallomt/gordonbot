"""Autonomous driving primitives for self-driving features.

This module contains all autonomous navigation code ported from selfdrive_sandbox.py.
It is completely separate from manual control (joystick/WASD) in sockets/control.py.

DO NOT modify manual control code - use this module for autonomous features.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Optional

from app.core.config import settings
from app.services.sensors import get_sensor_status
from app.sockets import slam as slam_socket

if TYPE_CHECKING:
    from fastapi import WebSocket

log = logging.getLogger(__name__)

# Import motor controller from control module
# We only use it for autonomous driving, never touching manual control logic
from app.sockets.control import process_drive_command, stop_all_motors


# ---- Constants mirrored from sandbox ------------------------------

COMMAND_HZ = 20.0
MAX_SPEED = 1.0
BOOST_MULTIPLIER = 1.5
KEY_ACCEL_PER_S = 1.2
KEY_DECEL_PER_S = 2.0
KEY_TURN_ACCEL_PER_S = 1.2
KEY_TURN_DECEL_PER_S = 2.0
_TICK_INTERVAL = 1.0 / COMMAND_HZ
_SENSOR_INTERVAL = 0.12  # 120ms, matches sandbox sleep(120)

_TOF_STOP_DISTANCE_MM = max(30, int(settings.tof_alert_threshold_mm) + 10)
_GO_TO_MAX_DURATION_S = 120.0
_GO_TO_STALL_TIMEOUT_S = 6.0
_GO_TO_POSE_TIMEOUT_S = 1.0
_GO_TO_FORWARD_MAX = 0.7  # Matches CREEP_FORWARD_SPEED (no longer cubed)
_GO_TO_TURN_MAX = 0.8     # Matches CREEP_TURN_SPEED (no longer cubed)
_GO_TO_FORWARD_MIN_ACTIVE = 0.7  # Robot requires 0.7 minimum to overcome friction


def set_motors(left: float, right: float) -> None:
    """Drive motors immediately with normalized inputs (-1..1) for autonomous routines."""
    left_clamped = _clamp(float(left))
    right_clamped = _clamp(float(right))
    process_drive_command(left_clamped, right_clamped, source="autonomous", shape=False)


def stop_motors() -> None:
    """Stop both motors and reset drive state for autonomous routines."""
    _drive_controller.reset()
    stop_all_motors()


def _approach(current: float, target: float, accel_per_s: float, decel_per_s: float, dt: float) -> float:
    """Replica of sandbox smoothing: accelerate/decelerate toward the target."""
    rate = accel_per_s if abs(target) > abs(current) else decel_per_s
    max_delta = rate * dt
    delta = max(-max_delta, min(max_delta, target - current))
    next_val = _clamp(current + delta)
    if abs(next_val - target) < 1e-4:
        return target
    return next_val


@dataclass
class _DriveState:
    turn: float = 0.0
    forward: float = 0.0
    boost: bool = False


class _DriveController:
    """Smoothing pipeline that mirrors sandbox tick + setDriveVector behaviour."""

    def __init__(self) -> None:
        self._state = _DriveState()
        self._vec_turn = 0.0
        self._vec_forward = 0.0
        self._last_update = time.monotonic()

    def reset(self) -> None:
        self._state = _DriveState()
        self._vec_turn = 0.0
        self._vec_forward = 0.0
        self._last_update = time.monotonic()

    def set_target(self, turn: float, forward: float, *, boost: bool = False) -> None:
        self._state.turn = _clamp(turn)
        self._state.forward = _clamp(forward)
        self._state.boost = boost

    def step(self) -> tuple[float, float]:
        now = time.monotonic()
        dt = max(0.0, min(0.05, now - self._last_update))
        self._last_update = now

        self._vec_turn = _approach(
            self._vec_turn,
            self._state.turn,
            KEY_TURN_ACCEL_PER_S,
            KEY_TURN_DECEL_PER_S,
            dt,
        )
        self._vec_forward = _approach(
            self._vec_forward,
            self._state.forward,
            KEY_ACCEL_PER_S,
            KEY_DECEL_PER_S,
            dt,
        )

        left = _clamp(self._vec_forward + self._vec_turn)
        right = _clamp(self._vec_forward - self._vec_turn)
        multiplier = MAX_SPEED * (BOOST_MULTIPLIER if self._state.boost else 1.0)
        left = _clamp(left * multiplier)
        right = _clamp(right * multiplier)
        set_motors(left, right)
        return left, right


_drive_controller = _DriveController()


# --- Helper functions from sandbox ---


def _normalize_heading(deg: float) -> float:
    """Normalize heading to 0-360 range."""
    return ((deg % 360) + 360) % 360


def _normalize_angle_diff(deg: float) -> float:
    """Normalize angle difference to -180 to 180 range."""
    return ((deg + 180) % 360 + 360) % 360 - 180


def _normalize_angle_rad(angle: float) -> float:
    """Normalize angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, min_val: float = -1.0, max_val: float = 1.0) -> float:
    """Clamp value to range."""
    return max(min_val, min(max_val, value))


# --- Circle driving from sandbox (EXACT port) ---


async def drive_circle(
    ws: WebSocket,
    stop_event: asyncio.Event,
    direction: str,
    radius: float,
    start_yaw: float,
) -> tuple[bool, float]:
    """
    Drive a single circle (left or right) - EXACT port from selfdrive_sandbox.py.

    Args:
        ws: WebSocket (unused but kept for API compatibility)
        stop_event: Event to signal abort
        direction: "left" or "right"
        radius: Circle radius in meters
        start_yaw: Starting heading in degrees

    Returns:
        (success: bool, travelled: float in meters)
    """
    track_width = 0.13  # approximate wheel track (m)
    circumference = 2 * math.pi * radius
    forward_throttle = 0.6
    wheel_ratio = (radius + track_width / 2) / max(0.05, radius - track_width / 2)
    base_turn_magnitude = forward_throttle * (wheel_ratio - 1) / (wheel_ratio + 1)
    base_turn = (
        -_clamp(base_turn_magnitude, 0.05, 0.35)
        if direction == "left"
        else _clamp(base_turn_magnitude, 0.05, 0.35)
    )

    log.info(
        "%s circle: radius=%.2fm, circumference=%.2fm, base_turn=%.3f",
        direction.capitalize(),
        radius,
        circumference,
        base_turn,
    )

    # Snapshot encoder distances for travelled computation
    sensors = get_sensor_status()
    start_left = sensors.encoders.left.distance_m if sensors.encoders and sensors.encoders.left else None
    start_right = sensors.encoders.right.distance_m if sensors.encoders and sensors.encoders.right else None
    start_avg = None
    if start_left is not None and start_right is not None:
        start_avg = (start_left + start_right) / 2.0

    travelled = 0.0
    last_log_distance = 0.0

    current_turn = base_turn
    _drive_controller.set_target(current_turn, forward_throttle)
    sensor_elapsed = _SENSOR_INTERVAL  # force immediate poll on first iteration

    try:
        while not stop_event.is_set():
            # Mirror sandbox 20 Hz command loop
            _drive_controller.step()
            await asyncio.sleep(_TICK_INTERVAL)
            sensor_elapsed += _TICK_INTERVAL
            if sensor_elapsed + 1e-9 < _SENSOR_INTERVAL:
                continue
            sensor_elapsed = 0.0

            # Poll sensors
            sensors = get_sensor_status()
            if stop_event.is_set():
                break

            # Calculate travelled distance
            dist_left = sensors.encoders.left.distance_m if sensors.encoders and sensors.encoders.left else None
            dist_right = sensors.encoders.right.distance_m if sensors.encoders and sensors.encoders.right else None
            dist_avg = None
            if dist_left is not None and dist_right is not None:
                dist_avg = (dist_left + dist_right) / 2.0

            # Compute travelled using same priority logic as sandbox
            if dist_avg is not None and start_avg is not None:
                travelled = abs(dist_avg - start_avg)
            elif (
                dist_left is not None
                and start_left is not None
                and dist_right is not None
                and start_right is not None
            ):
                travelled = (abs(dist_left - start_left) + abs(dist_right - start_right)) / 2.0
            elif dist_left is not None and start_left is not None:
                travelled = abs(dist_left - start_left)
            elif dist_right is not None and start_right is not None:
                travelled = abs(dist_right - start_right)

            if travelled is not None and math.isfinite(travelled):
                progress = min(travelled / circumference, 1.0)

                # Get yaw for heading correction
                yaw = None
                if sensors.bno055 and sensors.bno055.euler and sensors.bno055.euler.yaw is not None:
                    yaw = _normalize_heading(sensors.bno055.euler.yaw)

                if yaw is not None:
                    yaw_delta = -progress * 360 if direction == "left" else progress * 360
                    desired_yaw = _normalize_heading(start_yaw + yaw_delta)
                    yaw_err = _normalize_angle_diff(desired_yaw - yaw)
                    correction = _clamp(yaw_err / 45.0, -0.3, 0.3)
                    current_turn = _clamp(base_turn + correction, -0.8, 0.8)
                    _drive_controller.set_target(current_turn, forward_throttle)
                else:
                    current_turn = base_turn
                    _drive_controller.set_target(current_turn, forward_throttle)

                # Log progress every 100mm
                if travelled - last_log_distance >= 0.1:
                    log.debug(
                        "%s circle progress: %dmm/%dmm",
                        direction.capitalize(),
                        int(travelled * 1000),
                        int(circumference * 1000),
                    )
                    last_log_distance = travelled

                if travelled >= circumference:
                    break

        return (not stop_event.is_set(), travelled)
    except Exception as exc:
        log.exception("Circle drive failed: %s", exc)
        return (False, travelled)
    finally:
        # Smoothly ramp down like sandbox stopDrive()
        _drive_controller.set_target(0.0, 0.0)
        for _ in range(4):
            _drive_controller.step()
            await asyncio.sleep(_TICK_INTERVAL)
        stop_motors()


async def drive_figure_eight(ws: WebSocket, stop_event: asyncio.Event) -> bool:
    """
    Execute figure-8 calibration pattern - EXACT port from selfdrive_sandbox.py.
    Drives left circle → pause → right circle.

    Args:
        ws: WebSocket for sending status updates
        stop_event: Event to signal abort

    Returns:
        True if successful, False if aborted or failed
    """
    success = True
    try:
        log.info("Starting figure-8 pattern (radius 20cm each loop)")

        # Poll sensors once
        await asyncio.sleep(0.2)
        sensors = get_sensor_status()

        # Get initial heading
        heading = None
        if sensors.bno055 and sensors.bno055.euler and sensors.bno055.euler.yaw is not None:
            heading = sensors.bno055.euler.yaw
        if heading is None:
            log.warning("No IMU heading available, using 0° as reference for figure-8")
            heading = 0.0
        else:
            log.info("Figure-8 starting heading: %.1f°", heading)

        start_yaw = _normalize_heading(heading)

        # First loop: left circle
        log.info("─────── Loop 1/2: Left Circle ───────")
        left_success, left_travelled = await drive_circle(ws, stop_event, "left", 0.20, start_yaw)
        if stop_event.is_set() or not left_success:
            success = False
            log.warning("Left circle incomplete")

        if success and not stop_event.is_set():
            await asyncio.sleep(0.3)

            # Update heading after first circle
            sensors = get_sensor_status()
            mid_heading = None
            if sensors.bno055 and sensors.bno055.euler and sensors.bno055.euler.yaw is not None:
                mid_heading = sensors.bno055.euler.yaw
            if mid_heading is None:
                mid_heading = start_yaw
            else:
                mid_heading = _normalize_heading(mid_heading)

            log.info("Completed left circle, current heading: %.1f°", mid_heading)

            # Second loop: right circle
            log.info("─────── Loop 2/2: Right Circle ───────")
            right_success, right_travelled = await drive_circle(ws, stop_event, "right", 0.20, mid_heading)
            if stop_event.is_set() or not right_success:
                success = False
                log.warning("Right circle incomplete")

        await asyncio.sleep(0.15)

        if stop_event.is_set():
            log.warning("Figure-8 drive aborted by user")
        elif success:
            log.info("Figure-8 pattern complete!")
        else:
            log.error("Figure-8 drive error")

        return success
    except Exception as exc:
        log.exception("Figure-8 pattern failed: %s", exc)
        return False
    finally:
        stop_motors()


# --- Go-to point navigation ----------------------------------------------------


@dataclass
class GoToResult:
    success: bool
    reason: str
    distance_remaining: Optional[float]
    elapsed_s: float
    tof_distance_mm: Optional[int] = None
    completed_at: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "reason": self.reason,
            "distance_remaining": self.distance_remaining,
            "elapsed_s": self.elapsed_s,
            "tof_distance_mm": self.tof_distance_mm,
            "completed_at": self.completed_at,
        }


@dataclass
class GoToStatus:
    state: str
    target: Optional[tuple[float, float]] = None
    tolerance: Optional[float] = None
    distance_remaining: Optional[float] = None
    initial_distance: Optional[float] = None
    started_at: Optional[float] = None
    elapsed_s: Optional[float] = None
    success: Optional[bool] = None
    reason: Optional[str] = None
    tof_distance_mm: Optional[int] = None
    finished_at: Optional[float] = None

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "state": self.state,
            "tolerance": self.tolerance,
            "distance_remaining": self.distance_remaining,
            "initial_distance": self.initial_distance,
            "started_at": self.started_at,
            "elapsed_s": self.elapsed_s,
            "success": self.success,
            "reason": self.reason,
            "tof_distance_mm": self.tof_distance_mm,
            "finished_at": self.finished_at,
        }
        payload["target"] = {"x": self.target[0], "y": self.target[1]} if self.target else None
        return {k: v for k, v in payload.items() if v is not None or k in {"state", "target"}}


@dataclass
class _GoToContext:
    target_x: float
    target_y: float
    tolerance: float
    initial_distance: float
    started_at_wall: float
    started_at_monotonic: float
    stop_event: asyncio.Event
    task: asyncio.Task[GoToResult]


@dataclass
class _GoToHistory:
    result: GoToResult
    target_x: float
    target_y: float
    tolerance: float
    initial_distance: float
    started_at_wall: float
    started_at_monotonic: float


class GoToError(Exception):
    """Base class for go-to navigation errors."""


class GoToValidationError(GoToError):
    """Raised when the requested go-to target is invalid."""


class GoToInProgressError(GoToError):
    """Raised when a go-to command is already running."""


class GoToNotRunningError(GoToError):
    """Raised when attempting to cancel a go-to command that is not active."""


_GO_TO_LOCK = asyncio.Lock()
_GO_TO_CONTEXT: Optional[_GoToContext] = None
_GO_TO_LAST: Optional[_GoToHistory] = None


def _lookup_map_cell(map_msg: "SlamMapMessage", x: float, y: float) -> Optional[int]:
    """Return occupancy value for world coordinate, or None if outside map."""
    dx = (x - map_msg.origin.x) / map_msg.resolution
    dy = (y - map_msg.origin.y) / map_msg.resolution
    col = int(math.floor(dx))
    row = int(math.floor(dy))
    if col < 0 or col >= map_msg.width or row < 0 or row >= map_msg.height:
        return None
    idx = row * map_msg.width + col
    if idx < 0 or idx >= len(map_msg.data):
        return None
    try:
        return int(map_msg.data[idx])
    except (ValueError, TypeError):
        return None


async def drive_to_point(
    target_x: float,
    target_y: float,
    stop_event: asyncio.Event,
    *,
    tolerance: float = 0.1,
    max_duration_s: float = _GO_TO_MAX_DURATION_S,
) -> GoToResult:
    """
    Drive the robot toward a target point in the SLAM map frame.

    Uses SLAM pose feedback and ToF safety stop. Returns the drive outcome.
    """
    log.info(
        "Go-to-point routine start target=(%.3f, %.3f)m tolerance=%.3f m",
        target_x,
        target_y,
        tolerance,
    )
    drive_start_monotonic = time.monotonic()
    best_distance = float("inf")
    progress_reset = drive_start_monotonic
    pose_last_seen = drive_start_monotonic
    tof_trigger_mm: Optional[int] = None
    final_distance: Optional[float] = None
    result: Optional[GoToResult] = None
    reason = "unknown"
    success = False
    sensor_elapsed = _SENSOR_INTERVAL

    try:
        while True:
            if stop_event.is_set():
                reason = "cancelled"
                break

            pose = slam_socket.get_latest_pose()
            now = time.monotonic()

            if pose is not None:
                pose_last_seen = now
                dx = target_x - float(pose.x)
                dy = target_y - float(pose.y)
                distance = math.hypot(dx, dy)
                final_distance = distance

                if distance <= tolerance:
                    success = True
                    reason = "reached"
                    stop_motors()
                    break

                heading_error = _normalize_angle_rad(math.atan2(dy, dx) - float(pose.theta))
                heading_abs = abs(heading_error)

                # Binary movement: robot needs 0.7 to overcome friction, so either go full or stop
                # Turn command: positive = turn right, negative = turn left
                turn_sign = 1.0 if heading_error > 0 else -1.0

                # Forward logic: only move if roughly aligned and not too close
                if distance <= tolerance * 1.5:
                    # Very close to target - stop and let tolerance check handle completion
                    forward_target = 0.0
                    turn_target = 0.0
                elif heading_abs > math.radians(45):
                    # Need significant heading correction - turn in place at full speed
                    forward_target = 0.0
                    turn_target = turn_sign * _GO_TO_TURN_MAX
                elif heading_abs > math.radians(10):
                    # Small correction while moving
                    forward_target = _GO_TO_FORWARD_MAX
                    turn_target = turn_sign * _GO_TO_TURN_MAX * 0.5
                else:
                    # Well aligned - go straight
                    forward_target = _GO_TO_FORWARD_MAX
                    turn_target = 0.0

                # Apply commands directly without ramping controller
                left = _clamp(forward_target + turn_target)
                right = _clamp(forward_target - turn_target)
                set_motors(left, right)

                if distance + 1e-3 < best_distance:
                    best_distance = distance
                    progress_reset = now
            else:
                stop_motors()

            await asyncio.sleep(_TICK_INTERVAL)

            sensor_elapsed += _TICK_INTERVAL
            if sensor_elapsed + 1e-9 >= _SENSOR_INTERVAL:
                sensor_elapsed = 0.0
                sensors = get_sensor_status()
                tof_mm = None
                if sensors.tof and sensors.tof.distance_mm is not None:
                    tof_mm = int(sensors.tof.distance_mm)
                if tof_mm is not None and tof_mm <= _TOF_STOP_DISTANCE_MM:
                    tof_trigger_mm = tof_mm
                    reason = "tof_stop"
                    break

            now = time.monotonic()
            if now - pose_last_seen > _GO_TO_POSE_TIMEOUT_S:
                reason = "pose_unavailable"
                break

            if now - progress_reset > _GO_TO_STALL_TIMEOUT_S:
                reason = "stalled"
                break

            if now - drive_start_monotonic > max_duration_s:
                reason = "timeout"
                break

        stop_monotonic = time.monotonic()
        stop_wall = time.time()
        result = GoToResult(
            success=success,
            reason=reason,
            distance_remaining=final_distance,
            elapsed_s=stop_monotonic - drive_start_monotonic,
            tof_distance_mm=tof_trigger_mm,
            completed_at=stop_wall,
        )

        if success:
            log.info(
                "Go-to-point reached target distance=%.3f m elapsed=%.1f s",
                final_distance if final_distance is not None else 0.0,
                result.elapsed_s,
            )
        else:
            log.warning(
                "Go-to-point stopped reason=%s distance=%s elapsed=%.1f s",
                reason,
                f"{final_distance:.3f} m" if final_distance is not None else "unknown",
                result.elapsed_s,
            )
    except Exception as exc:
        stop_monotonic = time.monotonic()
        stop_wall = time.time()
        log.exception("Go-to-point routine failed: %s", exc)
        result = GoToResult(
            success=False,
            reason="exception",
            distance_remaining=final_distance,
            elapsed_s=stop_monotonic - drive_start_monotonic,
            tof_distance_mm=tof_trigger_mm,
            completed_at=stop_wall,
        )
    finally:
        stop_motors()

    return result


async def start_go_to_point(
    target_x: float,
    target_y: float,
    *,
    tolerance: float = 0.1,
) -> GoToStatus:
    """
    Start an asynchronous go-to routine toward the requested SLAM coordinate.
    """
    map_msg = slam_socket.get_latest_map()
    pose = slam_socket.get_latest_pose()

    if map_msg is None:
        raise GoToValidationError("SLAM map not available")
    if pose is None:
        raise GoToValidationError("SLAM pose not available")

    occupancy = _lookup_map_cell(map_msg, target_x, target_y)
    if occupancy is None:
        raise GoToValidationError("Target lies outside the known map bounds")
    if occupancy == -1:
        raise GoToValidationError("Target cell is unknown space")
    if occupancy >= 65:
        raise GoToValidationError("Target cell is occupied")

    initial_distance = math.hypot(target_x - float(pose.x), target_y - float(pose.y))
    if initial_distance <= tolerance:
        raise GoToValidationError("Already within tolerance of the requested point")

    async with _GO_TO_LOCK:
        global _GO_TO_CONTEXT
        if _GO_TO_CONTEXT is not None and not _GO_TO_CONTEXT.task.done():
            raise GoToInProgressError("A go-to routine is already running")

        stop_event = asyncio.Event()
        started_wall = time.time()
        started_monotonic = time.monotonic()

        async def _runner() -> GoToResult:
            return await drive_to_point(target_x, target_y, stop_event, tolerance=tolerance)

        task = asyncio.create_task(_runner(), name="go-to-point")
        context = _GoToContext(
            target_x=target_x,
            target_y=target_y,
            tolerance=tolerance,
            initial_distance=initial_distance,
            started_at_wall=started_wall,
            started_at_monotonic=started_monotonic,
            stop_event=stop_event,
            task=task,
        )

        _GO_TO_CONTEXT = context

        def _store_result(fut: asyncio.Task[GoToResult]) -> None:
            global _GO_TO_CONTEXT, _GO_TO_LAST

            try:
                res = fut.result()
            except Exception as exc:  # pragma: no cover - defensive
                log.exception("Go-to-point task crashed: %s", exc)
                stop_monotonic = time.monotonic()
                stop_wall = time.time()
                res = GoToResult(
                    success=False,
                    reason="exception",
                    distance_remaining=None,
                    elapsed_s=stop_monotonic - context.started_at_monotonic,
                    tof_distance_mm=None,
                    completed_at=stop_wall,
                )

            _GO_TO_LAST = _GoToHistory(
                result=res,
                target_x=context.target_x,
                target_y=context.target_y,
                tolerance=context.tolerance,
                initial_distance=context.initial_distance,
                started_at_wall=context.started_at_wall,
                started_at_monotonic=context.started_at_monotonic,
            )
            _GO_TO_CONTEXT = None

        task.add_done_callback(_store_result)

    log.info(
        "Go-to-point task launched target=(%.3f, %.3f)m tolerance=%.3f m start_distance=%.3f m",
        target_x,
        target_y,
        tolerance,
        initial_distance,
    )

    return GoToStatus(
        state="running",
        target=(target_x, target_y),
        tolerance=tolerance,
        distance_remaining=initial_distance,
        initial_distance=initial_distance,
        started_at=started_wall,
        elapsed_s=0.0,
    )


async def cancel_go_to_point(*, wait: bool = True, timeout: float = 3.0) -> bool:
    """
    Cancel an active go-to routine. Returns True if a cancellation was issued.
    """
    task: Optional[asyncio.Task[GoToResult]] = None

    async with _GO_TO_LOCK:
        context = _GO_TO_CONTEXT
        if context is None or context.task.done():
            raise GoToNotRunningError("No go-to routine is currently running")

        context.stop_event.set()
        task = context.task

    if wait and task is not None:
        with contextlib.suppress(asyncio.TimeoutError):
            await asyncio.wait_for(task, timeout=timeout)

    return True


async def get_go_to_status() -> GoToStatus:
    """
    Return the current go-to routine status or the result of the last run.
    """
    async with _GO_TO_LOCK:
        context = _GO_TO_CONTEXT
        history = _GO_TO_LAST

    pose = slam_socket.get_latest_pose()
    now_monotonic = time.monotonic()

    if context is not None and not context.task.done():
        distance_remaining = None
        if pose is not None:
            distance_remaining = math.hypot(
                context.target_x - float(pose.x),
                context.target_y - float(pose.y),
            )
        elapsed = now_monotonic - context.started_at_monotonic
        return GoToStatus(
            state="running",
            target=(context.target_x, context.target_y),
            tolerance=context.tolerance,
            distance_remaining=distance_remaining,
            initial_distance=context.initial_distance,
            started_at=context.started_at_wall,
            elapsed_s=elapsed,
        )

    if history is not None:
        result = history.result
        return GoToStatus(
            state="idle",
            target=(history.target_x, history.target_y),
            tolerance=history.tolerance,
            distance_remaining=result.distance_remaining,
            initial_distance=history.initial_distance,
            started_at=history.started_at_wall,
            elapsed_s=result.elapsed_s,
            success=result.success,
            reason=result.reason,
            tof_distance_mm=result.tof_distance_mm,
            finished_at=result.completed_at,
        )

    return GoToStatus(state="idle")
