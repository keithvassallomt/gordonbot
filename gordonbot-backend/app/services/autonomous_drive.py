"""Autonomous driving primitives for self-driving features.

This module contains all autonomous navigation code ported from selfdrive_sandbox.py.
It is completely separate from manual control (joystick/WASD) in sockets/control.py.

DO NOT modify manual control code - use this module for autonomous features.
"""

from __future__ import annotations

import asyncio
import logging
import math
import time
from dataclasses import dataclass
from typing import TYPE_CHECKING

from app.services.sensors import get_sensor_status

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


def set_motors(left: float, right: float) -> None:
    """Drive motors immediately with normalized inputs (-1..1) for autonomous routines."""
    left_clamped = _clamp(float(left))
    right_clamped = _clamp(float(right))
    process_drive_command(left_clamped, right_clamped, source="autonomous")


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
