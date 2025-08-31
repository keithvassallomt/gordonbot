from __future__ import annotations
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.schemas import DriveMessage
import asyncio
import time
import logging
import contextlib

router = APIRouter()

# --- Hardware abstraction layer (safe to run on dev machines) ---
try:
    from gpiozero import Motor as GPIOMotor  # Raspberry Pi
    _HW_AVAILABLE = True
except Exception:
    GPIOMotor = None
    _HW_AVAILABLE = False

class MotorController:
    """Simple 2-motor tank drive controller with -1.0..1.0 inputs."""
    def __init__(self):
        self._log = logging.getLogger(__name__)
        if _HW_AVAILABLE:
            self.left = GPIOMotor(forward=5, backward=6, pwm=True)
            self.right = GPIOMotor(forward=19, backward=13, pwm=True)
            self._log.info("GPIOZero motors initialised (pins: L fwd=5,bwd=6 | R fwd=13,bwd=19)")
        else:
            self.left = None
            self.right = None
            self._log.warning("GPIOZero not available; running in no-hardware mode")

    @staticmethod
    def _apply(motor, value: float) -> None:
        # value in [-1, 1]
        if value > 0:
            motor.forward(value)
        elif value < 0:
            motor.backward(-value)
        else:
            motor.stop()

    def set(self, left: float, right: float) -> None:
        if not _HW_AVAILABLE:
            # Dev mode: just log
            logging.getLogger(__name__).debug("(DEV) motors.set left=%.2f right=%.2f", left, right)
            return
        self._apply(self.left, left)
        self._apply(self.right, right)

    def stop(self) -> None:
        if not _HW_AVAILABLE:
            logging.getLogger(__name__).debug("(DEV) motors.stop")
            return
        self.left.stop()
        self.right.stop()

# Module-level singleton
_motors = MotorController()

@router.websocket("/ws/control")
async def ws_control(ws: WebSocket):
    await ws.accept()
    log = logging.getLogger(__name__)

    # Dead-man timer: stop if no command within this window
    DEADMAN_SECONDS = 0.4
    last_cmd = {"t": time.monotonic()}  # mutable holder for closure

    async def watchdog():
        try:
            while True:
                await asyncio.sleep(0.2)
                if time.monotonic() - last_cmd["t"] > DEADMAN_SECONDS:
                    _motors.stop()
        except asyncio.CancelledError:
            # Task cancelled when connection closes
            pass

    wd_task = asyncio.create_task(watchdog())

    try:
        while True:
            data = await ws.receive_json()

            # Support simple heartbeat/ping messages
            if isinstance(data, dict) and data.get("type") == "ping":
                await ws.send_json({"type": "pong", "ts": data.get("ts")})
                continue

            msg = DriveMessage.model_validate(data)

            # Clamp for safety (server-side)
            left = max(-1.0, min(1.0, msg.payload.left))
            right = max(-1.0, min(1.0, msg.payload.right))
            _ts = msg.payload.ts

            # Optional response curve to soften centre and give finer control
            def shape(x: float) -> float:
                # cubic curve preserves sign, emphasises small inputs
                return x * x * x

            shaped_left = shape(left)
            shaped_right = shape(right)

            # Drive motors
            _motors.set(left=shaped_left, right=shaped_right)
            last_cmd["t"] = time.monotonic()

            # Ack back to client (useful for latency/telemetry)
            await ws.send_json({
                "type": "ack",
                "ts": _ts,
                "echo": {"left": left, "right": right}
            })
    except WebSocketDisconnect:
        log.info("WebSocket disconnected; stopping motors")
    except Exception as e:
        log.exception("Error in ws_control: %s", e)
        # Best effort to notify client (if still connected)
        try:
            await ws.send_json({"type": "error", "message": str(e)})
        except Exception:
            pass
    finally:
        wd_task.cancel()
        with contextlib.suppress(Exception):
            await wd_task
        _motors.stop()