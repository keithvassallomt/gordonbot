from __future__ import annotations
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.core.config import settings
from app.schemas import DriveMessage
from app.services.drive_state import set_drive as _set_drive_state
import asyncio
import time
import logging
import contextlib

router = APIRouter()

log = logging.getLogger(__name__)

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
        self.hw = False
        self.left = None
        self.right = None
        if _HW_AVAILABLE:
            try:
                self.left = GPIOMotor(forward=5, backward=6, pwm=True)
                self.right = GPIOMotor(forward=19, backward=13, pwm=True)
                self.hw = True
                self._log.info("GPIOZero motors initialised (pins: L fwd=5,bwd=6 | R fwd=19,bwd=13)")
                # Log detected pin factory if available
                try:
                    from gpiozero import devices as _devices  # type: ignore
                    pf = getattr(_devices.Device, "pin_factory", None)
                    self._log.debug("GPIOZero pin factory: %s", pf)
                except Exception:
                    pass
            except Exception as e:
                # GPIOZero present but no working pin factory (e.g., missing lgpio/pigpio, permissions)
                self.left = None
                self.right = None
                self.hw = False
                self._log.warning("GPIOZero available but pin factory failed (%s); falling back to no-hardware mode", e)
        else:
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
        if not self.hw:
            # Dev mode: just log
            logging.getLogger(__name__).debug("(DEV) motors.set left=%.3f right=%.3f", left, right)
            return
        logging.getLogger(__name__).debug("motors.set left=%.3f right=%.3f", left, right)
        self._apply(self.left, left)
        self._apply(self.right, right)

    def stop(self) -> None:
        if not self.hw:
            logging.getLogger(__name__).debug("(DEV) motors.stop")
            return
        logging.getLogger(__name__).debug("motors.stop")
        self.left.stop()
        self.right.stop()

# Module-level singleton
_motors = MotorController()

@router.websocket(settings.control_ws_path)
async def ws_control(ws: WebSocket):
    await ws.accept()
    log.info("WS connected to %s", settings.control_ws_path)

    # Dead-man timer: stop if no command within this window
    DEADMAN_SECONDS = 0.4
    last_cmd = {"t": time.monotonic()}  # mutable holder for closure
    wd_state = {"tripped": False}

    async def watchdog():
        try:
            while True:
                await asyncio.sleep(0.2)
                if time.monotonic() - last_cmd["t"] > DEADMAN_SECONDS:
                    if not wd_state["tripped"]:
                        log.warning("watchdog: dead-man timeout %.1fs reached; stopping", DEADMAN_SECONDS)
                        wd_state["tripped"] = True
                    _motors.stop()
                else:
                    # Reset tripped flag once commands resume
                    if wd_state["tripped"]:
                        log.debug("watchdog: commands resumed")
                        wd_state["tripped"] = False
        except asyncio.CancelledError:
            # Task cancelled when connection closes
            log.debug("watchdog: cancelled")

    wd_task = asyncio.create_task(watchdog())

    try:
        while True:
            data = await ws.receive_json()
            log.debug("recv: %s", data)

            # Support simple heartbeat/ping messages
            if isinstance(data, dict) and data.get("type") == "ping":
                log.debug("ping -> pong ts=%s", data.get("ts"))
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
            log.debug("drive: raw=(%.3f, %.3f) shaped=(%.3f, %.3f) ts=%s", left, right, shaped_left, shaped_right, _ts)

            # Drive motors and update shared drive state for encoder direction
            _motors.set(left=shaped_left, right=shaped_right)
            _set_drive_state(shaped_left, shaped_right)
            last_cmd["t"] = time.monotonic()

            # Ack back to client (useful for latency/telemetry)
            ack = {
                "type": "ack",
                "ts": _ts,
                "echo": {"left": left, "right": right}
            }
            log.debug("send: %s", ack)
            await ws.send_json(ack)
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
        log.info("WS closing; cancelling watchdog and stopping motors")
        wd_task.cancel()
        with contextlib.suppress(Exception):
            await wd_task
        _motors.stop()
