from __future__ import annotations
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.core.config import settings
from app.schemas import DriveMessage
from app.services.drive_state import set_drive as _set_drive_state
import asyncio
import time
import logging
import contextlib   

# lgpio 0.2.2.0 (latest release) lacks the SET_BIAS_* constants that
# gpiozero >=2.0 expects; alias them to the older SET_PULL_* names so the
# lgpio pin factory stays usable until an upstream release adds them.
try:  # pragma: no cover - hardware-specific adapter
    import lgpio  # type: ignore
except Exception:
    lgpio = None  # type: ignore
else:
    if hasattr(lgpio, "SET_PULL_NONE") and not hasattr(lgpio, "SET_BIAS_DISABLE"):
        lgpio.SET_BIAS_DISABLE = lgpio.SET_PULL_NONE  # type: ignore[attr-defined]
        lgpio.SET_BIAS_PULL_UP = lgpio.SET_PULL_UP  # type: ignore[attr-defined]
        lgpio.SET_BIAS_PULL_DOWN = lgpio.SET_PULL_DOWN  # type: ignore[attr-defined]

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

# Shared command timestamp for watchdog + external controllers
_LAST_COMMAND_TS = time.monotonic()


def _shape(value: float) -> float:
    return value * value * value


def _update_last_command(ts: float | None = None) -> None:
    global _LAST_COMMAND_TS
    _LAST_COMMAND_TS = ts if ts is not None else time.monotonic()


def seconds_since_last_command() -> float:
    return time.monotonic() - _LAST_COMMAND_TS


def process_drive_command(left: float, right: float, *, source: str = "unknown", shape: bool = True) -> tuple[float, float]:
    """Apply clamping, optional shaping, and motor output. Keeps watchdog happy."""
    left_clamped = max(-1.0, min(1.0, float(left)))
    right_clamped = max(-1.0, min(1.0, float(right)))

    if shape:
        shaped_left = _shape(left_clamped)
        shaped_right = _shape(right_clamped)
    else:
        shaped_left = left_clamped
        shaped_right = right_clamped

    _motors.set(left=shaped_left, right=shaped_right)
    _set_drive_state(shaped_left, shaped_right)
    _update_last_command()
    log.debug(
        "drive_cmd source=%s raw=(%.3f, %.3f) shaped=(%.3f, %.3f) shape=%s",
        source,
        left_clamped,
        right_clamped,
        shaped_left,
        shaped_right,
        shape,
    )
    return shaped_left, shaped_right


def stop_all_motors() -> None:
    """Immediate hardware stop that also resets watchdog timestamp."""
    _motors.stop()
    _set_drive_state(0.0, 0.0)
    _update_last_command()

@router.websocket(settings.control_ws_path)
async def ws_control(ws: WebSocket):
    await ws.accept()
    log.info("WS connected to %s", settings.control_ws_path)

    # Dead-man timer: stop if no command within this window
    DEADMAN_SECONDS = 0.4
    wd_state = {"tripped": False}

    async def watchdog():
        try:
            while True:
                await asyncio.sleep(0.2)
                if seconds_since_last_command() > DEADMAN_SECONDS:
                    if not wd_state["tripped"]:
                        log.debug("watchdog: dead-man timeout %.1fs reached; stopping", DEADMAN_SECONDS)
                        wd_state["tripped"] = True
                    stop_all_motors()
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

            shaped_left, shaped_right = process_drive_command(left, right, source="ws")
            log.debug("drive: raw=(%.3f, %.3f) shaped=(%.3f, %.3f) ts=%s", left, right, shaped_left, shaped_right, _ts)

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
        stop_all_motors()
