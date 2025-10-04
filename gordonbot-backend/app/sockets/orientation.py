from __future__ import annotations

import asyncio
import contextlib
import logging
import math
import time

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from app.core.config import settings
from app.schemas import OrientationAck, OrientationFrame
from app.services.autonomous_drive import drive_figure_eight, stop_motors
from app.services.bno055 import read_orientation_sample, start_bno055_calibration

router = APIRouter()

log = logging.getLogger(__name__)


async def _send_orientation(ws: WebSocket, stop_event: asyncio.Event) -> None:
    last_error: str | None = None
    while not stop_event.is_set():
        sample = read_orientation_sample()
        ts = int(time.time() * 1000)

        if sample is None:
            if last_error != "unavailable":
                last_error = "unavailable"
                with contextlib.suppress(Exception):
                    await ws.send_json({"type": "error", "message": "BNO055 orientation unavailable"})
            await asyncio.sleep(1.0)
            continue

        last_error = None
        quat = sample.quaternion
        calib = sample.calibration
        def _component(value: float | None, fallback: float) -> float:
            if value is None or not math.isfinite(value):
                return fallback
            return float(value)

        qw = _component(quat.w, 1.0)
        qx = _component(quat.x, 0.0)
        qy = _component(quat.y, 0.0)
        qz = _component(quat.z, 0.0)
        stale = bool(sample.stale)
        frame = OrientationFrame(
            ts=ts,
            qw=qw,
            qx=qx,
            qy=qy,
            qz=qz,
            euler=sample.euler,
            calib=calib,
            stale=stale,
        )
        try:
            await ws.send_json(frame.model_dump(mode="json"))
        except WebSocketDisconnect:
            stop_event.set()
            return
        except Exception as exc:
            log.debug("Failed to send orientation frame: %s", exc)
            await asyncio.sleep(0.2)
            continue
        await asyncio.sleep(0.05)


async def _auto_calibration_sequence(ws: WebSocket, stop_event: asyncio.Event) -> bool:
    """
    Execute figure-8 calibration pattern for BNO055 IMU.
    Delegates to autonomous_drive service.
    """
    try:
        with contextlib.suppress(Exception):
            await ws.send_json({"type": "calibration_run", "status": "started"})

        # Delegate to autonomous drive service
        return await drive_figure_eight(ws, stop_event)
    except Exception as exc:
        log.exception("Auto-calibration sequence failed: %s", exc)
        return False
    finally:
        stop_motors()


async def _receive_commands(
    ws: WebSocket,
    stop_event: asyncio.Event,
    calibration_state: dict[str, asyncio.Task | None],
) -> None:
    while not stop_event.is_set():
        try:
            message = await ws.receive_json()
        except WebSocketDisconnect:
            stop_event.set()
            break
        except Exception as exc:
            log.debug("Orientation WS receive error: %s", exc)
            continue

        if not isinstance(message, dict):
            continue

        action = str(message.get("action", "")).strip().lower()
        if action == "ping":
            with contextlib.suppress(Exception):
                await ws.send_json({"type": "pong", "ts": message.get("ts")})
            continue

        if action == "start_calibration":
            success = start_bno055_calibration()
            ack = OrientationAck(type="calibration", ok=success)
            if not success:
                ack.message = "BNO055 not available or calibration failed"
            with contextlib.suppress(Exception):
                await ws.send_json(ack.model_dump(mode="json"))

            if success:
                task = calibration_state.get("task")
                if task is None or task.done():
                    async def _run_and_notify() -> None:
                        result = await _auto_calibration_sequence(ws, stop_event)
                        if not stop_event.is_set():
                            with contextlib.suppress(Exception):
                                await ws.send_json({"type": "calibration_complete", "ok": bool(result)})

                    new_task = asyncio.create_task(_run_and_notify())

                    def _clear(_task: asyncio.Task) -> None:
                        calibration_state["task"] = None

                    new_task.add_done_callback(_clear)
                    calibration_state["task"] = new_task
            continue

        if action == "abort_calibration":
            task = calibration_state.get("task")
            if task is not None and not task.done():
                log.info("Aborting calibration by user request")
                stop_event.set()
                task.cancel()
                with contextlib.suppress(Exception):
                    await task
                stop_motors()
                calibration_state["task"] = None
                with contextlib.suppress(Exception):
                    await ws.send_json({"type": "calibration_complete", "ok": False})
            continue

        log.debug("Orientation WS received unknown action: %s", action)


@router.websocket(settings.orientation_ws_path)
async def ws_orientation(ws: WebSocket) -> None:
    await ws.accept()
    log.info("WS connected to %s", settings.orientation_ws_path)

    stop_event = asyncio.Event()
    calibration_state: dict[str, asyncio.Task | None] = {"task": None}
    sender = asyncio.create_task(_send_orientation(ws, stop_event))
    receiver = asyncio.create_task(_receive_commands(ws, stop_event, calibration_state))

    done, pending = await asyncio.wait(
        {sender, receiver},
        return_when=asyncio.FIRST_COMPLETED,
    )

    stop_event.set()
    cal_task = calibration_state.get("task")
    if cal_task is not None and not cal_task.done():
        cal_task.cancel()
        with contextlib.suppress(Exception):
            await cal_task
        stop_motors()
    for task in pending:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task

    for task in done:
        with contextlib.suppress(asyncio.CancelledError):
            await task

    log.info("WS disconnected from %s", settings.orientation_ws_path)
