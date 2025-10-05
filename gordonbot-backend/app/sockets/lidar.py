from __future__ import annotations

import asyncio
import logging
import json
from typing import Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from app.core.config import settings
from app.services.lidar import get_lidar_service

log = logging.getLogger(__name__)

router = APIRouter()


@router.websocket("/ws/lidar")
async def lidar_websocket(websocket: WebSocket) -> None:
    """
    WebSocket endpoint for streaming LIDAR scan data.

    Sends complete scans as they are captured by the LIDAR service.
    Message format:
    {
        "type": "scan",
        "ts": <timestamp_ms>,
        "points": [
            {"angle": <degrees>, "distance_mm": <mm>, "quality": <0-255>},
            ...
        ],
        "scan_rate_hz": <hz>
    }

    Also supports ping/pong for connection health:
    Client -> {"type": "ping", "ts": <ms>}
    Server -> {"type": "pong", "ts": <same>}
    """
    await websocket.accept()
    log.info("LIDAR WebSocket client connected")

    service = get_lidar_service()

    if service is None:
        await websocket.send_json({
            "type": "error",
            "message": "LIDAR service not initialized",
        })
        await websocket.close()
        return

    # Track last sent scan to avoid duplicates
    last_scan_ts: Optional[int] = None

    try:
        # Start background task to send scan updates
        async def send_scans():
            nonlocal last_scan_ts

            while True:
                try:
                    # Poll for new scans every 50ms
                    await asyncio.sleep(0.05)

                    scan = await service.get_latest_scan()

                    if scan is None:
                        continue

                    # Only send if this is a new scan
                    if last_scan_ts is not None and scan.ts <= last_scan_ts:
                        continue

                    last_scan_ts = scan.ts

                    # Convert to JSON-serializable format
                    message = {
                        "type": "scan",
                        "ts": scan.ts,
                        "points": [
                            {
                                "angle": point.angle,
                                "distance_mm": point.distance_mm,
                                "quality": point.quality,
                            }
                            for point in scan.points
                        ],
                        "scan_rate_hz": scan.scan_rate_hz,
                    }

                    await websocket.send_json(message)

                except Exception as e:
                    log.error(f"Error sending LIDAR scan: {e}")
                    break

        # Start scan sender task
        sender_task = asyncio.create_task(send_scans())

        # Listen for client messages (ping/pong)
        try:
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
            log.info("LIDAR WebSocket client disconnected")
        finally:
            # Cancel sender task
            sender_task.cancel()
            try:
                await sender_task
            except asyncio.CancelledError:
                pass

    except Exception as e:
        log.error(f"LIDAR WebSocket error: {e}", exc_info=True)
        try:
            await websocket.close()
        except Exception:
            pass
