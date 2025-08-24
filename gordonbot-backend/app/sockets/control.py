from __future__ import annotations
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.schemas import DriveMessage

router = APIRouter()

@router.websocket("/ws/control")
async def ws_control(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            data = await ws.receive_json()
            msg = DriveMessage.model_validate(data)

            # Clamp for safety (server-side)
            left = max(-1.0, min(1.0, msg.payload.left))
            right = max(-1.0, min(1.0, msg.payload.right))
            _ts = msg.payload.ts

            # TODO: send to motor controller here
            # e.g., motors.set(left=left, right=right)
            # Optionally ack:
            # await ws.send_json({ "type": "ack", "ts": _ts })
    except WebSocketDisconnect:
        # TODO: ensure motors stop on disconnect
        pass