from __future__ import annotations

from fastapi import APIRouter, Request

from app.schemas import WakeWordStatus

router = APIRouter()


@router.get("/wakeword/status", response_model=WakeWordStatus, tags=["wakeword"])
async def wakeword_status(request: Request) -> WakeWordStatus:
    svc = getattr(request.app.state, "wake_word_service", None)
    if svc is None:
        return WakeWordStatus(enabled=False)

    stats = svc.get_stats()
    recent = svc.recent_detections()
    return WakeWordStatus(
        enabled=True,
        detections=stats.detections,
        last_detected_at=stats.last_detected_at,
        recent=recent,
    )
