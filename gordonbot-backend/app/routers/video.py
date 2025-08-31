from __future__ import annotations
from fastapi import APIRouter
from fastapi.responses import Response, StreamingResponse

from app.services.camera import camera

router = APIRouter()


@router.get("/video/snapshot.jpg", tags=["video"])
def snapshot_jpg() -> Response:
    data = camera.capture_jpeg()
    return Response(content=data, media_type="image/jpeg")


@router.get("/video/stream.mjpg", tags=["video"])
def stream_mjpeg() -> StreamingResponse:
    boundary = "frame"
    return StreamingResponse(
        camera.mjpeg_generator(),
        media_type=f"multipart/x-mixed-replace; boundary={boundary}",
    )

