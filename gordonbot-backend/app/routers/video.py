from __future__ import annotations
from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import Response
import urllib.request
import urllib.error
from urllib.parse import urljoin

from app.services.camera import camera
from app.core.config import settings

router = APIRouter()


@router.get("/video/snapshot.jpg", tags=["video"])
def snapshot_jpg() -> Response:
    data = camera.capture_jpeg()
    return Response(content=data, media_type="image/jpeg")


@router.get("/video/status", tags=["video"])
def video_status(request: Request) -> dict[str, object | None]:
    """Return stream availability and decoder backend details."""
    detect_enabled = bool(getattr(settings, "detect_enabled", False))
    backend = str(getattr(settings, "detect_backend", "cpu")).lower()
    decoder = backend if detect_enabled else None
    return {
        "annotated_available": bool(getattr(settings, "camera_rtsp_annot_url", None)),
        "decoder": decoder,
        "detect_enabled": detect_enabled,
        "raw_active": bool(getattr(request.app.state, "raw_stream_active", False)),
    }


def _raw_state(request: Request) -> bool:
    return bool(getattr(request.app.state, "raw_stream_active", False))


@router.post("/video/raw/start", tags=["video"])
async def start_raw_stream(request: Request) -> dict[str, object]:
    if not settings.camera_rtsp_url:
        raise HTTPException(status_code=404, detail="Raw stream not configured")
    if _raw_state(request):
        return {"started": True, "already_running": True}
    ok = camera.start_publisher(settings.camera_rtsp_url, bitrate=settings.camera_bitrate)
    request.app.state.raw_stream_active = bool(ok)  # type: ignore[attr-defined]
    if not ok:
        raise HTTPException(status_code=500, detail="Failed to start raw stream")
    return {"started": True, "already_running": False}


@router.post("/video/raw/stop", tags=["video"])
async def stop_raw_stream(request: Request) -> dict[str, object]:
    if not settings.camera_rtsp_url:
        raise HTTPException(status_code=404, detail="Raw stream not configured")
    if not _raw_state(request):
        return {"stopped": True, "already_stopped": True}
    camera.stop_publisher()
    request.app.state.raw_stream_active = False  # type: ignore[attr-defined]
    return {"stopped": True, "already_stopped": False}


@router.post("/video/whep/{stream}", tags=["video"])
async def whep_offer(stream: str, request: Request) -> Response:
    """Proxy a WHEP offer to MediaMTX to avoid CORS.

    Expects `application/sdp` body and returns SDP answer.
    Uses MEDIAMTX_WHEP_BASE from settings or defaults to http://127.0.0.1:8889/whep
    """
    offer = await request.body()
    base = getattr(settings, "mediamtx_whep_base", None) or "http://127.0.0.1:8889/whep"
    style = getattr(settings, "mediamtx_whep_style", "prefix").lower()
    base_clean = base.rstrip("/")
    if style == "suffix":
        if base_clean.endswith("/whep"):
            base_root = base_clean[: -len("/whep")] or base_clean
            target = f"{base_root}/{stream}/whep"
        else:
            target = f"{base_clean}/{stream}/whep"
    else:  # prefix (default)
        if base_clean.endswith("/whep"):
            target = f"{base_clean}/{stream}"
        else:
            target = f"{base_clean}/whep/{stream}"
    req = urllib.request.Request(
        target,
        data=offer,
        headers={
            "Content-Type": "application/sdp",
            "Accept": "application/sdp",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:  # nosec B310
            answer = resp.read()
            location = resp.headers.get("Location")
            headers = {"Content-Type": "application/sdp"}
            if location:
                headers["Location"] = location
            return Response(content=answer, media_type="application/sdp", headers=headers, status_code=resp.status)
    except urllib.error.HTTPError as e:
        body = e.read() if hasattr(e, "read") else b""
        return Response(content=body, media_type="text/plain", status_code=e.code)
    except Exception as e:
        return Response(content=str(e), media_type="text/plain", status_code=502)


@router.options("/video/whep/{stream}")
async def whep_options(stream: str) -> Response:
    # Allow CORS preflight requests to succeed on mounted API
    return Response(status_code=204)
