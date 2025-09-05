from __future__ import annotations
from fastapi import FastAPI, Request
from fastapi.responses import Response
import urllib.request
import urllib.error
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import re

from app.core.config import settings
from app.routers import ping as ping_router
from app.routers import battery as battery_router
from app.routers import diagnostics as diagnostics_router
from app.routers import video as video_router
from app.sockets import control as control_socket
from app.services.camera import camera

app = FastAPI(title="GordonBot Backend", version="0.1.0")

# CORS setup. Prefer regex if provided; otherwise use explicit list.
cors_kwargs = dict(
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
if settings.cors_allow_origin_regex:
    cors_kwargs["allow_origin_regex"] = settings.cors_allow_origin_regex
else:
    cors_kwargs["allow_origins"] = settings.cors_origins

app.add_middleware(CORSMiddleware, **cors_kwargs)

# REST routes under /api
api = FastAPI()
# Also attach CORS to the mounted API app so preflight on mounted paths succeeds
api.add_middleware(CORSMiddleware, **cors_kwargs)
api.include_router(ping_router.router, prefix="")
api.include_router(battery_router.router, prefix="")
api.include_router(diagnostics_router.router, prefix="")
api.include_router(video_router.router, prefix="")

app.mount(settings.api_prefix, api)

# WebSocket (mounted on the root app, not under /api)
app.include_router(control_socket.router)

# (Optional for deployment) Serve built frontend from ./dist
# app.mount("/", StaticFiles(directory="dist", html=True), name="static")


@app.on_event("startup")
async def _start_streaming_if_configured() -> None:
    # Optionally publish to an RTSP server (e.g., MediaMTX) if configured
    if settings.camera_rtsp_url:
        camera.start_publisher(settings.camera_rtsp_url, bitrate=settings.camera_bitrate)


@app.on_event("shutdown")
async def _stop_streaming() -> None:
    camera.stop_publisher()


# Direct WHEP proxy on root app as well (to avoid any sub-app routing issues)
@app.post(f"{settings.api_prefix}/video/whep/{{stream}}")
async def whep_offer_root(stream: str, request: Request) -> Response:
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

@app.options(f"{settings.api_prefix}/video/whep/{{stream}}")
async def whep_options_root(stream: str) -> Response:
    return Response(status_code=204)
