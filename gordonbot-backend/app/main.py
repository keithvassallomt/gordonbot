from __future__ import annotations
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

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


# Root-level WHEP proxy removed; use router-mounted endpoint under /api
