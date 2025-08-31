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

app = FastAPI(title="GordonBot Backend", version="0.1.0")

# CORS (optional during dev when frontend runs on a different port)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # tighten later
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# REST routes under /api
api = FastAPI()
api.include_router(ping_router.router, prefix="")
api.include_router(battery_router.router, prefix="")
api.include_router(diagnostics_router.router, prefix="")
api.include_router(video_router.router, prefix="")

app.mount(settings.api_prefix, api)

# WebSocket (mounted on the root app, not under /api)
app.include_router(control_socket.router)

# (Optional for deployment) Serve built frontend from ./dist
# app.mount("/", StaticFiles(directory="dist", html=True), name="static")
