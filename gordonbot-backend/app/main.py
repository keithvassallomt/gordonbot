from __future__ import annotations
import logging
import logging.config
from datetime import datetime

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from app.core.config import settings
from app.routers import ping as ping_router
from app.routers import battery as battery_router
from app.routers import diagnostics as diagnostics_router
from app.routers import video as video_router
from app.routers import sensors as sensors_router
from app.routers import wakeword as wakeword_router
from app.sockets import control as control_socket
from app.services.camera import camera
from app.services.audio_playback import AudioCuePlayer
from app.services.wake_word import WakeWordService
from app.services.voice_interaction import VoiceInteractionController

log = logging.getLogger(__name__)

def _setup_logging() -> None:
    # Configure a consistent console logger for the backend and uvicorn
    # Format: time level [backend] [logger] message
    fmt = "%(asctime)s %(levelname)s [backend] [%(name)s] %(message)s"
    config = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "standard": {"format": fmt},
        },
        "handlers": {
            "console": {
                "class": "logging.StreamHandler",
                "level": "DEBUG" if settings.verbose else "INFO",
                "formatter": "standard",
                "stream": "ext://sys.stdout",
            },
        },
        "loggers": {
            # Uvicorn server and access logs
            "uvicorn": {"handlers": ["console"], "level": "INFO", "propagate": False},
            "uvicorn.error": {"handlers": ["console"], "level": "INFO", "propagate": False},
            "uvicorn.access": {"handlers": ["console"], "level": "INFO", "propagate": False},
        },
        "root": {"handlers": ["console"], "level": "DEBUG" if settings.verbose else "INFO"},
    }
    try:
        logging.config.dictConfig(config)
    except Exception:  # pragma: no cover
        # Last resort fallback
        logging.basicConfig(level=logging.DEBUG if settings.verbose else logging.INFO, format=fmt)


_setup_logging()

app = FastAPI(title="GordonBot Backend", version="0.1.0")

wake_word_service: WakeWordService | None = None
wake_audio_player: AudioCuePlayer | None = None
voice_controller: VoiceInteractionController | None = None
app.state.wake_word_service = None  # type: ignore[attr-defined]
app.state.wake_audio_player = None  # type: ignore[attr-defined]
app.state.voice_controller = None  # type: ignore[attr-defined]
app.state.raw_stream_active = False  # type: ignore[attr-defined]

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
api.include_router(sensors_router.router, prefix="")
api.include_router(wakeword_router.router, prefix="")

app.mount(settings.api_prefix, api)

# WebSocket (mounted on the root app, not under /api)
app.include_router(control_socket.router)

# (Optional for deployment) Serve built frontend from ./dist
# app.mount("/", StaticFiles(directory="dist", html=True), name="static")


def _on_wake_word_detected(ts: datetime) -> None:
    log.info("Wake word detected at %s", ts.isoformat())
    controller = app.state.voice_controller  # type: ignore[attr-defined]
    if controller is not None:
        controller.notify_wake(ts)


@app.on_event("startup")
async def _start_streaming_if_configured() -> None:
    global wake_word_service, wake_audio_player, voice_controller

    # Optional annotated stream publisher (OpenCV overlays)
    if settings.camera_rtsp_annot_url:
        try:
            camera.start_publisher_annotated(
                settings.camera_rtsp_annot_url,
                fps=settings.annot_fps,
                bitrate=min(settings.camera_bitrate, 2_000_000),
                min_area=settings.annot_min_area,
            )
        except Exception:
            # Avoid bringing down the app on annotation errors
            pass

    if settings.wakeword_audio_enabled:
        try:
            wake_audio_player = AudioCuePlayer(
                path=settings.wakeword_audio_path,
                command_template=settings.wakeword_audio_command,
                enabled=True,
            )
            app.state.wake_audio_player = wake_audio_player  # type: ignore[attr-defined]
        except Exception as exc:
            log.error("Failed to initialise wake-word audio player: %s", exc)
            wake_audio_player = None
            app.state.wake_audio_player = None  # type: ignore[attr-defined]
    else:
        wake_audio_player = None
        app.state.wake_audio_player = None  # type: ignore[attr-defined]

    app.state.raw_stream_active = False  # type: ignore[attr-defined]

    try:
        voice_controller = VoiceInteractionController(
            settings=settings,
            wakeword_service_getter=lambda: wake_word_service,
            wake_player=wake_audio_player,
        )
        app.state.voice_controller = voice_controller  # type: ignore[attr-defined]
    except Exception as exc:
        log.error("Failed to initialise voice interaction controller: %s", exc)
        voice_controller = None
        app.state.voice_controller = None  # type: ignore[attr-defined]

    if settings.wakeword_enabled:
        if not settings.wakeword_access_key:
            log.warning("Wake word enabled but PICOVOICE access key is missing; skipping")
        else:
            try:
                wake_word_service = WakeWordService(
                    access_key=settings.wakeword_access_key,
                    keyword_path=settings.wakeword_keyword_path,
                    sensitivity=settings.wakeword_sensitivity,
                    device_index=settings.wakeword_audio_device_index,
                    callback=_on_wake_word_detected,
                    allow_missing_deps=settings.wakeword_allow_missing_deps,
                )
                app.state.wake_word_service = wake_word_service  # type: ignore[attr-defined]
                wake_word_service.start()
            except Exception as exc:
                log.error("Failed to start wake word listener: %s", exc)
    else:
        app.state.wake_word_service = None  # type: ignore[attr-defined]


@app.on_event("shutdown")
async def _stop_streaming() -> None:
    global wake_word_service, wake_audio_player, voice_controller
    camera.stop_publisher()
    camera.stop_publisher_annotated()
    if wake_word_service is not None:
        wake_word_service.stop()
        wake_word_service = None
    app.state.wake_word_service = None  # type: ignore[attr-defined]
    wake_audio_player = None
    app.state.wake_audio_player = None  # type: ignore[attr-defined]
    voice_controller = None
    app.state.voice_controller = None  # type: ignore[attr-defined]
    app.state.raw_stream_active = False  # type: ignore[attr-defined]


# Root-level WHEP proxy removed; use router-mounted endpoint under /api
