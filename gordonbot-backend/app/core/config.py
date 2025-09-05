from __future__ import annotations
from pydantic import BaseModel
import os
try:
    from dotenv import load_dotenv  # type: ignore
    load_dotenv()
except Exception:
    # dotenv is optional; ignore if not installed
    pass

def _getenv_bool(name: str, default: bool = False) -> bool:
    val = os.getenv(name)
    if val is None:
        return default
    return val.strip().lower() in {"1", "true", "yes", "on"}

def _getenv_list(name: str, default: list[str] | None = None) -> list[str]:
    raw = os.getenv(name)
    if raw is None:
        return list(default or [])
    # Split on commas and whitespace, keep non-empty
    parts = [p.strip() for p in raw.replace("\n", ",").split(",")]
    return [p for p in parts if p]

class Settings(BaseModel):
    api_prefix: str = "/api"
    control_ws_path: str = "/ws/control"
    verbose: bool = _getenv_bool("VERBOSE", False)
    camera_rtsp_url: str | None = os.getenv("CAMERA_RTSP_URL")
    camera_bitrate: int = int(os.getenv("CAMERA_BITRATE", "2000000"))
    # Base URL for MediaMTX HTTP signaling. Can include or omit '/whep'.
    # Combined with MEDIAMTX_WHEP_STYLE to form the final target.
    mediamtx_whep_base: str = os.getenv("MEDIAMTX_WHEP_BASE", "http://127.0.0.1:8889/whep")
    # WHEP URL style: 'prefix' -> '/whep/{stream}', 'suffix' -> '/{stream}/whep'
    # Defaults to 'prefix' to preserve existing behavior.
    mediamtx_whep_style: str = os.getenv("MEDIAMTX_WHEP_STYLE", "suffix")
    # Comma-separated list of allowed CORS origins, e.g. "http://localhost:5173,http://192.168.0.5:5173"
    cors_origins: list[str] = _getenv_list(
        "CORS_ORIGINS",
        [
            "http://127.0.0.1:5173",
            "http://localhost:5173",
        ],
    )
    # Optional regex for allowed CORS origins; if set, it is used instead of the list
    cors_allow_origin_regex: str | None = os.getenv(
        "CORS_ALLOW_ORIGIN_REGEX",
        # Default: allow common private/LAN dev hosts
        r"^https?://(localhost|127\.0\.0\.1|10\.\d+\.\d+\.\d+|192\.168\.\d+\.\d+|172\.(1[6-9]|2[0-9]|3[0-1])\.\d+\.\d+)(:\d+)?$",
    )

# Simple settings instance (expand later for env vars)
settings = Settings()
