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

class Settings(BaseModel):
    api_prefix: str = "/api"
    control_ws_path: str = "/ws/control"
    verbose: bool = _getenv_bool("VERBOSE", False)

# Simple settings instance (expand later for env vars)
settings = Settings()
