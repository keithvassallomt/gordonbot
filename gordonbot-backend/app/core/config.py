from __future__ import annotations
from pydantic import BaseModel

class Settings(BaseModel):
    api_prefix: str = "/api"
    control_ws_path: str = "/ws/control"

# Simple settings instance (expand later for env vars)
settings = Settings()
