from __future__ import annotations

import json
import time
from pathlib import Path

from fastapi import APIRouter, HTTPException, Body

from app.services.bno055 import save_bno055_offsets
from app.core.config import settings

router = APIRouter()


@router.post("/orientation/calibration/save", tags=["orientation"])
async def save_orientation_calibration() -> dict[str, bool]:
    if not save_bno055_offsets():
        raise HTTPException(status_code=500, detail="Failed to save BNO055 calibration offsets")
    return {"saved": True}


@router.post("/orientation/debug-log", tags=["orientation"])
async def save_debug_log(data: dict = Body(...)) -> dict[str, str]:
    """Save orientation debug log with motion data."""
    try:
        # Create debug logs directory
        debug_dir = Path(settings.bno055_calibration_path).parent / "debug"
        debug_dir.mkdir(parents=True, exist_ok=True)

        # Generate filename with timestamp
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        log_path = debug_dir / f"orientation_debug_{timestamp}.json"

        # Save data
        with log_path.open("w", encoding="utf-8") as fh:
            json.dump(data, fh, indent=2, sort_keys=True)

        return {"path": str(log_path)}
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"Failed to save debug log: {exc}")
