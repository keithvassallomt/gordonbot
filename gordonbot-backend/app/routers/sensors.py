from __future__ import annotations

from fastapi import APIRouter

from app.schemas import SensorsStatus
from app.services.sensors import get_sensor_status

router = APIRouter()


@router.get("/sensors/status", response_model=SensorsStatus, tags=["sensors"])
def sensors_status() -> SensorsStatus:
    return get_sensor_status()

