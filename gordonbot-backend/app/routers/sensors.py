from __future__ import annotations

from fastapi import APIRouter

from app.schemas import SensorsStatus
from app.services.sensors import get_sensor_status

router = APIRouter()


@router.get("/sensors/status", response_model=SensorsStatus, tags=["sensors"])
def sensors_status() -> SensorsStatus:
    return get_sensor_status()


@router.get("/sensors", response_model=SensorsStatus, tags=["sensors"])
def sensors() -> SensorsStatus:
    """Alias for /sensors/status for ROS2 bridge compatibility."""
    return get_sensor_status()

