from __future__ import annotations
from fastapi import APIRouter
from app.schemas import BatteryData

router = APIRouter()

@router.get("/battery/status", response_model=BatteryData, tags=["battery"])
async def battery_status() -> BatteryData:
    # TODO: Replace with real sensor reads
    return BatteryData(
        percent=83.0,
        charging=False,
        vbusVoltage=5.02,
        vbusCurrent=0.12,
        vbusPower=0.60,
        batteryVoltage=15.8,
        batteryCurrent=-0.85,
        remainingCapacity=220.0,
        avgTimeToFullMin=None,
        cells=[3.95, 3.95, 3.95, 3.95],
    )