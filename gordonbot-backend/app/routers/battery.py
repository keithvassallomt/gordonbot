from __future__ import annotations
from fastapi import APIRouter
from app.schemas import BatteryData
from app.services.battery import get_battery_status

router = APIRouter()

@router.get("/battery/status", response_model=BatteryData, tags=["battery"])
async def battery_status() -> BatteryData:
    
    data = get_battery_status()
    
    return BatteryData(
        percent=data["battery_percent"],
        charging=data["battery_status"] in ["charging", "fast_charging", "discharging"],
        vbusVoltage=data["vbus_voltage"],
        vbusCurrent=data["vbus_current"],
        vbusPower=data["vbus_power"],
        batteryVoltage=data["battery_voltage"],
        batteryCurrent=data["current"],
        remainingCapacity=data["remaining_capacity"],
        avgTimeToFullMin=data["avg_time_to_full"],
        cells=data["cells"]
    )