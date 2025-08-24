from __future__ import annotations
from pydantic import BaseModel, Field
from typing import List, Optional

# ---- Battery ----------------------------------------------------------------

class BatteryData(BaseModel):
    percent: float = Field(ge=0, le=100)
    charging: Optional[bool] = None
    vbusVoltage: Optional[float] = None
    vbusCurrent: Optional[float] = None
    vbusPower: Optional[float] = None
    batteryVoltage: Optional[float] = None
    batteryCurrent: Optional[float] = None  # negative = discharging
    remainingCapacity: Optional[float] = None
    avgTimeToFullMin: Optional[int] = None
    cells: Optional[List[float]] = None  # [c1,c2,c3,c4]

# ---- Drive control -----------------------------------------------------------

class DrivePayload(BaseModel):
    left: float
    right: float
    ts: int

class DriveMessage(BaseModel):
    type: str
    payload: DrivePayload