from __future__ import annotations
from pydantic import BaseModel, Field
from typing import List, Optional

# ---- Battery ----------------------------------------------------------------

class BatteryData(BaseModel):
    percent: float = Field(ge=0, le=100)
    charging: Optional[str] = Field(default=None, description="One of: 'idle', 'charging', 'fast_charging', 'discharging'")
    vbusVoltage: Optional[float] = None
    vbusCurrent: Optional[float] = None
    vbusPower: Optional[float] = None
    batteryVoltage: Optional[float] = None
    batteryCurrent: Optional[float] = None  # negative = discharging
    remainingCapacity: Optional[float] = None
    avgTimeToFullMin: Optional[int] = None
    cells: Optional[List[float]] = None  # [c1,c2,c3,c4]

# ---- Diagnostics -----------------------------------------------------------
class Diagnostics(BaseModel):
    cpu_util_percent: Optional[float] = Field(default=None, description="Instantaneous CPU utilization percent")
    load_average: Optional[dict] = Field(default=None, description="Load averages for 1m, 5m, 15m")
    load_per_cpu: Optional[dict] = Field(default=None, description="Load averages normalized by CPU count")
    cpu_temperature: Optional[float] = Field(default=None, description="CPU temperature in Celsius")
    rp1_temperature: Optional[float] = Field(default=None, description="RP1 I/O controller temperature in Celsius (if available)")
    memory: Optional[dict] = Field(default=None, description="Virtual memory stats")
    swap: Optional[dict] = Field(default=None, description="Swap memory stats")
    disk_root: Optional[dict] = Field(default=None, description="Disk usage for root filesystem")
    uptime_seconds: Optional[int] = Field(default=None, description="System uptime in seconds")
    boot_time: Optional[int] = Field(default=None, description="System boot time (epoch seconds)")
    throttling: Optional[dict] = Field(default=None, description="Raspberry Pi throttling flags (if available)")

# ---- Drive control -----------------------------------------------------------

class DrivePayload(BaseModel):
    left: float
    right: float
    ts: int

class DriveMessage(BaseModel):
    type: str
    payload: DrivePayload