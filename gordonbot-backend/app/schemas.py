from __future__ import annotations
from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime

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

# ---- Sensors ----------------------------------------------------------------

class EncoderData(BaseModel):
    connected: Optional[bool] = None
    ticks: Optional[int] = None
    distance_m: Optional[float] = None
    distance_mm: Optional[float] = None
    rpm: Optional[float] = None
    speed_mm_s: Optional[float] = None

class MotorEncoders(BaseModel):
    left: Optional[EncoderData] = None
    right: Optional[EncoderData] = None

class Vector3(BaseModel):
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None

class Quaternion(BaseModel):
    w: Optional[float] = None
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None

class EulerDeg(BaseModel):
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None

class BNO055Data(BaseModel):
    euler: Optional[EulerDeg] = None  # degrees
    quat: Optional[Quaternion] = None  # unit quaternion
    ang_vel_rad_s: Optional[Vector3] = None
    accel_m_s2: Optional[Vector3] = None
    mag_uT: Optional[Vector3] = None
    lin_accel_m_s2: Optional[Vector3] = None
    gravity_m_s2: Optional[Vector3] = None
    temp_c: Optional[float] = None

class ToFData(BaseModel):
    distance_mm: Optional[int] = None

class SensorsStatus(BaseModel):
    ts: int
    encoders: Optional[MotorEncoders] = None
    tof: Optional[ToFData] = None
    bno055: Optional[BNO055Data] = None


class WakeWordStatus(BaseModel):
    enabled: bool
    detections: int = 0
    last_detected_at: Optional[datetime] = None
    recent: List[datetime] = Field(default_factory=list)
