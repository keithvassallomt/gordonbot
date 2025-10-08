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

class BNO055CalibrationStatus(BaseModel):
    sys: Optional[int] = None
    gyro: Optional[int] = None
    accel: Optional[int] = None
    mag: Optional[int] = None

class BNO055Data(BaseModel):
    euler: Optional[EulerDeg] = None  # degrees
    quat: Optional[Quaternion] = None  # unit quaternion
    ang_vel_rad_s: Optional[Vector3] = None
    accel_m_s2: Optional[Vector3] = None
    mag_uT: Optional[Vector3] = None
    lin_accel_m_s2: Optional[Vector3] = None
    gravity_m_s2: Optional[Vector3] = None
    temp_c: Optional[float] = None
    calibration: Optional[BNO055CalibrationStatus] = None

class ToFData(BaseModel):
    distance_mm: Optional[int] = None

class SensorsStatus(BaseModel):
    ts: int
    encoders: Optional[MotorEncoders] = None
    tof: Optional[ToFData] = None
    bno055: Optional[BNO055Data] = None


class OrientationFrame(BaseModel):
    type: str = "orientation"
    ts: int
    qw: Optional[float] = None
    qx: Optional[float] = None
    qy: Optional[float] = None
    qz: Optional[float] = None
    euler: Optional[EulerDeg] = None
    calib: Optional[BNO055CalibrationStatus] = None
    stale: bool = False


class OrientationAck(BaseModel):
    type: str
    ok: bool
    message: Optional[str] = None


class WakeWordStatus(BaseModel):
    enabled: bool
    detections: int = 0
    last_detected_at: Optional[datetime] = None
    recent: List[datetime] = Field(default_factory=list)


# ---- LIDAR ------------------------------------------------------------------

class LidarPointData(BaseModel):
    angle: float = Field(description="Angle in degrees (0-360)")
    distance_mm: float = Field(description="Distance in millimeters")
    quality: int = Field(ge=0, le=255, description="Signal quality (0-255)")


class LidarScanData(BaseModel):
    ts: int = Field(description="Timestamp in milliseconds")
    points: List[LidarPointData] = Field(description="List of scan points")
    scan_rate_hz: float = Field(description="Scan rate in Hz")


# ---- SLAM -------------------------------------------------------------------

class SlamMapOrigin(BaseModel):
    """Map origin position and orientation."""
    x: float = Field(description="X position in meters")
    y: float = Field(description="Y position in meters")
    theta: float = Field(description="Rotation in radians")


class SlamMapMessage(BaseModel):
    """SLAM map data (occupancy grid)."""
    type: str = Field(default="map", description="Message type")
    ts: int = Field(description="Timestamp in milliseconds")
    width: int = Field(description="Map width in cells")
    height: int = Field(description="Map height in cells")
    resolution: float = Field(description="Meters per cell")
    origin: SlamMapOrigin = Field(description="Map origin (lower-left corner)")
    data: List[int] = Field(description="Occupancy grid data: -1=unknown, 0=free, 100=occupied")


class SlamPoseMessage(BaseModel):
    """Robot pose in map frame."""
    type: str = Field(default="pose", description="Message type")
    ts: int = Field(description="Timestamp in milliseconds")
    x: float = Field(description="X position in meters")
    y: float = Field(description="Y position in meters")
    theta: float = Field(description="Orientation in radians")
    frame_id: str = Field(default="map", description="Reference frame")


class SlamGoToPointRequest(BaseModel):
    """Request payload for go-to-point navigation."""
    x: float = Field(description="Target X coordinate in map frame (meters)")
    y: float = Field(description="Target Y coordinate in map frame (meters)")
    tolerance: float = Field(
        default=0.1,
        ge=0.02,
        le=1.0,
        description="Acceptable distance to consider the target reached (meters)",
    )


class LidarStatus(BaseModel):
    connected: bool
    running: bool
    scan_rate_hz: float
    port: str
    health: str
