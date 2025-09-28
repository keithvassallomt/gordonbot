"""BNO055 IMU sensor helper."""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional, Sequence

from app.schemas import BNO055Data, EulerDeg, Quaternion, Vector3

log = logging.getLogger(__name__)


try:  # pragma: no cover - hardware-specific import
    import board  # type: ignore
    import adafruit_bno055  # type: ignore
    _IMPORT_OK = True
except Exception as exc:  # pragma: no cover - dev machines without hardware
    board = None  # type: ignore
    adafruit_bno055 = None  # type: ignore
    _IMPORT_OK = False
    _IMPORT_ERR = exc
else:
    _IMPORT_ERR = None


_SENSOR_LOCK = threading.Lock()
_SENSOR: Optional["adafruit_bno055.BNO055_I2C"] = None
_LAST_INIT_ATTEMPT = 0.0
_INIT_BACKOFF_SEC = 5.0


def _ensure_sensor() -> Optional["adafruit_bno055.BNO055_I2C"]:
    """Initialise the BNO055 instance lazily and return it when ready."""

    global _SENSOR, _LAST_INIT_ATTEMPT

    if _SENSOR is not None:
        return _SENSOR

    if not _IMPORT_OK:
        if _IMPORT_ERR and log.isEnabledFor(logging.DEBUG):
            log.debug("BNO055 import unavailable: %s", _IMPORT_ERR)
        return None

    now = time.monotonic()
    if now - _LAST_INIT_ATTEMPT < _INIT_BACKOFF_SEC:
        return None

    with _SENSOR_LOCK:
        if _SENSOR is not None:
            return _SENSOR
        _LAST_INIT_ATTEMPT = now
        try:
            i2c = board.I2C()  # type: ignore[attr-defined]
            sensor = adafruit_bno055.BNO055_I2C(i2c)
            _SENSOR = sensor
            log.info("BNO055 sensor initialised over I2C")
        except Exception as exc:  # pragma: no cover - hardware path
            log.warning("BNO055 init failed: %s", exc)
            _SENSOR = None
    return _SENSOR


def _reset_sensor() -> None:
    global _SENSOR, _LAST_INIT_ATTEMPT
    with _SENSOR_LOCK:
        _SENSOR = None
        _LAST_INIT_ATTEMPT = 0.0


def _vec3(values: Optional[Sequence[Optional[float]]]) -> Vector3:
    if not values:
        return Vector3()
    x, y, z = (float(v) if v is not None else None for v in values[:3])
    return Vector3(x=x, y=y, z=z)


def _quat(values: Optional[Sequence[Optional[float]]]) -> Quaternion:
    if not values:
        return Quaternion()
    w, x, y, z = (float(v) if v is not None else None for v in values[:4])
    return Quaternion(w=w, x=x, y=y, z=z)


def _euler(values: Optional[Sequence[Optional[float]]]) -> EulerDeg:
    # Library returns (heading, roll, pitch) in degrees
    if not values:
        return EulerDeg()
    heading, roll, pitch = values[:3]
    return EulerDeg(
        roll=float(roll) if roll is not None else None,
        pitch=float(pitch) if pitch is not None else None,
        yaw=float(heading) if heading is not None else None,
    )


def read_bno055() -> Optional[BNO055Data]:
    """Read all available BNO055 channels and return them as a schema object."""

    sensor = _ensure_sensor()
    if sensor is None:
        return None

    try:
        euler = sensor.euler
        quat = sensor.quaternion
        ang_vel = sensor.gyro  # radians/sec
        accel = sensor.acceleration  # m/s^2
        mag = sensor.magnetic  # microTesla
        lin_accel = sensor.linear_acceleration  # m/s^2 without gravity
        gravity = sensor.gravity  # m/s^2 gravity vector
        temp_c = sensor.temperature
    except Exception as exc:  # pragma: no cover - hardware path
        log.debug("BNO055 read error: %s", exc)
        _reset_sensor()
        return None

    return BNO055Data(
        euler=_euler(euler),
        quat=_quat(quat),
        ang_vel_rad_s=_vec3(ang_vel),
        accel_m_s2=_vec3(accel),
        mag_uT=_vec3(mag),
        lin_accel_m_s2=_vec3(lin_accel),
        gravity_m_s2=_vec3(gravity),
        temp_c=float(temp_c) if temp_c is not None else None,
    )

