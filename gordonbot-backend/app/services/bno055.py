"""BNO055 IMU sensor helper."""
from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

from app.schemas import (
    BNO055CalibrationStatus,
    BNO055Data,
    EulerDeg,
    Quaternion,
    Vector3,
)
from app.core.config import settings

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
_CALIBRATION_PATH = Path(settings.bno055_calibration_path)

_DEFAULT_POLL_RATE_HZ = 50.0
_READ_LOCK = threading.Lock()
_CACHE_LOCK = threading.Lock()
_latest_data: Optional[BNO055Data] = None
_latest_orientation: Optional["OrientationSample"] = None
_latest_timestamp: float = 0.0
_poller: Optional["_BNO055Poller"] = None


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
            _apply_saved_offsets(sensor)
        except Exception as exc:  # pragma: no cover - hardware path
            log.warning("BNO055 init failed: %s", exc)
            _SENSOR = None
    return _SENSOR


def _reset_sensor() -> None:
    global _SENSOR, _LAST_INIT_ATTEMPT
    with _SENSOR_LOCK:
        _SENSOR = None
        _LAST_INIT_ATTEMPT = 0.0
    _clear_cache()


def _apply_saved_offsets(sensor: "adafruit_bno055.BNO055_I2C") -> None:  # type: ignore[name-defined]
    path = _CALIBRATION_PATH
    if not path.exists():
        return
    try:
        with path.open("r", encoding="utf-8") as fh:
            data = json.load(fh)

        # Check for new format (individual properties)
        if "offsets_accelerometer" in data:
            accel_offsets = data.get("offsets_accelerometer")
            mag_offsets = data.get("offsets_magnetometer")
            gyro_offsets = data.get("offsets_gyroscope")
            accel_radius = data.get("radius_accelerometer")
            mag_radius = data.get("radius_magnetometer")

            with _READ_LOCK:
                if accel_offsets is not None:
                    sensor.offsets_accelerometer = tuple(accel_offsets)  # type: ignore[attr-defined]
                if mag_offsets is not None:
                    sensor.offsets_magnetometer = tuple(mag_offsets)  # type: ignore[attr-defined]
                if gyro_offsets is not None:
                    sensor.offsets_gyroscope = tuple(gyro_offsets)  # type: ignore[attr-defined]
                if accel_radius is not None:
                    sensor.radius_accelerometer = int(accel_radius)  # type: ignore[attr-defined]
                if mag_radius is not None:
                    sensor.radius_magnetometer = int(mag_radius)  # type: ignore[attr-defined]

            log.info("Applied saved BNO055 calibration offsets from %s", path)
        else:
            # Legacy format - single offsets list is not supported anymore
            log.warning("Calibration file %s uses legacy format; please recalibrate", path)
    except Exception as exc:
        log.warning("Failed to apply saved BNO055 offsets from %s: %s", path, exc)


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


def _calibration(values: Optional[Sequence[Optional[int]]]) -> BNO055CalibrationStatus:
    if not values:
        return BNO055CalibrationStatus()
    sys, gyro, accel, mag = (int(v) if v is not None else None for v in values[:4])
    return BNO055CalibrationStatus(sys=sys, gyro=gyro, accel=accel, mag=mag)


def _sanitize_quaternion(quat: Quaternion) -> tuple[Quaternion, bool]:
    stale = False

    def _component(val: Optional[float], fallback: float) -> float:
        nonlocal stale
        if val is None or not math.isfinite(val):
            stale = True
            return fallback
        return float(val)

    return (
        Quaternion(
            w=_component(quat.w, 1.0),
            x=_component(quat.x, 0.0),
            y=_component(quat.y, 0.0),
            z=_component(quat.z, 0.0),
        ),
        stale,
    )


def _sanitize_euler(euler: Optional[EulerDeg]) -> tuple[Optional[EulerDeg], bool]:
    if euler is None:
        return None, True

    stale = False

    def _angle(val: Optional[float]) -> Optional[float]:
        nonlocal stale
        if val is None:
            stale = True
            return None
        if not math.isfinite(val):
            stale = True
            return None
        return float(val)

    sanitized = EulerDeg(
        roll=_angle(euler.roll),
        pitch=_angle(euler.pitch),
        yaw=_angle(euler.yaw),
    )
    return sanitized, stale


def _read_sensor_snapshot() -> tuple[Optional[BNO055Data], Optional["OrientationSample"]]:
    sensor = _ensure_sensor()
    if sensor is None:
        return None, None

    try:
        with _READ_LOCK:
            euler = sensor.euler
            quat = sensor.quaternion
            ang_vel = sensor.gyro
            accel = sensor.acceleration
            mag = sensor.magnetic
            lin_accel = sensor.linear_acceleration
            gravity = sensor.gravity
            temp_c = sensor.temperature
            calib = getattr(sensor, "calibration_status", None)
    except Exception as exc:  # pragma: no cover - hardware path
        log.debug("BNO055 read error: %s", exc)
        _reset_sensor()
        return None, None

    base_euler = _euler(euler)
    base_quat = _quat(quat)

    quat_model, quat_stale = _sanitize_quaternion(base_quat)
    euler_model, euler_stale = _sanitize_euler(base_euler)

    data = BNO055Data(
        euler=euler_model,
        quat=quat_model,
        ang_vel_rad_s=_vec3(ang_vel),
        accel_m_s2=_vec3(accel),
        mag_uT=_vec3(mag),
        lin_accel_m_s2=_vec3(lin_accel),
        gravity_m_s2=_vec3(gravity),
        temp_c=float(temp_c) if temp_c is not None else None,
        calibration=_calibration(calib),
    )

    orientation = OrientationSample(
        quaternion=quat_model,
        euler=euler_model,
        calibration=data.calibration,
        stale=quat_stale or euler_stale,
    )

    return data, orientation


def _update_cache(data: Optional[BNO055Data], orientation: Optional["OrientationSample"]) -> None:
    global _latest_data, _latest_orientation, _latest_timestamp
    with _CACHE_LOCK:
        if data is not None:
            _latest_data = data
        if orientation is not None:
            _latest_orientation = orientation
        _latest_timestamp = time.monotonic()


def _clear_cache() -> None:
    global _latest_data, _latest_orientation, _latest_timestamp
    with _CACHE_LOCK:
        _latest_data = None
        _latest_orientation = None
        _latest_timestamp = 0.0


def read_bno055() -> Optional[BNO055Data]:
    """Read all available BNO055 channels and return them as a schema object."""

    with _CACHE_LOCK:
        cached = _latest_data
    if cached is not None:
        return cached

    data, orientation = _read_sensor_snapshot()
    if data is None and orientation is None:
        return None

    _update_cache(data, orientation)
    with _CACHE_LOCK:
        return _latest_data


@dataclass
class OrientationSample:
    quaternion: Quaternion
    euler: Optional[EulerDeg]
    calibration: Optional[BNO055CalibrationStatus]
    stale: bool = False


def read_orientation_sample() -> Optional[OrientationSample]:
    with _CACHE_LOCK:
        cached = _latest_orientation
    if cached is not None:
        return cached

    data, orientation = _read_sensor_snapshot()
    if data is None and orientation is None:
        return None

    _update_cache(data, orientation)
    with _CACHE_LOCK:
        return _latest_orientation


def start_bno055_calibration() -> bool:
    sensor = _ensure_sensor()
    if sensor is None or adafruit_bno055 is None:
        return False

    try:
        config_mode = getattr(adafruit_bno055, "CONFIG_MODE", None)
        ndof_mode = getattr(adafruit_bno055, "NDOF_MODE", None)

        with _READ_LOCK:
            if hasattr(sensor, "mode") and config_mode is not None:
                sensor.mode = config_mode
                time.sleep(0.1)

            reset = getattr(sensor, "reset", None)
            if callable(reset):
                try:
                    reset()
                    time.sleep(0.5)
                except Exception as exc:  # pragma: no cover - hardware path
                    log.debug("BNO055 reset during calibration start failed: %s", exc)

            if hasattr(sensor, "mode") and ndof_mode is not None:
                sensor.mode = ndof_mode

        # Clear cache so next read reflects fresh calibration
        _clear_cache()

        return True
    except Exception as exc:  # pragma: no cover - hardware path
        log.warning("Failed to start BNO055 calibration: %s", exc)
        _reset_sensor()
        return False


def save_bno055_offsets() -> bool:
    sensor = _ensure_sensor()
    if sensor is None:
        log.error("Cannot save BNO055 calibration: sensor unavailable")
        return False

    try:
        with _READ_LOCK:
            # Read calibration offsets using individual properties
            # These properties automatically switch to CONFIG_MODE
            accel_offsets = sensor.offsets_accelerometer  # type: ignore[attr-defined]
            mag_offsets = sensor.offsets_magnetometer  # type: ignore[attr-defined]
            gyro_offsets = sensor.offsets_gyroscope  # type: ignore[attr-defined]
            accel_radius = sensor.radius_accelerometer  # type: ignore[attr-defined]
            mag_radius = sensor.radius_magnetometer  # type: ignore[attr-defined]

        payload = {
            "offsets_accelerometer": list(accel_offsets),
            "offsets_magnetometer": list(mag_offsets),
            "offsets_gyroscope": list(gyro_offsets),
            "radius_accelerometer": int(accel_radius),
            "radius_magnetometer": int(mag_radius),
            "saved_at": time.time(),
        }

        _CALIBRATION_PATH.parent.mkdir(parents=True, exist_ok=True)
        with _CALIBRATION_PATH.open("w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2, sort_keys=True)
        log.info("Saved BNO055 calibration offsets to %s", _CALIBRATION_PATH)
        return True
    except Exception as exc:
        log.error("Failed to save BNO055 calibration offsets: %s", exc)
        return False


class _BNO055Poller(threading.Thread):
    def __init__(self, rate_hz: float) -> None:
        super().__init__(daemon=True)
        self._rate_hz = max(1.0, float(rate_hz))
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        interval = 1.0 / self._rate_hz
        failure_count = 0
        while not self._stop_event.is_set():
            started = time.monotonic()
            data, orientation = _read_sensor_snapshot()
            if data is not None or orientation is not None:
                _update_cache(data, orientation)
                failure_count = 0
            else:
                failure_count += 1
                if failure_count % int(self._rate_hz) == 0:
                    log.debug("BNO055 poller still waiting for sensor data")
            elapsed = time.monotonic() - started
            wait = interval - elapsed
            if wait < 0.0:
                wait = interval
            self._stop_event.wait(wait)


def start_bno055_poller(rate_hz: float = _DEFAULT_POLL_RATE_HZ) -> None:
    global _poller
    if _poller is not None and _poller.is_alive():
        return
    poller = _BNO055Poller(rate_hz)
    poller.start()
    _poller = poller
    log.info("Started BNO055 poller at %.1f Hz", rate_hz)


def stop_bno055_poller(timeout: float = 1.0) -> None:
    global _poller
    poller = _poller
    if poller is None:
        return
    poller.stop()
    poller.join(timeout=timeout)
    if poller.is_alive():
        log.warning("BNO055 poller did not stop cleanly; continuing shutdown")
    _poller = None
