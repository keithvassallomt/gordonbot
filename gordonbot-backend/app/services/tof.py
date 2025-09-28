"""VL53L1X time-of-flight distance sensor service."""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional

log = logging.getLogger(__name__)

try:  # pragma: no cover - hardware-specific import
    import board  # type: ignore
    import adafruit_vl53l1x  # type: ignore
    _IMPORT_OK = True
except Exception as exc:  # pragma: no cover - dev machines without hardware
    board = None  # type: ignore
    adafruit_vl53l1x = None  # type: ignore
    _IMPORT_OK = False
    _IMPORT_ERR = exc
else:
    _IMPORT_ERR = None


_SENSOR_LOCK = threading.Lock()
_SENSOR: Optional["adafruit_vl53l1x.VL53L1X"] = None
_LAST_INIT_ATTEMPT = 0.0
_INIT_BACKOFF_SEC = 10.0


def _ensure_sensor() -> Optional["adafruit_vl53l1x.VL53L1X"]:
    global _SENSOR, _LAST_INIT_ATTEMPT

    if _SENSOR is not None:
        return _SENSOR

    if not _IMPORT_OK:
        if _IMPORT_ERR and log.isEnabledFor(logging.DEBUG):
            log.debug("VL53L1X import unavailable: %s", _IMPORT_ERR)
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
            sensor = adafruit_vl53l1x.VL53L1X(i2c)
            distance_mode = getattr(adafruit_vl53l1x, "DistanceMode", None)
            if distance_mode is not None:
                sensor.distance_mode = distance_mode.LONG
            else:
                # Fallback constant for older driver versions (2 = long)
                sensor.distance_mode = 2  # type: ignore[attr-defined]
            sensor.timing_budget = 100
            sensor.start_ranging()
            _SENSOR = sensor
            log.info("VL53L1X sensor initialised in long-distance mode")
        except Exception as exc:  # pragma: no cover - hardware path
            log.warning("VL53L1X init failed: %s", exc)
            _SENSOR = None
    return _SENSOR


def read_distance_mm() -> Optional[int]:
    """Return the latest distance measurement (mm) or None if unavailable."""
    sensor = _ensure_sensor()
    if sensor is None:
        return None
    try:
        distance = sensor.distance  # type: ignore[attr-defined]
    except Exception as exc:  # pragma: no cover - hardware path
        log.warning("VL53L1X read failed: %s", exc)
        _reset_sensor()
        return None
    if distance is None:
        return None
    try:
        return int(distance)
    except (TypeError, ValueError):
        return None


def _reset_sensor() -> None:
    global _SENSOR, _LAST_INIT_ATTEMPT
    with _SENSOR_LOCK:
        if _SENSOR is not None:
            try:
                _SENSOR.stop_ranging()
            except Exception:
                pass
        _SENSOR = None
        _LAST_INIT_ATTEMPT = 0.0
