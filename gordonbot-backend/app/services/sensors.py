from __future__ import annotations

import time
import logging
from typing import Optional

from app.schemas import (
    SensorsStatus,
    MotorEncoders,
    EncoderData,
    ToFData,
    BNO055Data,
)
from app.core.config import settings
from app.services.encoder import get_left_encoder, get_right_encoder
from app.services.tof import read_distance_mm
from app.services.bno055 import read_bno055

log = logging.getLogger(__name__)


def get_sensor_status() -> SensorsStatus:
    """Return a consolidated snapshot of sensor readings.

    Designed to be safe on dev machines without hardware attached. If a
    specific sensor is unavailable, its field is left as None.
    """
    ts_ms = int(time.time() * 1000)

    encoders: Optional[MotorEncoders] = None
    tof: Optional[ToFData] = None
    bno: Optional[BNO055Data] = None

    # Encoders: left and right
    try:
        left = get_left_encoder(
            settings.encoder_left_a,
            settings.encoder_left_b,
            settings.encoder_counts_per_rev_output,
            settings.wheel_diameter_m,
        )
        left.set_scales(settings.encoder_scale_fwd, settings.encoder_scale_rev)
        left_connected = left.connected()
        left_ticks = left.ticks()
        left_dist_m = left.distance_m() if left_connected else None
        left_dist_mm = (left_dist_m * 1000.0) if (left_dist_m is not None) else None
        left_rpm = left.rpm() if left_connected else None
        left_spd_mm_s = left.speed_mm_s() if left_connected else None

        right = get_right_encoder(
            settings.encoder_right_a,
            settings.encoder_right_b,
            settings.encoder_counts_per_rev_output,
            settings.wheel_diameter_m,
        )
        right.set_scales(settings.encoder_scale_fwd_right, settings.encoder_scale_rev_right)
        right_connected = right.connected()
        right_ticks = right.ticks()
        right_dist_m = right.distance_m() if right_connected else None
        right_dist_mm = (right_dist_m * 1000.0) if (right_dist_m is not None) else None
        right_rpm = right.rpm() if right_connected else None
        right_spd_mm_s = right.speed_mm_s() if right_connected else None

        encoders = MotorEncoders(
            left=EncoderData(
                connected=left_connected,
                ticks=left_ticks,
                distance_m=left_dist_m,
                distance_mm=left_dist_mm,
                rpm=left_rpm,
                speed_mm_s=left_spd_mm_s,
            ),
            right=EncoderData(
                connected=right_connected,
                ticks=right_ticks,
                distance_m=right_dist_m,
                distance_mm=right_dist_mm,
                rpm=right_rpm,
                speed_mm_s=right_spd_mm_s,
            ),
        )
    except Exception as e:
        log.debug("Encoders read failed: %s", e)

    # ToF distance sensor
    try:
        distance_mm = read_distance_mm()
        if distance_mm is not None:
            tof = ToFData(distance_mm=distance_mm)
    except Exception as e:
        log.debug("ToF read failed: %s", e)

    # BNO055 IMU
    try:
        bno = read_bno055()
    except Exception as e:
        log.debug("BNO055 read failed: %s", e)
        bno = None

    return SensorsStatus(ts=ts_ms, encoders=encoders, tof=tof, bno055=bno)
