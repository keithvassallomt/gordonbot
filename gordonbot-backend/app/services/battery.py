import logging

log = logging.getLogger(__name__)

try:
    from smbus2 import SMBus  # type: ignore
except Exception as e:  # pragma: no cover - dev machines often lack smbus
    SMBus = None  # type: ignore
    log.warning("SMBus unavailable; battery telemetry will be stubbed: %s", e)

ADDR = 0x2D

def _open_bus():
    if SMBus is None:
        return None
    try:
        return SMBus(1)
    except Exception as e:  # pragma: no cover
        log.warning("Failed to open I2C bus for battery: %s", e)
        return None


def get_battery_status():
    """Get battery status and readings from UPS HAT or return a safe stub.

    Returns a dict with keys expected by the router. On non‑hardware/dev
    systems, returns a stub with percent=0 and optional fields None.
    """
    bus = _open_bus()
    if bus is None:
        return {
            "battery_status": "idle",
            "vbus_voltage": None,
            "vbus_current": None,
            "vbus_power": None,
            "battery_voltage": None,
            "current": None,
            "battery_percent": 0,
            "remaining_capacity": None,
            "run_time_to_empty": None,
            "avg_time_to_full": None,
            "cells": None,
        }

    try:
        # Status
        data = bus.read_i2c_block_data(ADDR, 0x02, 0x01)
        status_raw = data[0]
        battery_status = "idle"
        if status_raw & 0x40:
            battery_status = "fast_charging"
        elif status_raw & 0x80:
            battery_status = "charging"
        elif status_raw & 0x20:
            battery_status = "discharging"

        # VBUS block
        data = bus.read_i2c_block_data(ADDR, 0x10, 0x06)
        vbus_voltage = (data[0] | data[1] << 8) / 1000.0
        vbus_current = (data[2] | data[3] << 8) / 1000.0
        vbus_power = (data[4] | data[5] << 8) / 1000.0

        # Battery block
        data = bus.read_i2c_block_data(ADDR, 0x20, 0x0C)
        battery_voltage = (data[0] | data[1] << 8) / 1000.0
        current = (data[2] | data[3] << 8)
        if current > 0x7FFF:
            current -= 0xFFFF
        battery_percent = int(data[4] | data[5] << 8)
        remaining_capacity = (data[6] | data[7] << 8)
        run_time_to_empty = (data[8] | data[9] << 8) if current < 0 else None
        avg_time_to_full = (data[10] | data[11] << 8) if current >= 0 else None

        # Cell voltages
        cells = []
        data = bus.read_i2c_block_data(ADDR, 0x30, 0x08)
        for i in range(0, 8, 2):
            cells.append((data[i] | data[i + 1] << 8) / 1000.0)

        return {
            "battery_status": battery_status,
            "vbus_voltage": vbus_voltage,
            "vbus_current": vbus_current,
            "vbus_power": vbus_power,
            "battery_voltage": battery_voltage,
            "current": current,
            "battery_percent": battery_percent,
            "remaining_capacity": remaining_capacity,
            "run_time_to_empty": run_time_to_empty,
            "avg_time_to_full": avg_time_to_full,
            "cells": cells,
        }
    except Exception as e:  # pragma: no cover
        log.warning("Battery read failed; returning stub: %s", e)
        return {
            "battery_status": "idle",
            "vbus_voltage": None,
            "vbus_current": None,
            "vbus_power": None,
            "battery_voltage": None,
            "current": None,
            "battery_percent": 0,
            "remaining_capacity": None,
            "run_time_to_empty": None,
            "avg_time_to_full": None,
            "cells": None,
        }
