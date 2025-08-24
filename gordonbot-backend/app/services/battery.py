import time
import os
import logging

from smbus2 import SMBus

ADDR = 0x2d
LOW_VOL = 3150 # mV
bus = SMBus(1)

def get_battery_status():
    """Get battery status and readings from UPS HAT or return mock data"""
    
    # Read battery status register
    data = bus.read_i2c_block_data(ADDR, 0x02, 0x01)
    
    battery_status = 'idle'
    if data[0] & 0x40:
        battery_status = 'fast_charging'
    elif data[0] & 0x80:
        battery_status = 'charging'
    elif data[0] & 0x20:
        battery_status = 'discharging'
        
    # Read VBUS data (USB power input)
    data = bus.read_i2c_block_data(ADDR, 0x10, 0x06)
    vbus_voltage = (data[0] | data[1] << 8)  # in mV
    vbus_current = (data[2] | data[3] << 8)  # in mA
    vbus_power = (data[4] | data[5] << 8)    # in mW
        
    # Read battery data
    data = bus.read_i2c_block_data(ADDR, 0x20, 0x0C)
    battery_voltage = (data[0] | data[1] << 8)  # in mV
    
    current = (data[2] | data[3] << 8)  # in mA
    if current > 0x7FFF:
        current -= 0xFFFF
    
    battery_percent = int(data[4] | data[5] << 8)  # in %
    remaining_capacity = (data[6] | data[7] << 8)  # in mAh
    
    run_time_to_empty = data[8] | data[9] << 8 if current < 0 else None  # in min
    avg_time_to_full = data[10] | data[11] << 8 if current >= 0 else None  # in min
    
    # Read individual cell voltages
    cells = []
    data = bus.read_i2c_block_data(ADDR, 0x30, 0x08)
    for i in range(0, 8, 2):
        cell_voltage = (data[i] | data[i+1] << 8)
        cells.append(cell_voltage / 1000.0)  # convert to V  
    
    return {
        "battery_status": battery_status,
        "vbus_voltage": vbus_voltage / 1000.0,       # convert mV to V
        "vbus_current": vbus_current / 1000.0,       # convert mA to A
        "vbus_power": vbus_power / 1000.0,           # convert mW to W
        "battery_voltage": battery_voltage / 1000.0, # convert mV to V
        "current": current,                          # still in mA
        "battery_percent": battery_percent,
        "remaining_capacity": remaining_capacity,
        "run_time_to_empty": run_time_to_empty,
        "avg_time_to_full": avg_time_to_full,
        "cells": cells,                              # already in V
    }
