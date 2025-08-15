import time
from datetime import datetime
import spidev

import logging
import os

log_file = os.path.join(os.path.dirname(__file__), 'charging_log.log')
logging.basicConfig(
    filename=log_file,
    level=logging.INFO,
    format='%(asctime)s %(message)s'
)

# calibration
# VREF_REAL: actual ADC reference voltage (volts); tie ADC input to the Pi's 3.3 V pin, read raw_counts, then compute VREF_REAL = 3.3 * 1023 / raw_counts
VREF_REAL = 3.336  # measured actual Pi 3.3V rail voltage

# DIVIDER_RATIO: resistor divider ratio (unitless); apply known Vin to divider, measure Vadc at midpoint, then compute DIVIDER_RATIO = Vin / Vadc
DIVIDER_RATIO = (10220 + 4640) / 4640  # using R1=10.22kΩ, R2=4.64kΩ (≈3.203)

# CAL_SLOPE: gain correction from two-point calibration (unitless); m = (Vtrue2 - Vtrue1) / (Vmeas2 - Vmeas1)
CAL_SLOPE = 1.0

# CAL_OFFSET: offset correction from two-point calibration (volts); b = Vtrue1 - CAL_SLOPE * Vmeas1
CAL_OFFSET = 0.39

V_DROP = 0.0

# Set up SPI using hardware CS (CE0)
spi = spidev.SpiDev()
spi.open(0, 0)  # bus 0, device 0 (CE0)
spi.max_speed_hz = 1350000

def read_channel(channel):
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

def read_voltage(channel):
    value = read_channel(channel)
    voltage = (value * 3.3) / 1023
    return voltage


print("Monitoring battery voltage. Press Ctrl+C to stop.\n")

try:
    while True:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        NUM_SAMPLES = 10
        voltages = [read_voltage(0) for _ in range(NUM_SAMPLES)]
        avg_voltage = sum(voltages) / NUM_SAMPLES
        raw_voltage = voltages[0]  # First sample for reference
        # Divider ratio: V_batt = V_measured * (R1 + R2) / R2
        # Using R1 = 10.22kΩ, R2 = 4.64kΩ as measured
        DIVIDER_RATIO = (10220 + 4640) / 4640  # ≈ 3.203
        # Apply correction for voltage drop across Schottky diode (voltage divider is placed after diode, not directly on battery)
        # Based on observed discrepancy: script reads 26% while actual battery is around 83%, implying ~0.9V loss

        estimated_batt_voltage = avg_voltage * DIVIDER_RATIO * CAL_SLOPE + CAL_OFFSET

        # Estimate charge percentage (2S pack)
        max_voltage = 8.4
        min_voltage = 6.0
        charge_pct = max(0, min(100, (estimated_batt_voltage - min_voltage) / (max_voltage - min_voltage) * 100))

        print(f"[{timestamp}] Raw: {raw_voltage:.2f} V | Avg: {avg_voltage:.2f} V | Est. Batt: {estimated_batt_voltage:.2f} V | Charge: {charge_pct:.0f}%")
        logging.info(f"Raw: {raw_voltage:.2f} V | Avg: {avg_voltage:.2f} V | Est. Batt: {estimated_batt_voltage:.2f} V | Charge: {charge_pct:.0f}%")
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopped monitoring.")