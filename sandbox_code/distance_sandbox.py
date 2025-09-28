#!/usr/bin/env python3
"""Simple command-line tester for the VL53L1X time-of-flight distance sensor."""
import sys
import time

try:
    import board
    import adafruit_vl53l1x
    try:
        from adafruit_vl53l1x import DistanceMode
    except ImportError:
        DistanceMode = None
except ImportError as exc:
    sys.stderr.write(
        "Missing dependency: {0}.\n"
        "Install it with 'pip3 install adafruit-circuitpython-vl53l1x'.\n".format(exc)
    )
    sys.exit(1)


def main() -> None:
    # Use the Pi's default I2C bus (SCL/SDA pins)
    i2c = board.I2C()
    sensor = adafruit_vl53l1x.VL53L1X(i2c)

    # Distance mode constants differ by library version; prefer enum when available.
    if DistanceMode is not None:
        sensor.distance_mode = DistanceMode.LONG
    else:
        sensor.distance_mode = 2  # LONG mode fallback constant

    sensor.timing_budget = 100  # ms between measurements (20-1000 ms allowed)

    print("Starting continuous ranging; press Ctrl+C to stop.")
    sensor.start_ranging()

    try:
        while True:
            distance_mm = sensor.distance
            if distance_mm is None:
                print("Waiting for data...")
            else:
                print(
                    f"Distance: {distance_mm:5.0f} mm  (~{distance_mm / 10:.1f} cm)"
                )
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping measurement.")
    finally:
        sensor.stop_ranging()


if __name__ == "__main__":
    main()
