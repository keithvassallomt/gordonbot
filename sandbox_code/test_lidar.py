#!/usr/bin/env python3
"""
Simple test script to verify RPLIDAR C1 connection and basic scanning.
Run this to test if the LIDAR hardware is working correctly.

Usage:
    cd /home/keith/gordonbot/gordonbot-backend
    . .venv/bin/activate
    python3 ../test_lidar.py
"""

import asyncio
import sys
from rplidarc1 import RPLidar

PORT = "/dev/ttyUSB0"
BAUDRATE = 460800
TIMEOUT = 0.2


async def test_connection():
    """Test basic connection to LIDAR."""
    print(f"Attempting to connect to RPLIDAR C1 on {PORT} at {BAUDRATE} baud...")

    try:
        lidar = RPLidar(PORT, BAUDRATE, TIMEOUT)
        print("✓ LIDAR instance created")

        # Note: get_info() and healthcheck() are not implemented in rplidarc1
        # Connection will be verified by successful scanning

        # Try a short scan
        print("\nStarting 5-second scan test...")
        scan_task = asyncio.create_task(lidar.simple_scan(make_return_dict=True))

        # Collect some points
        points_collected = 0
        start_time = asyncio.get_event_loop().time()

        try:
            while asyncio.get_event_loop().time() - start_time < 5.0:
                try:
                    point = await asyncio.wait_for(lidar.output_queue.get(), timeout=1.0)
                    points_collected += 1

                    if points_collected == 1:
                        print(f"✓ First point received: {point}")

                    if points_collected % 100 == 0:
                        print(f"  Collected {points_collected} points...")

                except asyncio.TimeoutError:
                    print("  Waiting for data...")
                    continue

        finally:
            # Stop scan
            lidar.stop_event.set()
            await scan_task

        print(f"\n✓ Scan test complete! Collected {points_collected} points in 5 seconds")
        print(f"  Estimated scan rate: ~{points_collected / 5:.0f} points/second")

        # Shutdown
        await asyncio.to_thread(lidar.shutdown)
        print("✓ LIDAR shutdown successful")

        return True

    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


async def main():
    success = await test_connection()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    asyncio.run(main())
