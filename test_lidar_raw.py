#!/usr/bin/env python3
"""Test what angles the RPLIDAR C1 is actually producing."""

import asyncio
from rplidarc1 import RPLidar

PORT = "/dev/ttyUSB0"
BAUDRATE = 460800

async def sample_points():
    lidar = RPLidar(PORT, BAUDRATE, 0.2)

    print("Starting scan, collecting 100 sample points...\n")
    scan_task = asyncio.create_task(lidar.simple_scan(make_return_dict=True))

    samples = []
    try:
        for i in range(100):
            point = await asyncio.wait_for(lidar.output_queue.get(), timeout=5.0)
            samples.append(point)

            if i < 10 or i % 10 == 0:
                angle = point.get("a_deg", 0)
                dist = point.get("d_mm", 0)
                qual = point.get("q", 0)
                print(f"Point {i:3d}: angle={angle:7.2f}° dist={dist:6.1f}mm quality={qual:2d}")
    finally:
        lidar.stop_event.set()
        await scan_task

    # Analyze
    print("\n=== Analysis ===")
    valid = [p for p in samples if 0 <= p.get("a_deg", -1) < 360]
    invalid_angle = [p for p in samples if p.get("a_deg", -1) >= 360 or p.get("a_deg", -1) < 0]
    zero_quality = [p for p in samples if p.get("q", 0) <= 0]

    print(f"Total points: {len(samples)}")
    print(f"Valid angles (0-360): {len(valid)} ({len(valid)/len(samples)*100:.1f}%)")
    print(f"Invalid angles: {len(invalid_angle)} ({len(invalid_angle)/len(samples)*100:.1f}%)")
    print(f"Zero quality: {len(zero_quality)} ({len(zero_quality)/len(samples)*100:.1f}%)")

    if valid:
        angles = [p.get("a_deg", 0) for p in valid]
        print(f"Angle range: {min(angles):.2f}° to {max(angles):.2f}°")

asyncio.run(sample_points())
