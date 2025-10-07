#!/usr/bin/env python3
"""Test SLAM WebSocket to verify pose updates."""

import asyncio
import websockets
import json

async def test_slam_ws():
    uri = "ws://localhost:8000/ws/slam"

    print(f"Connecting to {uri}...")
    async with websockets.connect(uri) as websocket:
        print("Connected!")

        map_count = 0
        pose_count = 0

        # Receive 20 messages or timeout after 10 seconds
        try:
            for i in range(20):
                data = await asyncio.wait_for(websocket.recv(), timeout=10.0)
                msg = json.loads(data)

                if msg.get('type') == 'map':
                    map_count += 1
                    print(f"[{i+1}] MAP: {msg['width']}x{msg['height']} @ {msg['resolution']}m/cell")

                elif msg.get('type') == 'pose':
                    pose_count += 1
                    x = msg['x']
                    y = msg['y']
                    theta = msg.get('theta', 0)
                    theta_deg = theta * 180 / 3.14159
                    print(f"[{i+1}] POSE: x={x:.3f}m, y={y:.3f}m, theta={theta:.3f}rad ({theta_deg:.1f}°)")

        except asyncio.TimeoutError:
            print("\nTimeout waiting for messages")

        print(f"\nReceived {map_count} map updates and {pose_count} pose updates")

if __name__ == "__main__":
    asyncio.run(test_slam_ws())
