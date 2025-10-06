#!/usr/bin/env python3
"""Test script for SLAM WebSocket endpoint."""

import asyncio
import websockets
import json


async def test_slam_websocket():
    uri = "ws://localhost:8000/ws/slam"

    print(f"Connecting to {uri}...")

    try:
        async with websockets.connect(uri) as websocket:
            print("Connected!")

            # Receive messages for 10 seconds
            timeout = 10
            start = asyncio.get_event_loop().time()

            while asyncio.get_event_loop().time() - start < timeout:
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                    data = json.loads(message)

                    msg_type = data.get("type")

                    if msg_type == "map":
                        print(f"Received map: {data['width']}x{data['height']} @ {data['resolution']}m/cell")
                        print(f"  Data length: {len(data['data'])} cells")
                    elif msg_type == "pose":
                        print(f"Received pose: x={data['x']:.2f}, y={data['y']:.2f}, θ={data['theta']:.2f}rad")
                    else:
                        print(f"Received: {msg_type}")

                except asyncio.TimeoutError:
                    # Send ping
                    await websocket.send(json.dumps({"type": "ping", "ts": int(asyncio.get_event_loop().time() * 1000)}))

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    asyncio.run(test_slam_websocket())
