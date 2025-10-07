#!/usr/bin/env python3
"""Test orientation WebSocket connection."""

import asyncio
import websockets
import json

async def test_orientation_ws():
    uri = "ws://localhost:8000/ws/orientation"

    print(f"Connecting to {uri}...")
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected!")

            # Receive a few messages
            for i in range(5):
                try:
                    data = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                    msg = json.loads(data)
                    print(f"[{i+1}] {msg.get('type', 'unknown')}: {json.dumps(msg, indent=2)[:200]}")
                except asyncio.TimeoutError:
                    print(f"[{i+1}] Timeout waiting for message")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(test_orientation_ws())
