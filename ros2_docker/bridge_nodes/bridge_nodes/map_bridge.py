#!/usr/bin/env python3
"""
Map Bridge Node - Subscribes to /map from slam_toolbox and serves via WebSocket.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import asyncio
import websockets
import json
import threading
from datetime import datetime


class MapBridge(Node):
    def __init__(self):
        super().__init__('map_bridge')

        # Declare parameters
        self.declare_parameter('ws_port', 9001)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/pose')

        # Get parameters
        self.ws_port = self.get_parameter('ws_port').value
        map_topic = self.get_parameter('map_topic').value
        pose_topic = self.get_parameter('pose_topic').value

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self._map_callback,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            pose_topic,
            self._pose_callback,
            10
        )

        # Latest data
        self.latest_map = None
        self.latest_pose = None
        self.map_lock = threading.Lock()
        self.pose_lock = threading.Lock()

        # Connected WebSocket clients
        self._clients = set()

        self.get_logger().info(f'Map bridge starting WebSocket server on port {self.ws_port}')

        # Start WebSocket server in separate thread
        self.ws_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
        self.ws_thread.start()

    def _map_callback(self, msg):
        """Callback for /map topic."""
        with self.map_lock:
            # Convert OccupancyGrid to JSON-serializable format
            self.latest_map = {
                'type': 'map',
                'ts': int(datetime.now().timestamp() * 1000),
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                    'theta': 0.0  # Could extract from quaternion if needed
                },
                # Convert data to list (values: -1=unknown, 0-100=occupancy probability)
                'data': list(msg.data)
            }

        # Broadcast to all connected clients
        asyncio.run(self._broadcast_map())

    def _pose_callback(self, msg):
        """Callback for /pose topic."""
        with self.pose_lock:
            self.latest_pose = {
                'type': 'pose',
                'ts': int(datetime.now().timestamp() * 1000),
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z,
                'orientation': {
                    'x': msg.pose.orientation.x,
                    'y': msg.pose.orientation.y,
                    'z': msg.pose.orientation.z,
                    'w': msg.pose.orientation.w
                }
            }

        # Broadcast to all connected clients
        asyncio.run(self._broadcast_pose())

    async def _broadcast_map(self):
        """Broadcast latest map to all clients."""
        if not self._clients or not self.latest_map:
            return

        message = json.dumps(self.latest_map)
        disconnected = set()

        for client in self._clients:
            try:
                await client.send(message)
            except:
                disconnected.add(client)

        # Remove disconnected clients
        self._clients -= disconnected

    async def _broadcast_pose(self):
        """Broadcast latest pose to all clients."""
        if not self._clients or not self.latest_pose:
            return

        message = json.dumps(self.latest_pose)
        disconnected = set()

        for client in self._clients:
            try:
                await client.send(message)
            except:
                disconnected.add(client)

        self._clients -= disconnected

    def _run_websocket_server(self):
        """Run WebSocket server."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def handler(websocket):
            self._clients.add(websocket)
            self.get_logger().info(f'Client connected, total clients: {len(self._clients)}')

            try:
                # Send latest data immediately on connect
                if self.latest_map:
                    await websocket.send(json.dumps(self.latest_map))
                if self.latest_pose:
                    await websocket.send(json.dumps(self.latest_pose))

                # Keep connection alive and handle pings
                async for message in websocket:
                    try:
                        msg = json.loads(message)
                        if msg.get('type') == 'ping':
                            await websocket.send(json.dumps({
                                'type': 'pong',
                                'ts': msg.get('ts', 0)
                            }))
                    except:
                        pass

            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                self._clients.discard(websocket)
                self.get_logger().info(f'Client disconnected, total clients: {len(self._clients)}')

        async def start_server():
            async with websockets.serve(handler, '0.0.0.0', self.ws_port):
                await asyncio.Future()  # Run forever

        loop.run_until_complete(start_server())


def main(args=None):
    rclpy.init(args=args)
    node = MapBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
