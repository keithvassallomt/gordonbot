#!/usr/bin/env python3
"""
Map Bridge Node - Subscribes to /map from slam_toolbox and serves via WebSocket.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import asyncio
import websockets
import json
import threading
import math
from datetime import datetime


class MapBridge(Node):
    def __init__(self):
        super().__init__('map_bridge')

        # Declare parameters
        self.declare_parameter('ws_port', 9001)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_rate_hz', 10.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # Get parameters
        self.ws_port = self.get_parameter('ws_port').value
        map_topic = self.get_parameter('map_topic').value
        self.pose_rate = self.get_parameter('pose_rate_hz').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # TF2 buffer and listener for pose extraction
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self._map_callback,
            10
        )

        # Timer for periodic pose updates
        self.pose_timer = self.create_timer(
            1.0 / self.pose_rate,
            self._update_pose
        )

        # Latest data
        self.latest_map = None
        self.latest_pose = None
        self.map_lock = threading.Lock()
        self.pose_lock = threading.Lock()

        # Connected WebSocket clients
        self._ws_clients = set()

        self.get_logger().info(f'Map bridge starting WebSocket server on port {self.ws_port}')
        self.get_logger().info(f'Publishing pose at {self.pose_rate}Hz from TF: {self.map_frame} -> {self.base_frame}')

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

    def _update_pose(self):
        """Extract robot pose from TF transform (map -> base_link)."""
        try:
            # Look up the transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()  # Get latest available transform
            )

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Extract orientation (quaternion -> yaw)
            quat = transform.transform.rotation
            # Convert quaternion to yaw (theta)
            # For 2D: yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
            theta = math.atan2(
                2.0 * (quat.w * quat.z + quat.x * quat.y),
                1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
            )

            with self.pose_lock:
                self.latest_pose = {
                    'type': 'pose',
                    'ts': int(datetime.now().timestamp() * 1000),
                    'x': x,
                    'y': y,
                    'theta': theta,
                    'frame_id': self.map_frame
                }

            # Broadcast to all connected clients
            asyncio.run(self._broadcast_pose())

        except TransformException as ex:
            # Transform not available yet (expected at startup)
            pass

    async def _broadcast_map(self):
        """Broadcast latest map to all clients."""
        if not self._ws_clients or not self.latest_map:
            return

        message = json.dumps(self.latest_map)
        disconnected = set()

        for client in self._ws_clients:
            try:
                await client.send(message)
            except:
                disconnected.add(client)

        # Remove disconnected clients
        self._ws_clients -= disconnected

    async def _broadcast_pose(self):
        """Broadcast latest pose to all clients."""
        if not self._ws_clients or not self.latest_pose:
            return

        message = json.dumps(self.latest_pose)
        disconnected = set()

        for client in self._ws_clients:
            try:
                await client.send(message)
            except:
                disconnected.add(client)

        self._ws_clients -= disconnected

    def _run_websocket_server(self):
        """Run WebSocket server."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        async def handler(websocket):
            self._ws_clients.add(websocket)
            self.get_logger().info(f'Client connected, total clients: {len(self._ws_clients)}')

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
                self._ws_clients.discard(websocket)
                self.get_logger().info(f'Client disconnected, total clients: {len(self._ws_clients)}')

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
