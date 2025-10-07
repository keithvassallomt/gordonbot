#!/usr/bin/env python3
"""
Odometry Bridge Node - Reads encoder data from GordonBot backend and publishes odometry.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import aiohttp
import asyncio
import threading
import math


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # Declare parameters
        self.declare_parameter('backend_url', 'http://localhost:8000')
        self.declare_parameter('poll_rate_hz', 20.0)
        self.declare_parameter('wheel_base_m', 0.14)  # Distance between wheels
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Get parameters
        self.backend_url = self.get_parameter('backend_url').value
        self.poll_rate = self.get_parameter('poll_rate_hz').value
        self.wheel_base = self.get_parameter('wheel_base_m').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Publisher and broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Previous encoder values
        self.prev_left_dist = None
        self.prev_right_dist = None

        self.get_logger().info(f'Odometry bridge starting, polling {self.backend_url}/api/sensors at {self.poll_rate}Hz')

        # Start polling thread
        self.poll_thread = threading.Thread(target=self._run_polling, daemon=True)
        self.poll_thread.start()

    def _run_polling(self):
        """Run polling in asyncio event loop."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._poll_encoders())

    async def _poll_encoders(self):
        """Poll encoder data from backend and compute odometry."""
        async with aiohttp.ClientSession() as session:
            while rclpy.ok():
                try:
                    # Fetch sensor data
                    async with session.get(f"{self.backend_url}/api/sensors") as resp:
                        if resp.status == 200:
                            data = await resp.json()
                            self._update_odometry(data.get('encoders', {}))

                    await asyncio.sleep(1.0 / self.poll_rate)

                except Exception as e:
                    self.get_logger().warn(f'Encoder polling error: {e}')
                    await asyncio.sleep(1.0)

    def _update_odometry(self, encoders):
        """Update odometry from encoder data."""
        left = encoders.get('left', {})
        right = encoders.get('right', {})

        left_dist = left.get('distance_m')
        right_dist = right.get('distance_m')

        if left_dist is None or right_dist is None:
            return

        # Initialize on first reading
        if self.prev_left_dist is None:
            self.prev_left_dist = left_dist
            self.prev_right_dist = right_dist
            return

        # Calculate distance traveled by each wheel
        delta_left = left_dist - self.prev_left_dist
        delta_right = right_dist - self.prev_right_dist

        # Update previous values
        self.prev_left_dist = left_dist
        self.prev_right_dist = right_dist

        # Differential drive odometry
        # Calculate distance traveled (average of both wheels)
        delta_dist = (delta_left + delta_right) / 2.0

        # NOTE: We do NOT calculate theta from encoders - it's garbage!
        # Orientation comes from IMU via EKF instead.
        # delta_theta = (delta_right - delta_left) / self.wheel_base  # REMOVED

        # Update position only (orientation comes from EKF/IMU)
        # Use current theta from EKF for position update
        self.x += delta_dist * math.cos(self.theta)
        self.y += delta_dist * math.sin(self.theta)
        # self.theta += delta_theta  # REMOVED - no encoder-based orientation!

        # Publish odometry
        self._publish_odometry()

    def _publish_odometry(self):
        """Publish odometry message and TF transform."""
        now = self.get_clock().now()

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from theta)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Covariance (placeholder, tune based on encoder accuracy)
        odom.pose.covariance[0] = 0.01  # x
        odom.pose.covariance[7] = 0.01  # y
        odom.pose.covariance[35] = 0.05  # theta

        # Publish
        self.odom_pub.publish(odom)

        # NOTE: TF broadcasting disabled - EKF publishes odom→base_link TF instead
        # The EKF fuses wheel odometry (position) with IMU (orientation)
        # This eliminates encoder-based orientation from the TF tree
        #
        # Broadcast TF
        # t = TransformStamped()
        # t.header.stamp = now.to_msg()
        # t.header.frame_id = self.odom_frame
        # t.child_frame_id = self.base_frame
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation.z = qz
        # t.transform.rotation.w = qw
        #
        # self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
