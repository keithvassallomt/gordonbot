#!/usr/bin/env python3
"""
Odometry Bridge Node - Reads encoder data from GordonBot backend and publishes odometry.

VELOCITY-ONLY MODE: Publishes encoder-based linear velocity for motion detection only.
Position/orientation stay at origin - SLAM figures those out via scan matching.
This allows motion filtering without trusting inaccurate encoder distance/heading.
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
        self.declare_parameter('velocity_only', True)  # NEW: velocity-only mode

        # Get parameters
        self.backend_url = self.get_parameter('backend_url').value
        self.poll_rate = self.get_parameter('poll_rate_hz').value
        self.wheel_base = self.get_parameter('wheel_base_m').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.velocity_only = self.get_parameter('velocity_only').value

        # Publisher and broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odometry state (position/orientation always at origin in velocity-only mode)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocity state
        self.linear_vel = 0.0   # m/s
        self.angular_vel = 0.0  # rad/s

        # Previous encoder values
        self.prev_left_dist = None
        self.prev_right_dist = None
        self.prev_time = None

        mode = "velocity-only (motion detection)" if self.velocity_only else "full odometry"
        self.get_logger().info(f'Odometry bridge starting in {mode} mode')
        self.get_logger().info(f'Polling {self.backend_url}/api/sensors at {self.poll_rate}Hz')

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
                            self._update_odometry(data)

                    await asyncio.sleep(1.0 / self.poll_rate)

                except Exception as e:
                    self.get_logger().warn(f'Encoder polling error: {e}')
                    await asyncio.sleep(1.0)

    def _update_odometry(self, sensor_data):
        """Update odometry from encoder data."""
        encoders = sensor_data.get('encoders', {}) if isinstance(sensor_data, dict) else {}
        left = encoders.get('left', {}) if isinstance(encoders, dict) else {}
        right = encoders.get('right', {}) if isinstance(encoders, dict) else {}
        imu = sensor_data.get('bno055') if isinstance(sensor_data, dict) else None

        left_dist = left.get('distance_m')
        right_dist = right.get('distance_m')

        if left_dist is None or right_dist is None:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9  # seconds

        # Initialize on first reading
        if self.prev_left_dist is None:
            self.prev_left_dist = left_dist
            self.prev_right_dist = right_dist
            self.prev_time = current_time
            return

        # Calculate distance traveled by each wheel
        delta_left = left_dist - self.prev_left_dist
        delta_right = right_dist - self.prev_right_dist
        dt = current_time - self.prev_time

        if dt <= 0:
            return

        # Update previous values
        self.prev_left_dist = left_dist
        self.prev_right_dist = right_dist
        self.prev_time = current_time

        # Differential drive kinematics
        delta_dist = (delta_left + delta_right) / 2.0  # Forward distance
        delta_theta = (delta_right - delta_left) / self.wheel_base  # Rotation

        # Calculate velocities
        self.linear_vel = delta_dist / dt  # m/s
        self.angular_vel = delta_theta / dt  # rad/s

        if self.velocity_only:
            # VELOCITY-ONLY MODE: Position and orientation stay at origin
            # This gives SLAM ONLY velocity for motion detection
            # IMU orientation comes from imu_odom_bridge on /odom topic
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
        else:
            # FULL ODOMETRY MODE: Update orientation from IMU, integrate position
            imu_yaw_deg = None
            if isinstance(imu, dict):
                euler = imu.get('euler') if isinstance(imu.get('euler'), dict) else None
                if euler and euler.get('yaw') is not None:
                    imu_yaw_deg = float(euler['yaw'])
            if imu_yaw_deg is not None and math.isfinite(imu_yaw_deg):
                theta_raw = math.radians(imu_yaw_deg)
                self.theta = math.atan2(math.sin(theta_raw), math.cos(theta_raw))
            else:
                self.theta += delta_theta

            # Update position using latest heading
            self.x += delta_dist * math.cos(self.theta)
            self.y += delta_dist * math.sin(self.theta)

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

        # Position (always origin in velocity-only mode)
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation (quaternion from theta)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity (THIS IS THE KEY FOR MOTION FILTERING)
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.angular_vel

        # Covariance
        if self.velocity_only:
            # High position uncertainty (we don't know where we are)
            odom.pose.covariance[0] = 999.0  # x - huge uncertainty
            odom.pose.covariance[7] = 999.0  # y - huge uncertainty
            odom.pose.covariance[35] = 999.0  # theta - huge uncertainty
            # Low velocity uncertainty (encoders are decent at detecting motion)
            odom.twist.covariance[0] = 0.1  # linear x
            odom.twist.covariance[35] = 0.2  # angular z
        else:
            # Normal odometry covariances
            odom.pose.covariance[0] = 0.01  # x
            odom.pose.covariance[7] = 0.01  # y
            odom.pose.covariance[35] = 0.05  # theta
            odom.twist.covariance[0] = 0.01
            odom.twist.covariance[35] = 0.05

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
