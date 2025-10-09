#!/usr/bin/env python3
"""
IMU Odometry Bridge - Publishes odometry from IMU only (no encoders).
Orientation from IMU, position at origin (SLAM handles position via scan matching).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class ImuOdomBridge(Node):
    def __init__(self):
        super().__init__('imu_odom_bridge')

        # Declare parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Get parameters
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Publisher and broadcaster
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self._imu_callback,
            10
        )

        # State - only track IMU messages
        self.latest_imu = None

        self.get_logger().info(f'IMU Odometry Bridge started')
        self.get_logger().info(f'Publishing odometry with IMU orientation only')
        self.get_logger().info(f'Position stays at origin - SLAM handles position via scan matching')

    def _imu_callback(self, msg):
        """Callback for IMU messages."""
        self.latest_imu = msg
        self._publish_odometry()

    def _publish_odometry(self):
        """Publish odometry message and TF with IMU orientation only."""
        if self.latest_imu is None:
            return

        now = self.get_clock().now()

        # Create odometry message
        # Position at origin (SLAM provides position via map→odom transform)
        # Orientation directly from IMU
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position: origin (SLAM handles actual position via map→odom TF)
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        rotated = self._rotate_quaternion_z(
            self.latest_imu.orientation.x,
            self.latest_imu.orientation.y,
            self.latest_imu.orientation.z,
            self.latest_imu.orientation.w,
            math.pi,
        )
        odom.pose.pose.orientation.x = rotated[0]
        odom.pose.pose.orientation.y = rotated[1]
        odom.pose.pose.orientation.z = rotated[2]
        odom.pose.pose.orientation.w = rotated[3]

        # Velocity: angular velocity from IMU, no linear velocity
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular = self.latest_imu.angular_velocity

        # Covariance: IMU orientation is accurate, position unknown
        odom.pose.covariance[0] = 1e9   # x - infinite (unknown)
        odom.pose.covariance[7] = 1e9   # y - infinite (unknown)
        odom.pose.covariance[35] = 0.01 # yaw - low covariance (accurate from IMU)

        # Publish odometry
        self.odom_pub.publish(odom)

        # Broadcast TF: odom → base_link with IMU orientation only
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = rotated[0]
        t.transform.rotation.y = rotated[1]
        t.transform.rotation.z = rotated[2]
        t.transform.rotation.w = rotated[3]

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _rotate_quaternion_z(qx: float, qy: float, qz: float, qw: float, angle_rad: float):
        half = angle_rad / 2.0
        rz_x = 0.0
        rz_y = 0.0
        rz_z = math.sin(half)
        rz_w = math.cos(half)

        out_x = rz_w * qx + rz_x * qw + rz_y * qz - rz_z * qy
        out_y = rz_w * qy - rz_x * qz + rz_y * qw + rz_z * qx
        out_z = rz_w * qz + rz_x * qy - rz_y * qx + rz_z * qw
        out_w = rz_w * qw - rz_x * qx - rz_y * qy - rz_z * qz

        norm = math.sqrt(out_x * out_x + out_y * out_y + out_z * out_z + out_w * out_w)
        if norm == 0.0:
            return qx, qy, qz, qw
        return out_x / norm, out_y / norm, out_z / norm, out_w / norm


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
