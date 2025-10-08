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

        # Orientation: directly from IMU (already normalized in imu_bridge)
        odom.pose.pose.orientation = self.latest_imu.orientation

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
        t.transform.rotation = self.latest_imu.orientation

        self.tf_broadcaster.sendTransform(t)


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
