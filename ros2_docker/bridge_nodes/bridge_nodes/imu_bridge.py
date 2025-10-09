#!/usr/bin/env python3
"""
IMU Bridge Node - Reads BNO055 data from GordonBot backend and publishes to ROS2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import aiohttp
import asyncio
import threading
import math
from typing import Tuple


class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        # Declare parameters
        self.declare_parameter('backend_url', 'http://localhost:8000')
        self.declare_parameter('poll_rate_hz', 50.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('base_frame', 'base_link')

        # Get parameters
        self.backend_url = self.get_parameter('backend_url').value
        self.poll_rate = self.get_parameter('poll_rate_hz').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value

        # Publisher
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f'IMU bridge starting, polling {self.backend_url}/api/sensors at {self.poll_rate}Hz')

        # Start polling thread
        self.poll_thread = threading.Thread(target=self._run_polling, daemon=True)
        self.poll_thread.start()

    def _run_polling(self):
        """Run polling in asyncio event loop."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._poll_imu())

    async def _poll_imu(self):
        """Poll IMU data from backend."""
        async with aiohttp.ClientSession() as session:
            while rclpy.ok():
                try:
                    # Fetch sensor data
                    async with session.get(f"{self.backend_url}/api/sensors") as resp:
                        if resp.status == 200:
                            data = await resp.json()
                            self._publish_imu(data.get('bno055', {}))

                    await asyncio.sleep(1.0 / self.poll_rate)

                except Exception as e:
                    self.get_logger().warn(f'IMU polling error: {e}')
                    await asyncio.sleep(1.0)

    def _publish_imu(self, bno055_data):
        """Publish IMU message from BNO055 data."""
        quat = bno055_data.get('quat', {})
        ang_vel = bno055_data.get('ang_vel_rad_s', {})
        lin_accel = bno055_data.get('lin_accel_m_s2', {})
        calib = bno055_data.get('calibration', {})

        if not quat:
            return

        # Create IMU message
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        # Orientation (quaternion)
        # Normalize quaternion sign to avoid ambiguity (q and -q are the same rotation)
        # Convention: ensure w >= 0
        qw = quat.get('w', 1.0)
        qx = quat.get('x', 0.0)
        qy = quat.get('y', 0.0)
        qz = quat.get('z', 0.0)

        if qw < 0:
            qw = -qw
            qx = -qx
            qy = -qy
            qz = -qz

        # Remap quaternion to align sensor mounting with base_link frame
        # Frontend orientation cube uses the same mapping:
        # sensor(qx, qy, qz, qw) → robot(-qy, qz, -qx, qw)
        corrected_x = -qy
        corrected_y = qz
        corrected_z = -qx
        corrected_w = qw

        # Normalize corrected quaternion
        norm = math.sqrt(
            corrected_w * corrected_w +
            corrected_x * corrected_x +
            corrected_y * corrected_y +
            corrected_z * corrected_z
        )
        if norm == 0:
            return

        corrected_w /= norm
        corrected_x /= norm
        corrected_y /= norm
        corrected_z /= norm

        # Maintain convention w >= 0 to avoid sign ambiguity
        if corrected_w < 0:
            corrected_w = -corrected_w
            corrected_x = -corrected_x
            corrected_y = -corrected_y
            corrected_z = -corrected_z

        rotated_x, rotated_y, rotated_z, rotated_w = self._rotate_quaternion_z(corrected_x, corrected_y, corrected_z, corrected_w, math.pi)

        imu.orientation.w = rotated_w
        imu.orientation.x = rotated_x
        imu.orientation.y = rotated_y
        imu.orientation.z = rotated_z

        # Angular velocity
        imu.angular_velocity.x = ang_vel.get('x', 0.0) or 0.0
        imu.angular_velocity.y = ang_vel.get('y', 0.0) or 0.0
        imu.angular_velocity.z = ang_vel.get('z', 0.0) or 0.0

        # Linear acceleration
        imu.linear_acceleration.x = lin_accel.get('x', 0.0) or 0.0
        imu.linear_acceleration.y = lin_accel.get('y', 0.0) or 0.0
        imu.linear_acceleration.z = lin_accel.get('z', 0.0) or 0.0

        # Covariance based on calibration
        # Better calibration = lower covariance
        sys_calib = calib.get('sys', 0) or 0
        orientation_cov = 0.1 / max(sys_calib, 1)  # Lower when calibrated
        angular_cov = 0.05
        linear_cov = 0.1

        imu.orientation_covariance = [orientation_cov] * 9
        imu.angular_velocity_covariance = [angular_cov] * 9
        imu.linear_acceleration_covariance = [linear_cov] * 9

        # Publish
        self.imu_pub.publish(imu)

        # Also publish static TF for IMU frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05  # 5cm above base_link
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _rotate_quaternion_z(
        qx: float,
        qy: float,
        qz: float,
        qw: float,
        angle_rad: float,
    ) -> Tuple[float, float, float, float]:
        """Rotate quaternion about Z axis by angle."""
        half = angle_rad / 2.0
        rz_x = 0.0
        rz_y = 0.0
        rz_z = math.sin(half)
        rz_w = math.cos(half)

        # Quaternion multiplication: q_rot * q
        out_x = rz_w * qx + rz_x * qw + rz_y * qz - rz_z * qy
        out_y = rz_w * qy - rz_x * qz + rz_y * qw + rz_z * qx
        out_z = rz_w * qz + rz_x * qy - rz_y * qx + rz_z * qw
        out_w = rz_w * qw - rz_x * qx - rz_y * qy - rz_z * qz

        norm = math.sqrt(out_x * out_x + out_y * out_y + out_z * out_z + out_w * out_w)
        if norm == 0.0:
            return qx, qy, qz, qw
        out_x /= norm
        out_y /= norm
        out_z /= norm
        out_w /= norm
        return out_x, out_y, out_z, out_w


def main(args=None):
    rclpy.init(args=args)
    node = ImuBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
