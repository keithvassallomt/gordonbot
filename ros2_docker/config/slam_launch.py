#!/usr/bin/env python3
"""
Launch file for GordonBot SLAM system.
Starts all bridge nodes and slam_toolbox.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Get backend URL from environment or use default
    # Since we use network_mode: host in docker-compose, localhost works
    backend_http_url = os.environ.get('BACKEND_URL', 'http://localhost:8000')
    backend_ws_url = os.environ.get('BACKEND_WS_URL', 'ws://localhost:8000')

    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # LIDAR bridge node
        Node(
            package='bridge_nodes',
            executable='lidar_bridge',
            name='lidar_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'backend_url': backend_ws_url,
                'frame_id': 'laser'
            }]
        ),

        # Odometry bridge node
        Node(
            package='bridge_nodes',
            executable='odom_bridge',
            name='odom_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'backend_url': backend_http_url,
                'poll_rate_hz': 20.0,
                'wheel_base_m': 0.085,
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }]
        ),

        # IMU bridge node
        Node(
            package='bridge_nodes',
            executable='imu_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'backend_url': backend_http_url,
                'poll_rate_hz': 20.0,  # Reduced from 50Hz to match backend cache update rate
                'frame_id': 'imu_link',
                'base_frame': 'base_link'
            }]
        ),

        # EKF node - fuses wheel odometry (position) with IMU (orientation)
        # Output: /odom_fused topic with accurate pose combining both sources
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/config/ekf_params.yaml',
                       {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odom_fused')]
        ),

        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),

        # SLAM Toolbox (async mode)
        # Now uses /odom_fused from EKF instead of raw /odom
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/config/slam_toolbox_params.yaml',
                {'use_sim_time': use_sim_time}
            ],
            remappings=[('odom', 'odom_fused')]
        ),

        # Map bridge node (serves map via WebSocket)
        Node(
            package='bridge_nodes',
            executable='map_bridge',
            name='map_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'ws_port': 9001,
                'map_topic': '/map',
                'pose_rate_hz': 10.0,
                'map_frame': 'map',
                'base_frame': 'base_link'
            }]
        ),
    ])
