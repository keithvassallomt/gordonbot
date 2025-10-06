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
    backend_url = os.environ.get('BACKEND_URL', 'http://host.docker.internal:8000')

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
                'backend_url': backend_url,
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
                'backend_url': backend_url,
                'poll_rate_hz': 20.0,
                'wheel_base_m': 0.14,
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
                'backend_url': backend_url,
                'poll_rate_hz': 50.0,
                'frame_id': 'imu_link',
                'base_frame': 'base_link'
            }]
        ),

        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),

        # SLAM Toolbox (async mode)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/config/slam_toolbox_params.yaml',
                {'use_sim_time': use_sim_time}
            ]
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
                'pose_topic': '/pose'
            }]
        ),
    ])
