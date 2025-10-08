from setuptools import find_packages, setup

package_name = 'bridge_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Keith',
    maintainer_email='keith@gordonbot.local',
    description='ROS2 bridge nodes for GordonBot sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_bridge = bridge_nodes.lidar_bridge:main',
            'odom_bridge = bridge_nodes.odom_bridge:main',
            'imu_bridge = bridge_nodes.imu_bridge:main',
            'imu_odom_bridge = bridge_nodes.imu_odom_bridge:main',
            'map_bridge = bridge_nodes.map_bridge:main',
        ],
    },
)
