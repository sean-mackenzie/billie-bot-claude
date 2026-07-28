"""Rung 01: RPLidar A1 driver (or mock /scan publisher)."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')

    # Port and scan settings live in config/lidar.yaml so the lidar follows the same
    # convention as the Arduino (billiebot_base/config/base_driver.yaml): a stable
    # /dev/serial/by-id/ path, not an enumeration-order ttyUSBn index (GAP-20).
    lidar_config = os.path.join(
        get_package_share_directory('billiebot_bringup'), 'config', 'lidar.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        # Real lidar driver
        Node(
            condition=UnlessCondition(mock),
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[lidar_config],
            output='screen',
        ),

        # Mock: publish fake /scan at 10Hz
        Node(
            condition=IfCondition(mock),
            package='billiebot_base',
            executable='mock_scan',
            name='mock_scan',
            parameters=[{'frame_id': 'laser_frame'}],
            output='screen',
        ),
    ])
