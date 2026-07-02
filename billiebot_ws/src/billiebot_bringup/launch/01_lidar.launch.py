"""Rung 01: RPLidar A1 driver (or mock /scan publisher)."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mock = LaunchConfiguration('mock')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        # Real lidar driver
        Node(
            condition=UnlessCondition(mock),
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB1',
                'serial_baudrate': 115200,
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
            output='screen',
        ),

        # Mock: publish fake /scan at 5Hz
        Node(
            condition=IfCondition(mock),
            package='billiebot_base',
            executable='base_bridge',
            name='mock_lidar_stub',
            parameters=[{'mock': True}],
            output='screen',
            # In practice, a dedicated mock scan publisher would be better.
            # For now, this is a placeholder.
        ),
    ])
