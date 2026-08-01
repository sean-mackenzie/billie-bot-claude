"""Rung 07: OAK-D Lite dog detector."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_perception')
    config_file = os.path.join(pkg_share, 'config', 'perception.yaml')

    mock = LaunchConfiguration('mock')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        Node(
            package='billiebot_perception',
            executable='oakd_dog_detector',
            name='oakd_dog_detector',
            parameters=[config_file, {'mock': mock}],
            output='screen',
        ),
    ])
