"""Rung 09: MLX90640 thermal camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mock = LaunchConfiguration('mock')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        Node(
            package='billiebot_perception',
            executable='thermal_node',
            name='thermal_node',
            parameters=[{'mock': mock}],
            output='screen',
        ),
    ])
