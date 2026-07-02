"""Rung 07: OAK-D Lite dog detector."""

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
            executable='oakd_dog_detector',
            name='oakd_dog_detector',
            parameters=[{'mock': mock}],
            output='screen',
        ),
    ])
