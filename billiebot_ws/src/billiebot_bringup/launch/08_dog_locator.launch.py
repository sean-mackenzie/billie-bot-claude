"""Rung 08: Dog locator (camera->map TF transform)."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='billiebot_perception',
            executable='dog_locator',
            name='dog_locator',
            output='screen',
        ),
    ])
