"""Rung 10: Pi Camera 3 NoIR."""

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
            executable='noir_cam_node',
            name='noir_cam_node',
            parameters=[{'mock': mock}],
            output='screen',
        ),
    ])
