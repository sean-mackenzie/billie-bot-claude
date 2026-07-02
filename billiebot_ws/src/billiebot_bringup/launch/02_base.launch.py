"""Rung 02: Base bridge + robot_state_publisher."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')

    base_launch = os.path.join(
        get_package_share_directory('billiebot_base'), 'launch', 'base.launch.py'
    )
    desc_launch = os.path.join(
        get_package_share_directory('billiebot_description'), 'launch', 'description.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(desc_launch),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={'mock': mock}.items(),
        ),
    ])
