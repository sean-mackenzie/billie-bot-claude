"""Rung 06: Full Nav2 stack on top of AMCL."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    map_file = LaunchConfiguration('map')
    nav_share = get_package_share_directory('billiebot_navigation')
    bringup_share = get_package_share_directory('billiebot_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', '05_amcl.launch.py')
            ),
            launch_arguments={'mock': mock, 'map': map_file}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'navigation.launch.py')
            ),
        ),
    ])
