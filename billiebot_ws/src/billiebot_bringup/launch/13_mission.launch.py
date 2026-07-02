"""Rung 13: Mission controller + action servers."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    mission_share = get_package_share_directory('billiebot_mission')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mission_share, 'launch', 'mission.launch.py')
            ),
            launch_arguments={'mock': mock}.items(),
        ),
    ])
