"""Rung 11: Audio classifier + speaker."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    audio_share = get_package_share_directory('billiebot_audio')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(audio_share, 'launch', 'audio.launch.py')
            ),
            launch_arguments={'mock': mock}.items(),
        ),
    ])
