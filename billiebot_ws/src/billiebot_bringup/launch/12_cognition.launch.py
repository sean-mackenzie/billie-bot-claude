"""Rung 12: Cognition (state fusion, logger, report, server)."""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cog_share = get_package_share_directory('billiebot_cognition')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cog_share, 'launch', 'cognition.launch.py')
            ),
        ),
    ])
