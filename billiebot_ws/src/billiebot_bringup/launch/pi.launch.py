"""Raspberry Pi-only nodes: thermal, NoIR, audio, cognition."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    bringup_share = get_package_share_directory('billiebot_bringup')
    cyclone_config = os.path.join(bringup_share, 'config', 'cyclonedds.xml')

    def _include(filename, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', filename)
            ),
            launch_arguments=(args or {}).items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        SetEnvironmentVariable('CYCLONEDDS_URI', f'file://{cyclone_config}'),

        _include('09_thermal.launch.py', {'mock': mock}),
        _include('10_noir.launch.py', {'mock': mock}),
        _include('11_audio.launch.py', {'mock': mock}),
        _include('12_cognition.launch.py'),
    ])
