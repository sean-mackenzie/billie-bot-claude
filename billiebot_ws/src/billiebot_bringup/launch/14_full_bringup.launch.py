"""Rung 14: Full BillieBot bringup — everything."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    map_file = LaunchConfiguration('map')
    bringup_share = get_package_share_directory('billiebot_bringup')

    def _include(filename, args=None):
        path = os.path.join(bringup_share, 'launch', filename)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(path),
            launch_arguments=(args or {}).items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),

        # Navigation stack
        _include('06_nav2.launch.py', {'mock': mock, 'map': map_file}),

        # Perception
        _include('07_oakd.launch.py', {'mock': mock}),
        _include('08_dog_locator.launch.py'),
        _include('09_thermal.launch.py', {'mock': mock}),
        _include('10_noir.launch.py', {'mock': mock}),

        # Audio
        _include('11_audio.launch.py', {'mock': mock}),

        # Cognition
        _include('12_cognition.launch.py'),

        # Mission
        _include('13_mission.launch.py', {'mock': mock}),
    ])
