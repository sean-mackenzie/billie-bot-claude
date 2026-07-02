"""Jetson-only nodes: base, lidar, nav2, OAK-D, mission."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    map_file = LaunchConfiguration('map')
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
        DeclareLaunchArgument('map', default_value=''),

        SetEnvironmentVariable('CYCLONEDDS_URI', f'file://{cyclone_config}'),

        _include('06_nav2.launch.py', {'mock': mock, 'map': map_file}),
        _include('07_oakd.launch.py', {'mock': mock}),
        _include('08_dog_locator.launch.py'),
        _include('13_mission.launch.py', {'mock': mock}),
    ])
