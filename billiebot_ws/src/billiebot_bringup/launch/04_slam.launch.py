"""Rung 04: SLAM (slam_toolbox) on top of base + lidar + EKF."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    nav_share = get_package_share_directory('billiebot_navigation')
    bringup_share = get_package_share_directory('billiebot_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', '01_lidar.launch.py')
            ),
            launch_arguments={'mock': mock}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', '03_ekf.launch.py')
            ),
            launch_arguments={'mock': mock}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_share, 'launch', 'slam.launch.py')
            ),
        ),
    ])
