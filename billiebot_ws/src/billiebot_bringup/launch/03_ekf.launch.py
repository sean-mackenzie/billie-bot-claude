"""Rung 03: EKF (robot_localization) on top of base + lidar."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mock = LaunchConfiguration('mock')
    nav_share = get_package_share_directory('billiebot_navigation')
    ekf_config = os.path.join(nav_share, 'config', 'ekf.yaml')

    bringup_share = get_package_share_directory('billiebot_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        # Include rung 02 (base + description)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', '02_base.launch.py')
            ),
            launch_arguments={'mock': mock}.items(),
        ),

        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen',
        ),
    ])
