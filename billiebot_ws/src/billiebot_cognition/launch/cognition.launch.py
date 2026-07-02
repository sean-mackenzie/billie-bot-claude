import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_cognition')
    config_file = os.path.join(pkg_share, 'config', 'cognition.yaml')
    rooms_file = os.path.join(pkg_share, 'config', 'rooms.yaml')

    return LaunchDescription([
        Node(
            package='billiebot_cognition',
            executable='state_fusion',
            name='state_fusion',
            parameters=[config_file],
            output='screen',
        ),

        Node(
            package='billiebot_cognition',
            executable='dog_logger',
            name='dog_logger',
            parameters=[config_file, {'rooms_config': rooms_file}],
            output='screen',
        ),

        Node(
            package='billiebot_cognition',
            executable='daily_report',
            name='daily_report',
            parameters=[config_file],
            output='screen',
        ),

        Node(
            package='billiebot_cognition',
            executable='report_server',
            name='report_server',
            parameters=[config_file],
            output='screen',
        ),
    ])
