import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_mission')
    config_file = os.path.join(pkg_share, 'config', 'mission.yaml')

    mock = LaunchConfiguration('mock')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        Node(
            package='billiebot_mission',
            executable='mission_controller.py',
            name='mission_controller',
            parameters=[config_file, {'mock': mock}],
            output='screen',
        ),

        Node(
            package='billiebot_mission',
            executable='approach_dog_server.py',
            name='approach_dog_server',
            parameters=[config_file],
            output='screen',
        ),

        Node(
            package='billiebot_mission',
            executable='retreat_server.py',
            name='retreat_server',
            parameters=[config_file],
            output='screen',
        ),

        Node(
            package='billiebot_mission',
            executable='speak_server.py',
            name='speak_server',
            output='screen',
        ),

        Node(
            package='billiebot_mission',
            executable='dispense_treat_server.py',
            name='dispense_treat_server',
            parameters=[config_file],
            output='screen',
        ),
    ])
