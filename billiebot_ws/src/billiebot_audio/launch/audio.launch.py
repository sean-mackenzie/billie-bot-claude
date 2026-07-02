import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_audio')
    config_file = os.path.join(pkg_share, 'config', 'audio.yaml')

    mock = LaunchConfiguration('mock')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        Node(
            package='billiebot_audio',
            executable='audio_classifier',
            name='audio_classifier',
            parameters=[config_file, {'mock': mock}],
            output='screen',
        ),

        Node(
            package='billiebot_audio',
            executable='speaker_node',
            name='speaker_node',
            parameters=[config_file, {'mock': mock}],
            output='screen',
        ),
    ])
