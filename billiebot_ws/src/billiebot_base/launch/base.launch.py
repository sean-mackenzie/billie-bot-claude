import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_base')
    config_file = os.path.join(pkg_share, 'config', 'base_driver.yaml')

    mock = LaunchConfiguration('mock')

    return LaunchDescription([
        DeclareLaunchArgument('mock', default_value='false'),

        Node(
            package='billiebot_base',
            executable='base_bridge',
            name='base_bridge',
            parameters=[
                config_file,
                {'mock': mock},
            ],
            output='screen',
        ),
    ])
