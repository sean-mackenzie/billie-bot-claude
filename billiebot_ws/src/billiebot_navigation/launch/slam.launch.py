import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_navigation')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            output='screen',
        ),
    ])
