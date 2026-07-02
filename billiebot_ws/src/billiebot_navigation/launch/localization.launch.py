import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_navigation')
    amcl_config = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='',
                              description='Full path to map yaml file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                nav2_config,
                {'yaml_filename': map_file},
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[
                amcl_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ])
