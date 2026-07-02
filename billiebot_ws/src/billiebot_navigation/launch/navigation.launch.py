import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_navigation')
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                ekf_config,
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),

        # Nav2 controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Nav2 planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Nav2 behaviors (spin, backup, wait — SYS-NAV-4)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Nav2 BT navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_config, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
    ])
