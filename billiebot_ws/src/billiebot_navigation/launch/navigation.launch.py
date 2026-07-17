"""Nav2 stack only (controller, planner, behaviors, bt_navigator, lifecycle).

Does NOT start an EKF: it expects `odom→base_link` TF and `/odometry/filtered`
from an already-running `ekf_filter_node` — the ladder's `03_ekf.launch.py` is
the canonical owner (GAP-6). If you launch this file standalone, bring your own
EKF (e.g. `ros2 launch billiebot_bringup 03_ekf.launch.py`).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_navigation')
    nav2_config = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

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
