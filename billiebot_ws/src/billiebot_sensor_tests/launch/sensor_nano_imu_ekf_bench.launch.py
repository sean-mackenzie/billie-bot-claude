"""UT-IMU-02: /imu/data ROS contract and robot_localization compatibility (120 s default).

Adds, on top of the UT-IMU-01 graph:
  * `robot_localization`'s ekf_node using config/ekf_imu_bench.yaml -- a BENCH-ONLY file.
    Production billiebot_navigation/config/ekf.yaml is not touched: its imu0 block is still
    commented out (BLK-04), and enabling IMU there just to pass a bench test would change
    robot behaviour outside this campaign.
  * a static base_link -> imu_link transform with identity rotation, which is correct for
    the flat bench fixture. The installed robot transform is still unmeasured (BLK-12);
    override `imu_xyz` / `imu_rpy` once it is.
  * /rosout in the recorded set, so "no sustained TF/timeout failures" is scored from the
    bag rather than from console.log (which holds only preflight output unless ROS_LOG_DIR
    is redirected).

This is a compatibility test, not an EKF-accuracy qualification.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from billiebot_sensor_tests.common.launch_helpers import (
    declare_common_bench_args,
    duration_shutdown_action,
    finalize_on_shutdown_action,
    foxglove_bridge_action,
    manifest_bootstrap_action,
    record_bag_action,
)
from billiebot_sensor_tests.sensor_nano.launch_common import (
    declare_sensor_nano_args,
    ground_truth_marker_action,
    sensor_nano_bridge_action,
)

_TOPICS = {
    'imu': '/imu/data',
    'mag': '/imu/mag',
    'pressure': '/barometer/pressure',
    'temperature': '/barometer/temperature',
    'diagnostics': '/bench/sensor_nano/diagnostics',
    'filtered_odometry': '/odometry/filtered',
    'rosout': '/rosout',
}


def _static_transform_action(context, *args, **kwargs):
    """base_link -> imu_link. Split out so the xyz/rpy launch arguments can be expanded into
    static_transform_publisher's positional argument list."""
    xyz = LaunchConfiguration('imu_xyz').perform(context).split()
    rpy = LaunchConfiguration('imu_rpy').perform(context).split()
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError(
            f'imu_xyz and imu_rpy must each be three space-separated numbers, '
            f'got imu_xyz={xyz!r} imu_rpy={rpy!r}'
        )
    return [Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='bench_base_link_to_imu_link',
        arguments=['--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                    '--roll', rpy[0], '--pitch', rpy[1], '--yaw', rpy[2],
                    '--frame-id', 'base_link', '--child-frame-id',
                    LaunchConfiguration('imu_frame_id_resolved').perform(context)],
        output='screen',
    )]


def generate_launch_description():
    share = get_package_share_directory('billiebot_sensor_tests')
    ekf_config = os.path.join(share, 'config', 'ekf_imu_bench.yaml')

    rate_topics_config = json.dumps([
        {'name': _TOPICS['imu'], 'type': 'sensor_msgs/msg/Imu', 'required': True,
         'min_rate_hz': 45.0},
        {'name': _TOPICS['filtered_odometry'], 'type': 'nav_msgs/msg/Odometry',
         'required': True, 'min_rate_hz': 27.0},
    ])

    actions = declare_common_bench_args(default_duration_sec='120')
    actions += declare_sensor_nano_args()
    actions += [
        DeclareLaunchArgument(
            'ekf_config_file', default_value=ekf_config,
            description='bench-only robot_localization parameters; NOT the production ekf.yaml'),
        DeclareLaunchArgument(
            'imu_xyz', default_value='0.0 0.0 0.0',
            description='base_link -> imu_link translation, three space-separated metres'),
        DeclareLaunchArgument(
            'imu_rpy', default_value='0.0 0.0 0.0',
            description='base_link -> imu_link rotation, three space-separated radians; '
                        'identity is correct for the flat bench fixture (BLK-12)'),
        DeclareLaunchArgument(
            'imu_frame_id_resolved', default_value='imu_link',
            description='child frame of the bench static transform; keep it equal to the '
                        'bridge imu_frame_id'),
    ]

    actions.append(manifest_bootstrap_action(
        'UT-IMU-02', 'sensor_nano', 'DFRobot SEN0253 (BNO055 + BMP280) on Arduino Nano V3',
        preflight_context_args=('sensor_port',),
    ))

    actions.append(sensor_nano_bridge_action())
    actions.append(OpaqueFunction(function=_static_transform_action))

    actions.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[LaunchConfiguration('ekf_config_file')],
        output='screen',
    ))

    actions.append(Node(
        package='billiebot_sensor_tests',
        executable='topic_rate_monitor',
        name='topic_rate_monitor',
        parameters=[{
            'topics_config_json': rate_topics_config,
            'output_path': PathJoinSubstitution(
                [LaunchConfiguration('results_dir'), 'exports', 'topic_rate_monitor.json']
            ),
        }],
        output='screen',
    ))

    actions.append(ground_truth_marker_action())

    actions += record_bag_action(list(_TOPICS.values()))
    actions.append(foxglove_bridge_action())
    actions += duration_shutdown_action()
    actions.append(finalize_on_shutdown_action())

    return LaunchDescription(actions)
