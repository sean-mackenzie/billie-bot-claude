"""UT-OAK-01 (RGB/stereo-depth/point-cloud stream test) and UT-OAK-02 (2.0 m flat-target
depth accuracy, via test_mode:=flat_target) share this launch file.

Starts a standalone OAK-D acquisition node (oakd_bench_publisher) -- Type 1 tests are
acquisition-only and never start the production oakd_dog_detector -- plus the rate
monitor, rosbag2 recording, and (optionally) foxglove_bridge.
"""

import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

from billiebot_sensor_tests.common.launch_helpers import (
    declare_common_bench_args,
    duration_shutdown_action,
    finalize_on_shutdown_action,
    foxglove_bridge_action,
    manifest_bootstrap_action,
    record_bag_action,
)

_TOPICS = {
    'rgb': '/bench/oakd/rgb/image_raw',
    'rgb_info': '/bench/oakd/rgb/camera_info',
    'depth': '/bench/oakd/depth/image_raw',
    'depth_info': '/bench/oakd/depth/camera_info',
    'points': '/bench/oakd/points',
    'diagnostics': '/bench/oakd/diagnostics',
}


def generate_launch_description():
    test_id = PythonExpression([
        "'UT-OAK-02' if '", LaunchConfiguration('test_mode'), "' == 'flat_target' else 'UT-OAK-01'"
    ])

    rate_topics_config = json.dumps([
        {'name': _TOPICS['rgb'], 'type': 'sensor_msgs/msg/Image', 'required': True,
         'min_rate_hz': 4.5, 'hash_enabled': True},
        {'name': _TOPICS['depth'], 'type': 'sensor_msgs/msg/Image', 'required': True,
         'min_rate_hz': 4.5, 'hash_enabled': False},
        {'name': _TOPICS['points'], 'type': 'sensor_msgs/msg/PointCloud2', 'required': True},
    ])

    actions = declare_common_bench_args(default_duration_sec='60')
    actions += [
        DeclareLaunchArgument('test_mode', default_value=''),
        DeclareLaunchArgument('mock', default_value='false'),
    ]

    actions.append(manifest_bootstrap_action(test_id, 'oakd', 'OAK-D Lite'))

    actions.append(Node(
        package='billiebot_sensor_tests',
        executable='oakd_bench_publisher',
        name='oakd_bench_publisher',
        parameters=[{
            'mock': LaunchConfiguration('mock'),
            'sensor_serial': LaunchConfiguration('sensor_serial'),
            'test_mode': LaunchConfiguration('test_mode'),
            'fail_on_missing_device': LaunchConfiguration('fail_on_missing_device'),
        }],
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

    actions += record_bag_action(list(_TOPICS.values()))
    actions.append(foxglove_bridge_action())
    actions += duration_shutdown_action()
    actions.append(finalize_on_shutdown_action())

    return LaunchDescription(actions)
