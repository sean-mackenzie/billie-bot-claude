"""UT-OAK-01 (RGB/stereo-depth/point-cloud stream test) and UT-OAK-02 (2.0 m flat-target
depth accuracy, via test_mode:=flat_target) share this launch file.

Starts a standalone OAK-D acquisition node (oakd_bench_publisher) -- Type 1 tests are
acquisition-only and never start the production oakd_dog_detector -- plus the rate
monitor, rosbag2 recording, and (optionally) foxglove_bridge.

`_TOPICS` below is the *authoritative* topic set: it is what rosbag2 records, what the rate
monitor gates on, and what the analysis CLIs read. `start_visualization_previews` adds a
separate low-bandwidth `/bench/oakd/.../preview` set for Foxglove only -- those topics are
deliberately absent from `_TOPICS`, so enabling or disabling previews cannot change what is
recorded, what is measured, or the pass/fail verdict.
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

#: Visualization only -- never recorded, never rate-gated, never read by analysis.
_PREVIEW_TOPICS = {
    'rgb_preview': '/bench/oakd/rgb/preview/compressed',
    'depth_preview': '/bench/oakd/depth/preview/compressed',
    'points_preview': '/bench/oakd/points_preview',
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
        DeclareLaunchArgument(
            'start_visualization_previews', default_value='true',
            description='publish low-bandwidth Foxglove preview topics alongside the raw ones; '
                        'set false to leave the graph exactly as it was before previews existed'),
        DeclareLaunchArgument('preview_width', default_value='640'),
        DeclareLaunchArgument('preview_height', default_value='360'),
        DeclareLaunchArgument('preview_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('preview_jpeg_quality', default_value='70'),
        DeclareLaunchArgument('preview_format', default_value='jpeg',
                               description="'jpeg', 'png', or 'raw' (uncompressed, downsampled)"),
        DeclareLaunchArgument('depth_preview_width', default_value='320'),
        DeclareLaunchArgument('depth_preview_height', default_value='200'),
        DeclareLaunchArgument('depth_preview_min_m', default_value='0.1'),
        DeclareLaunchArgument('depth_preview_max_m', default_value='5.0'),
        DeclareLaunchArgument('points_preview_stride', default_value='16'),
        DeclareLaunchArgument('points_preview_rate_hz', default_value='2.0'),
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
            # Visualization knobs only -- the raw RGB/depth/points publishes above them are
            # untouched by every one of these.
            'publish_previews': LaunchConfiguration('start_visualization_previews'),
            'preview_width': LaunchConfiguration('preview_width'),
            'preview_height': LaunchConfiguration('preview_height'),
            'preview_rate_hz': LaunchConfiguration('preview_rate_hz'),
            'preview_jpeg_quality': LaunchConfiguration('preview_jpeg_quality'),
            'preview_format': LaunchConfiguration('preview_format'),
            'depth_preview_width': LaunchConfiguration('depth_preview_width'),
            'depth_preview_height': LaunchConfiguration('depth_preview_height'),
            'depth_preview_min_m': LaunchConfiguration('depth_preview_min_m'),
            'depth_preview_max_m': LaunchConfiguration('depth_preview_max_m'),
            'points_preview_stride': LaunchConfiguration('points_preview_stride'),
            'points_preview_rate_hz': LaunchConfiguration('points_preview_rate_hz'),
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
