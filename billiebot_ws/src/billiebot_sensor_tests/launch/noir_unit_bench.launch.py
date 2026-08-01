"""UT-NIR-01 (stream test) and UT-NIR-02 (focus/quality/low-light -- run 4x separately,
one per lighting condition, each with its own results_dir and duration_sec:=15) share
this launch file.

Starts the production noir_cam_node (mock:=false) with optional bench-only AF/exposure/
gain/metadata overrides (plan section 5c, all sentinel-default so behavior is unchanged
unless explicitly set), the online image-quality monitor, rate monitor (with repeated-
frame hashing enabled), rosbag2 recording, and foxglove_bridge.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from billiebot_sensor_tests.common.launch_helpers import (
    declare_common_bench_args,
    duration_shutdown_action,
    finalize_on_shutdown_action,
    foxglove_bridge_action,
    manifest_bootstrap_action,
    record_bag_action,
    replicate_production_node,
)

_TOPICS = {
    'image': '/noir/image',
    'diagnostics': '/bench/noir/diagnostics',
}


def generate_launch_description():
    perception_share = get_package_share_directory('billiebot_perception')
    production_config = os.path.join(perception_share, 'config', 'perception.yaml')

    rate_topics_config = json.dumps([
        {'name': _TOPICS['image'], 'type': 'sensor_msgs/msg/Image', 'required': True,
         'min_rate_hz': 4.5, 'hash_enabled': True},
    ])

    actions = declare_common_bench_args(default_duration_sec='60')
    actions += [
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('af_mode', default_value=''),
        DeclareLaunchArgument('af_trigger', default_value='false'),
        DeclareLaunchArgument('lens_position', default_value='-1.0'),
        DeclareLaunchArgument('exposure_time_us', default_value='-1'),
        DeclareLaunchArgument('analogue_gain', default_value='-1.0'),
        DeclareLaunchArgument('frame_duration_us', default_value='-1'),
        DeclareLaunchArgument('publish_metadata', default_value='false'),
    ]

    actions.append(manifest_bootstrap_action('UT-NIR-01', 'noir', 'Pi Camera 3 NoIR'))

    actions.append(replicate_production_node(
        'billiebot_perception', 'noir_cam_node', production_config,
        {
            'mock': LaunchConfiguration('mock'),
            'af_mode': LaunchConfiguration('af_mode'),
            'af_trigger': LaunchConfiguration('af_trigger'),
            'lens_position': LaunchConfiguration('lens_position'),
            'exposure_time_us': LaunchConfiguration('exposure_time_us'),
            'analogue_gain': LaunchConfiguration('analogue_gain'),
            'frame_duration_us': LaunchConfiguration('frame_duration_us'),
            'publish_metadata': LaunchConfiguration('publish_metadata'),
        },
    ))

    actions.append(Node(
        package='billiebot_sensor_tests',
        executable='image_quality_monitor',
        name='image_quality_monitor',
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
