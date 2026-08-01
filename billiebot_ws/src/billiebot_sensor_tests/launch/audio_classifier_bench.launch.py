"""DT-AUD-01 (bark/speech/other-sound classification) and DT-AUD-02 (XVF3800 DoA,
optional) share this launch file -- run it twice, once per test, each with its own
--results-dir (see score_audio_classifier.py's module docstring for why).

Starts the PRODUCTION audio_classifier (mock:=false) with continuous_capture left at its
new default (true) and publish_status:=true, plus the ground-truth marker, rate monitor
(the required topic is the processing-cadence /bench/audio_classifier/status, NOT the
sparse /audio/events), rosbag2 recording, and foxglove_bridge.
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
    'events': '/audio/events',
    'status': '/bench/audio_classifier/status',
}


def generate_launch_description():
    audio_share = get_package_share_directory('billiebot_audio')
    production_config = os.path.join(audio_share, 'config', 'audio.yaml')

    rate_topics_config = json.dumps([
        {'name': _TOPICS['status'], 'type': 'diagnostic_msgs/msg/DiagnosticArray',
         'required': True, 'min_rate_hz': 1.8},
        # /audio/events is deliberately NOT required here -- it is event-gated/sparse by
        # design, not a continuous stream (see plan's audio processing-rate discussion).
        {'name': _TOPICS['events'], 'type': 'billiebot_interfaces/msg/AudioEvent',
         'required': False},
    ])

    actions = declare_common_bench_args(default_duration_sec='0')
    actions += [
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('device_name_substring', default_value='XVF3800'),
        DeclareLaunchArgument('continuous_capture', default_value='true'),
    ]

    actions.append(manifest_bootstrap_action('DT-AUD-01', 'audio', 'reSpeaker XVF3800'))

    actions.append(replicate_production_node(
        'billiebot_audio', 'audio_classifier', production_config,
        {
            'mock': False,
            'model_path': LaunchConfiguration('model_path'),
            'device_name_substring': LaunchConfiguration('device_name_substring'),
            'continuous_capture': LaunchConfiguration('continuous_capture'),
            'publish_status': True,
        },
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

    actions.append(Node(
        package='billiebot_sensor_tests',
        executable='ground_truth_marker_node',
        name='ground_truth_marker_node',
        parameters=[{
            'output_csv': PathJoinSubstitution(
                [LaunchConfiguration('results_dir'), 'exports', 'ground_truth_segments.csv']
            ),
        }],
        output='screen',
    ))

    actions += record_bag_action(list(_TOPICS.values()))
    actions.append(foxglove_bridge_action())
    actions += duration_shutdown_action()
    actions.append(finalize_on_shutdown_action())

    return LaunchDescription(actions)
