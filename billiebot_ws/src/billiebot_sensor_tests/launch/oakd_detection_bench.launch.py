"""DT-OAK-01: live dog detection & stereo range against the PRODUCTION oakd_dog_detector
(mock:=false). Enables its bench-only preview/depth-preview/diagnostics outputs (plan
section 5a, all default off elsewhere) -- 07_oakd.launch.py and perception.launch.py are
never touched.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
    'detections': '/dog/detections_3d',
    'found': '/dog/found',
    'preview': '/oak/rgb/preview',
    'annotated': '/oak/rgb/annotated',
    'diagnostics': '/bench/oakd_detector/diagnostics',
}


def generate_launch_description():
    perception_share = get_package_share_directory('billiebot_perception')
    production_config = os.path.join(perception_share, 'config', 'perception.yaml')

    rate_topics_config = json.dumps([
        {'name': _TOPICS['found'], 'type': 'std_msgs/msg/Bool', 'required': True,
         'min_rate_hz': 4.5},
        {'name': _TOPICS['detections'], 'type': 'billiebot_interfaces/msg/DogDetection3D',
         'required': False},
        {'name': _TOPICS['preview'], 'type': 'sensor_msgs/msg/Image', 'required': False,
         'hash_enabled': True},
    ])

    # Type 2 launches default duration_sec:=0 (run until the operator stops them -- one
    # session covers many discrete distance/pose/lighting segments), but the operator can
    # still pass a nonzero duration_sec if a bounded run is wanted.
    actions = declare_common_bench_args(default_duration_sec='0')

    actions.append(manifest_bootstrap_action('DT-OAK-01', 'oakd', 'OAK-D Lite'))

    actions.append(replicate_production_node(
        'billiebot_perception', 'oakd_dog_detector', production_config,
        {
            'mock': False,
            'publish_preview': True,
            'publish_depth_preview': True,
            'publish_diagnostics': True,
        },
    ))

    actions.append(Node(
        package='billiebot_sensor_tests',
        executable='oakd_preview_overlay',
        name='oakd_preview_overlay',
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
