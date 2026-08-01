"""DT-THM-01: Billie warm-body blob detection against the PRODUCTION thermal_node
(mock:=false), UNMODIFIED -- plan section 5b, no production change for thermal.
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
    'image': '/thermal/image',
    'blob': '/thermal/blob',
    'image_color': '/bench/thermal/image_color',
}


def generate_launch_description():
    perception_share = get_package_share_directory('billiebot_perception')
    production_config = os.path.join(perception_share, 'config', 'perception.yaml')

    rate_topics_config = json.dumps([
        {'name': _TOPICS['image'], 'type': 'sensor_msgs/msg/Image', 'required': True,
         'min_rate_hz': 3.6},
        {'name': _TOPICS['blob'], 'type': 'billiebot_interfaces/msg/ThermalBlob',
         'required': False},
    ])

    actions = declare_common_bench_args(default_duration_sec='0')
    actions.append(manifest_bootstrap_action('DT-THM-01', 'thermal', 'MLX90640'))

    actions.append(replicate_production_node(
        'billiebot_perception', 'thermal_node', production_config, {'mock': False},
    ))

    actions.append(Node(
        package='billiebot_sensor_tests',
        executable='thermal_colorizer',
        name='thermal_colorizer',
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
