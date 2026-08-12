"""DT-OAK-01: live dog detection & stereo range against the PRODUCTION oakd_dog_detector
(mock:=false). Enables its bench-only preview/depth-preview/diagnostics outputs (plan
section 5a, all default off elsewhere) -- 07_oakd.launch.py and perception.launch.py are
never touched.

Foxglove is fed by `bench_preview_node`, which compresses the detector's preview streams
bench-side. That deliberately needs no new production-node parameter: the detector already
exposes the three raw preview/diagnostics streams this launch enables, and everything the
remote link sees is derived from them by a bench node. `_TOPICS` (what rosbag2 records and the
rate monitor gates) is unchanged by the preview path.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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

#: Visualization only -- never recorded, never rate-gated, never read by score_oakd_detector.
_PREVIEW_TOPICS = {
    'rgb_preview': '/bench/oakd_detector/rgb/preview/compressed',
    'annotated_preview': '/bench/oakd_detector/annotated/preview/compressed',
    'depth_preview': '/bench/oakd_detector/depth/preview/compressed',
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
    actions += [
        DeclareLaunchArgument(
            'start_visualization_previews', default_value='true',
            description='run bench_preview_node so Foxglove sees compressed previews instead of '
                        'the detector\'s raw preview streams'),
        DeclareLaunchArgument('preview_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('preview_jpeg_quality', default_value='70'),
        DeclareLaunchArgument('preview_format', default_value='jpeg'),
        DeclareLaunchArgument('depth_preview_width', default_value='320'),
        DeclareLaunchArgument('depth_preview_height', default_value='200'),
        DeclareLaunchArgument('depth_preview_min_m', default_value='0.1'),
        DeclareLaunchArgument('depth_preview_max_m', default_value='5.0'),
    ]

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

    # Bench-side compression of the detector's preview streams. The 416x416 sources are already
    # far smaller than UT-OAK's 1080p raw, so this is here for one consistent pattern (raw for
    # recording, light for remote viewing) as much as for the bandwidth it saves. Sizes stay at
    # 416x416 -- there is nothing to gain from decimating a frame that is already small, and
    # keeping native size means the annotated bboxes stay pixel-accurate against the detector's
    # own coordinate space.
    actions.append(Node(
        package='billiebot_sensor_tests',
        executable='bench_preview_node',
        name='bench_preview_node',
        parameters=[{
            'image_sources_json': json.dumps([
                {'in': _TOPICS['preview'], 'out': _PREVIEW_TOPICS['rgb_preview'],
                 'width': 416, 'height': 416},
                {'in': _TOPICS['annotated'], 'out': _PREVIEW_TOPICS['annotated_preview'],
                 'width': 416, 'height': 416},
            ]),
            # /oak/depth/preview is published by the production detector under
            # publish_depth_preview:=true (enabled above) but is deliberately NOT in _TOPICS --
            # it is not recorded and score_oakd_detector never reads it. This makes it visible
            # in Foxglove as a colorized preview, which is all it was ever enabled for.
            'depth_sources_json': json.dumps([
                {'in': '/oak/depth/preview', 'out': _PREVIEW_TOPICS['depth_preview']},
            ]),
            'default_rate_hz': LaunchConfiguration('preview_rate_hz'),
            'default_quality': LaunchConfiguration('preview_jpeg_quality'),
            'default_format': LaunchConfiguration('preview_format'),
            'default_depth_width': LaunchConfiguration('depth_preview_width'),
            'default_depth_height': LaunchConfiguration('depth_preview_height'),
            'default_depth_min_m': LaunchConfiguration('depth_preview_min_m'),
            'default_depth_max_m': LaunchConfiguration('depth_preview_max_m'),
        }],
        condition=IfCondition(LaunchConfiguration('start_visualization_previews')),
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
