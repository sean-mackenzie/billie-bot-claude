"""UT-BAT-01: battery-divider acquisition and voltage accuracy.

Operator-paced. `duration_sec` defaults to '0', which `duration_shutdown_action()` already
treats as "no automatic shutdown timer" -- the launch keeps streaming until the operator
stops it with Ctrl-C, which is what a PSU sweep with a DMM in hand actually needs. Pass a
non-zero `duration_sec` if a fixed-length capture is wanted instead.

While this is running, record each PSU setpoint from a SECOND terminal:

    ros2 run billiebot_sensor_tests record_battery_point \\
      --results-dir "$RESULTS" --setpoint-v 10.50 \\
      --dmm-battery-v 10.497 --dmm-a0-v 1.749

The IMU keeps streaming (the firmware always sends it) but is not recorded here: this test
is about the battery path, and 50 Hz of IMU data would dominate the bag for no benefit.
"""

import json

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
)
from billiebot_sensor_tests.sensor_nano.launch_common import (
    declare_sensor_nano_args,
    ground_truth_marker_action,
    sensor_nano_bridge_action,
)

_TOPICS = {
    'battery': '/battery_state',
    'battery_adc': '/bench/battery/adc',
    'diagnostics': '/bench/sensor_nano/diagnostics',
}


def generate_launch_description():
    rate_topics_config = json.dumps([
        {'name': _TOPICS['battery'], 'type': 'sensor_msgs/msg/BatteryState',
         'required': True, 'min_rate_hz': 4.0},
        {'name': _TOPICS['battery_adc'], 'type': 'std_msgs/msg/Float32',
         'required': True, 'min_rate_hz': 4.0},
        {'name': _TOPICS['diagnostics'], 'type': 'diagnostic_msgs/msg/DiagnosticArray',
         'required': True},
    ])

    actions = declare_common_bench_args(default_duration_sec='0')
    actions += declare_sensor_nano_args()

    actions.append(manifest_bootstrap_action(
        'UT-BAT-01', 'sensor_nano', 'DFRobot SEN0253 + battery divider on Arduino Nano V3',
        preflight_context_args=('sensor_port',),
    ))

    actions.append(sensor_nano_bridge_action())

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

    # Lets the operator annotate the sweep ("mark setpoint_10v50") alongside the
    # record_battery_point rows, which is useful when a point has to be retaken.
    actions.append(ground_truth_marker_action())

    actions += record_bag_action(list(_TOPICS.values()))
    actions.append(foxglove_bridge_action())
    actions += duration_shutdown_action()
    actions.append(finalize_on_shutdown_action())

    return LaunchDescription(actions)
