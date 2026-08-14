"""UT-IMU-01: Sensor Nano IMU/barometer acquisition (180 s default).

Starts the Sensor Nano bridge, the rate monitor, the ground-truth marker and rosbag2. The
Motor Nano, base_bridge, motors, encoders and Nav2 are all deliberately absent -- the Sensor
Nano is the only Arduino this test requires (test plan section 4.1).

`_TOPICS` is the authoritative recorded set: it is what rosbag2 captures and what
analyze_sensor_nano_imu reads. Battery topics are recorded too, because the firmware streams
them unconditionally and their presence in the bag costs almost nothing while making an
IMU-run bag self-contained if a battery question comes up later.

The operator must mark each hold with `mark <label>` in this launch's terminal (see
sensor_nano.ut_imu_01_rotation_sequence in sensor_bench.yaml for the expected labels);
without those marks the commanded-rotation gate has no evidence and fails.
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
    'imu': '/imu/data',
    'mag': '/imu/mag',
    'pressure': '/barometer/pressure',
    'temperature': '/barometer/temperature',
    'battery': '/battery_state',
    'battery_adc': '/bench/battery/adc',
    'diagnostics': '/bench/sensor_nano/diagnostics',
}


def generate_launch_description():
    rate_topics_config = json.dumps([
        {'name': _TOPICS['imu'], 'type': 'sensor_msgs/msg/Imu', 'required': True,
         'min_rate_hz': 45.0},
        {'name': _TOPICS['pressure'], 'type': 'sensor_msgs/msg/FluidPressure',
         'required': True, 'min_rate_hz': 1.5},
        {'name': _TOPICS['diagnostics'], 'type': 'diagnostic_msgs/msg/DiagnosticArray',
         'required': True},
    ])

    actions = declare_common_bench_args(default_duration_sec='180')
    actions += declare_sensor_nano_args()

    actions.append(manifest_bootstrap_action(
        'UT-IMU-01', 'sensor_nano', 'DFRobot SEN0253 (BNO055 + BMP280) on Arduino Nano V3',
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

    actions.append(ground_truth_marker_action())

    actions += record_bag_action(list(_TOPICS.values()))
    actions.append(foxglove_bridge_action())
    actions += duration_shutdown_action()
    actions.append(finalize_on_shutdown_action())

    return LaunchDescription(actions)
