"""UT-BAT-02B: exact 10.5 V SAFE boundary check against the production mission controller.

SOFTWARE ONLY. No Sensor Nano, no divider, no ADC, no PSU -- `battery_threshold_test`
publishes synthetic sensor_msgs/BatteryState directly. Analog uncertainty and 10-bit ADC
quantization make an exact 10.500 V boundary untestable through real hardware, which is why
this is separated from UT-BAT-02 (test plan section 18.1).

Expected result today: 10.5001 V passes, 10.4999 V passes, and **10.5000 V FAILS**, because
`mission_controller.py:147` compares with a strict `<` while SYS-PLT-2 requires `<=`. That
failure is BLK-05 and it is the intended output of this test. Do not "fix" it by editing the
mission controller as part of a test change, and do not soften the expectation.

The stimulus node shuts the launch down when its case list is complete (`on_exit=Shutdown`),
so `duration_sec` is only a safety net.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
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
    'battery': '/battery_state',
    'mission_status': '/billiebot/mission_status',
    'rosout': '/rosout',
}


def generate_launch_description():
    mission_share = get_package_share_directory('billiebot_mission')
    mission_config = os.path.join(mission_share, 'config', 'mission.yaml')

    rate_topics_config = json.dumps([
        {'name': _TOPICS['mission_status'], 'type': 'billiebot_interfaces/msg/MissionStatus',
         'required': True, 'min_rate_hz': 1.5},
        {'name': _TOPICS['battery'], 'type': 'sensor_msgs/msg/BatteryState',
         'required': True, 'min_rate_hz': 4.0},
    ])

    # 3 cases x (reset hold + set_mode + case hold) plus margin. Only a backstop: the
    # stimulus node ends the launch itself when it finishes.
    actions = declare_common_bench_args(default_duration_sec='90')
    actions += [
        DeclareLaunchArgument('mission_config_file', default_value=mission_config),
        DeclareLaunchArgument(
            'case_hold_sec', default_value='6.0',
            description='seconds to hold each boundary voltage; must span several 2 Hz '
                        'mission ticks'),
        DeclareLaunchArgument(
            'reset_hold_sec', default_value='3.0',
            description='seconds of healthy voltage before clearing SAFE via /set_mode'),
        DeclareLaunchArgument('reset_voltage', default_value='12.6'),
    ]

    actions.append(manifest_bootstrap_action(
        'UT-BAT-02B', 'mission_software', 'production mission_controller (no hardware)',
    ))

    actions.append(replicate_production_node(
        'billiebot_mission', 'mission_controller.py',
        LaunchConfiguration('mission_config_file'), {}, name='mission_controller',
    ))

    actions.append(Node(
        package='billiebot_sensor_tests',
        executable='battery_threshold_test',
        name='battery_threshold_test',
        parameters=[{
            'reset_voltage': LaunchConfiguration('reset_voltage'),
            'reset_hold_sec': LaunchConfiguration('reset_hold_sec'),
            'case_hold_sec': LaunchConfiguration('case_hold_sec'),
            'output_csv': PathJoinSubstitution(
                [LaunchConfiguration('results_dir'), 'exports', 'threshold_cases.csv']
            ),
        }],
        output='screen',
        on_exit=Shutdown(reason='UT-BAT-02B stimulus sequence complete'),
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
