"""UT-BAT-02: physical low-battery SAFE propagation (90 s default).

Real chain, end to end:

    PSU -> divider -> Sensor Nano A0 -> sensor_nano_bridge -> /battery_state
        -> PRODUCTION mission_controller -> /billiebot/mission_status -> SAFE

The mission controller started here is the real `billiebot_mission` node loaded with the
production `mission.yaml`, not a reimplemented state machine -- the whole point is to test
the shipping safety logic. It runs standalone: it constructs a NavigateToPose ActionClient
but never waits on a server, so Nav2 is not required.

NOT started, on purpose: the Motor Nano, `base_bridge`, motors, encoders, Nav2. `base_bridge`
in particular still owns its own /battery_state publisher fed from the Motor Nano's A0
(BLK-02); running both would put two publishers on the topic and make the result meaningless.

Note `mission_controller` latches SAFE with no exit path, so after the low-voltage step the
mission stays SAFE until /set_mode is called. That is existing production behaviour, not a
bench artefact.
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
from billiebot_sensor_tests.sensor_nano.launch_common import (
    declare_sensor_nano_args,
    ground_truth_marker_action,
    sensor_nano_bridge_action,
)

_TOPICS = {
    'battery': '/battery_state',
    'battery_adc': '/bench/battery/adc',
    'mission_status': '/billiebot/mission_status',
    'diagnostics': '/bench/sensor_nano/diagnostics',
    'rosout': '/rosout',
}


def generate_launch_description():
    mission_share = get_package_share_directory('billiebot_mission')
    mission_config = os.path.join(mission_share, 'config', 'mission.yaml')

    rate_topics_config = json.dumps([
        {'name': _TOPICS['battery'], 'type': 'sensor_msgs/msg/BatteryState',
         'required': True, 'min_rate_hz': 4.0},
        {'name': _TOPICS['mission_status'], 'type': 'billiebot_interfaces/msg/MissionStatus',
         'required': True, 'min_rate_hz': 1.5},
    ])

    actions = declare_common_bench_args(default_duration_sec='90')
    actions += declare_sensor_nano_args()
    actions.append(DeclareLaunchArgument(
        'mission_config_file', default_value=mission_config,
        description='production billiebot_mission parameters; the real thresholds under test'))

    actions.append(manifest_bootstrap_action(
        'UT-BAT-02', 'sensor_nano', 'DFRobot SEN0253 + battery divider on Arduino Nano V3',
        preflight_context_args=('sensor_port',),
    ))

    actions.append(sensor_nano_bridge_action())

    # name= is required here: billiebot_mission is ament_cmake and installs its entry point
    # as `mission_controller.py`, but a ROS node name cannot contain a dot, and mission.yaml
    # is keyed `mission_controller:` so the parameters would not load under any other name.
    actions.append(replicate_production_node(
        'billiebot_mission', 'mission_controller.py',
        LaunchConfiguration('mission_config_file'), {}, name='mission_controller',
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

    # For marking the PSU steps ("mark high_voltage", "mark psu_step_to_low").
    actions.append(ground_truth_marker_action())

    actions += record_bag_action(list(_TOPICS.values()))
    actions.append(foxglove_bridge_action())
    actions += duration_shutdown_action()
    actions.append(finalize_on_shutdown_action())

    return LaunchDescription(actions)
