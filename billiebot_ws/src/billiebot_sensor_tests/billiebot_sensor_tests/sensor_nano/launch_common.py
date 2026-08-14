"""Shared launch pieces for the four Sensor Nano bench launch files.

Keeps the bridge's parameter wiring in one place, and — more importantly — keeps
`config/sensor_bench.yaml` the single source of truth for the divider ratio, ADC reference,
frame IDs and orientation convention. Launch arguments default to the empty string and only
override the YAML when the operator actually supplies one, so a value can never be silently
different depending on whether a run went through `ros2 launch` or `run_sensor_test`.

No rclpy here: this module is imported at launch-description construction time, alongside
`launch`/`launch_ros`, exactly like the launch files themselves.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from billiebot_sensor_tests.common.config import load_bench_config

#: Launch arguments every Sensor Nano bench launch file accepts, on top of
#: `declare_common_bench_args()`. Empty defaults mean "take it from the config file".
SENSOR_NANO_ARGS = (
    ('sensor_port', '', 'serial device for the Sensor Nano; prefer a stable '
                         '/dev/serial/by-path/... entry over /dev/ttyUSB0 (BLK-09)'),
    ('baudrate', '', 'Sensor Nano serial baud rate (firmware default 115200)'),
    ('battery_divider_ratio', '', 'V_BAT / V_A0 of the installed divider; measure it, do '
                                   'not tune it to make a run pass'),
    ('adc_reference_voltage', '', 'measured Nano 5 V / AVcc rail, in volts'),
    ('orientation_frame_convention', '', "'bno055_native' (default) or 'nwu_to_enu'"),
    ('imu_frame_id', '', 'frame_id stamped on /imu/data'),
    ('publish_battery', 'true', 'publish /battery_state and /bench/battery/adc'),
    ('publish_magnetometer', 'true', 'publish /imu/mag when the firmware sends M records'),
)


def declare_sensor_nano_args() -> list:
    return [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description in SENSOR_NANO_ARGS
    ]


def default_config_path() -> str:
    return os.path.join(
        get_package_share_directory('billiebot_sensor_tests'), 'config', 'sensor_bench.yaml'
    )


def sensor_nano_bridge_action() -> OpaqueFunction:
    """The sensor_nano_bridge Node, with parameters resolved from the bench config file and
    overridden only by launch arguments the operator actually set."""

    def _build(context, *args, **kwargs):
        config_file = LaunchConfiguration('config_file').perform(context) or default_config_path()
        cfg = load_bench_config(config_file)
        results_dir = LaunchConfiguration('results_dir').perform(context)

        def _arg(name):
            return LaunchConfiguration(name).perform(context).strip()

        port = _arg('sensor_port')
        baudrate = _arg('baudrate') or cfg.get('sensor_nano.serial.baudrate', 115200)
        divider = _arg('battery_divider_ratio') or cfg.get(
            'sensor_nano.battery.divider_ratio', 6.0)
        adc_reference = _arg('adc_reference_voltage') or cfg.get(
            'sensor_nano.battery.adc_reference_voltage', 5.0)
        convention = _arg('orientation_frame_convention') or cfg.get(
            'sensor_nano.orientation.frame_convention', 'bno055_native')
        imu_frame = _arg('imu_frame_id') or cfg.get('sensor_nano.frames.imu', 'imu_link')

        covariance = cfg.get('sensor_nano.imu_covariance', {}) or {}

        parameters = {
            'port': port,
            'baudrate': int(baudrate),
            'imu_frame_id': str(imu_frame),
            'barometer_frame_id': str(cfg.get('sensor_nano.frames.barometer', 'imu_link')),
            'battery_frame_id': str(cfg.get('sensor_nano.frames.battery', 'base_link')),
            'orientation_frame_convention': str(convention),
            'battery_divider_ratio': float(divider),
            'adc_reference_voltage': float(adc_reference),
            'battery_cell_count': int(cfg.get('sensor_nano.battery.cell_count', 3)),
            'battery_low_voltage': float(cfg.get('sensor_nano.battery.safe_threshold_v', 10.5)),
            'battery_critical_voltage': float(
                cfg.get('sensor_nano.battery.critical_voltage_v', 9.9)),
            'startup_settle_sec': float(
                cfg.get('sensor_nano.serial.startup_settle_sec', 2.5)),
            'publish_battery': _arg('publish_battery').lower() != 'false',
            'publish_magnetometer': _arg('publish_magnetometer').lower() != 'false',
            'fail_on_missing_device':
                LaunchConfiguration('fail_on_missing_device').perform(context).lower() != 'false',
            # Redundant, bag-independent copy of the parser/peripheral counters. The
            # analyzers prefer this file and fall back to the bagged diagnostics topic.
            'stats_export_path': os.path.join(
                results_dir, 'exports', 'sensor_nano_parser_stats.json'),
            'orientation_covariance_diagonal': [
                float(v) for v in covariance.get('orientation_diagonal', [0.01, 0.01, 0.05])],
            'angular_velocity_covariance_diagonal': [
                float(v) for v in covariance.get(
                    'angular_velocity_diagonal', [0.005, 0.005, 0.005])],
            'linear_acceleration_covariance_diagonal': [
                float(v) for v in covariance.get(
                    'linear_acceleration_diagonal', [0.1, 0.1, 0.1])],
        }

        return [Node(
            package='billiebot_sensor_tests',
            executable='sensor_nano_bridge',
            name='sensor_nano_bridge',
            parameters=[parameters],
            output='screen',
        )]

    return OpaqueFunction(function=_build)


def ground_truth_marker_action() -> Node:
    """Operator segment marking, reused verbatim from the existing bench suite. The IMU
    tests depend on it: without `mark <label>` lines the commanded-rotation gate has no
    evidence to score and fails."""
    from launch.substitutions import PathJoinSubstitution

    return Node(
        package='billiebot_sensor_tests',
        executable='ground_truth_marker_node',
        name='ground_truth_marker_node',
        parameters=[{
            'output_csv': PathJoinSubstitution(
                [LaunchConfiguration('results_dir'), 'exports', 'ground_truth_segments.csv']
            ),
        }],
        output='screen',
    )
