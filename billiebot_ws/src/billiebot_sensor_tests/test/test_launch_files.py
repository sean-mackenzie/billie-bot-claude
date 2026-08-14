"""Import + generate_launch_description() smoke test for all 12 bench launch files.

Proves each one constructs without error (import mistakes, argument-wiring bugs) without
spawning any process, executing any Node, or touching hardware -- LaunchDescription
construction is pure Python object-graph building.
"""

import importlib.util
import os

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions

LAUNCH_FILES = [
    'oakd_unit_bench.launch.py',
    'oakd_detection_bench.launch.py',
    'thermal_unit_bench.launch.py',
    'thermal_detection_bench.launch.py',
    'noir_unit_bench.launch.py',
    'audio_capture_bench.launch.py',
    'audio_classifier_bench.launch.py',
    'sensor_nano_imu_bench.launch.py',
    'sensor_nano_imu_ekf_bench.launch.py',
    'sensor_nano_battery_bench.launch.py',
    'sensor_nano_battery_safe_bench.launch.py',
    'sensor_nano_battery_threshold_bench.launch.py',
]

SENSOR_NANO_LAUNCH_FILES = [
    'sensor_nano_imu_bench.launch.py',
    'sensor_nano_imu_ekf_bench.launch.py',
    'sensor_nano_battery_bench.launch.py',
    'sensor_nano_battery_safe_bench.launch.py',
]


def _load_launch_module(filename):
    share = get_package_share_directory('billiebot_sensor_tests')
    path = os.path.join(share, 'launch', filename)
    spec = importlib.util.spec_from_file_location(filename, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PREVIEW_LAUNCH_FILES = [
    'oakd_unit_bench.launch.py',
    'oakd_detection_bench.launch.py',
]


@pytest.mark.parametrize('filename', LAUNCH_FILES)
def test_launch_file_constructs(filename):
    module = _load_launch_module(filename)
    assert hasattr(module, 'generate_launch_description')
    ld = module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    assert len(ld.entities) > 0


@pytest.mark.parametrize('filename', PREVIEW_LAUNCH_FILES)
@pytest.mark.parametrize('previews', ['true', 'false'])
def test_preview_argument_resolves_for_both_settings(filename, previews):
    """Constructs the description and resolves every condition under both settings of
    `start_visualization_previews`, so a substitution/typo in the visualization wiring fails
    here rather than on the bench."""
    module = _load_launch_module(filename)
    ld = module.generate_launch_description()
    context = LaunchContext()
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            context.launch_configurations[entity.name] = perform_substitutions(
                context, list(entity.default_value)
            )
    context.launch_configurations['start_visualization_previews'] = previews
    for entity in ld.entities:
        if getattr(entity, 'condition', None) is not None:
            assert isinstance(entity.condition.evaluate(context), bool)


@pytest.mark.parametrize('filename', PREVIEW_LAUNCH_FILES)
def test_preview_argument_is_declared_and_defaults_on(filename):
    ld = _load_launch_module(filename).generate_launch_description()
    declared = {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}
    assert 'start_visualization_previews' in declared
    assert declared['start_visualization_previews'].default_value[0].text == 'true'


def _declared_arguments(filename) -> dict:
    ld = _load_launch_module(filename).generate_launch_description()
    return {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}


@pytest.mark.parametrize('filename', SENSOR_NANO_LAUNCH_FILES)
def test_sensor_nano_launch_files_declare_the_serial_arguments(filename):
    # run_sensor_test forwards sensor_port/baudrate only to launch files that declare them;
    # a missing declaration here would make the orchestrated path fail at include time.
    declared = _declared_arguments(filename)
    for name in ('sensor_port', 'baudrate', 'battery_divider_ratio',
                  'adc_reference_voltage', 'orientation_frame_convention'):
        assert name in declared, f'{filename} does not declare {name}'


@pytest.mark.parametrize('filename', SENSOR_NANO_LAUNCH_FILES)
def test_sensor_nano_serial_arguments_default_to_empty_so_the_config_file_wins(filename):
    # Empty defaults are what keep config/sensor_bench.yaml the single source of truth for
    # the divider ratio and ADC reference; a non-empty default here would silently shadow it.
    declared = _declared_arguments(filename)
    for name in ('battery_divider_ratio', 'adc_reference_voltage',
                  'orientation_frame_convention'):
        assert declared[name].default_value[0].text == ''


@pytest.mark.parametrize('filename', LAUNCH_FILES)
def test_every_launch_file_declares_the_common_bench_arguments(filename):
    declared = _declared_arguments(filename)
    for name in ('results_dir', 'duration_sec', 'record_bag', 'start_foxglove',
                  'config_file'):
        assert name in declared, f'{filename} does not declare {name}'


def test_ut_bat_01_defaults_to_operator_paced_with_no_shutdown_timer():
    declared = _declared_arguments('sensor_nano_battery_bench.launch.py')
    assert declared['duration_sec'].default_value[0].text == '0'


@pytest.mark.parametrize('filename,expected', [
    ('sensor_nano_imu_bench.launch.py', '180'),
    ('sensor_nano_imu_ekf_bench.launch.py', '120'),
    ('sensor_nano_battery_safe_bench.launch.py', '90'),
])
def test_sensor_nano_durations_match_the_approved_test_plan(filename, expected):
    assert _declared_arguments(filename)['duration_sec'].default_value[0].text == expected


def test_ut_bat_02b_launch_needs_no_serial_port():
    # UT-BAT-02B is software-only; declaring a sensor_port would imply hardware it does not
    # use and would let an operator think a Sensor Nano must be connected.
    declared = _declared_arguments('sensor_nano_battery_threshold_bench.launch.py')
    assert 'sensor_port' not in declared
