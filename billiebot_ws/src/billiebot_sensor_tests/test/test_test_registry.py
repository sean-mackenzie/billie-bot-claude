"""TEST_REGISTRY and run_sensor_test wiring tests.

The registry had no test coverage before the Sensor Nano tests were added. The most valuable
assertions here are the regression ones: the eleven pre-existing OAK-D / thermal / NoIR /
audio entries must keep their exact launch files, analysis modules, profiles and durations,
and `run_sensor_test` must not start forwarding launch arguments that those older launch
files never declared.
"""

from pathlib import Path

import pytest

from billiebot_sensor_tests.orchestrate.test_registry import TEST_REGISTRY, TestSpec

#: run_sensor_test imports `launch` and `ament_index_python` at module scope, so the
#: argument-wiring checks read its source instead of importing it. That keeps this file
#: runnable without a sourced ROS install, matching how test_bench_topic_contracts.py
#: already asserts on production node internals it cannot instantiate.
_RUN_SENSOR_TEST_SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'billiebot_sensor_tests' / 'orchestrate' / 'run_sensor_test.py'
).read_text()

#: Frozen snapshot of the suite as it existed before the Sensor Nano work.
#: test_id -> (sensor, test_type, launch_file, analysis_module_suffix, profile, duration)
_PRE_EXISTING = {
    'UT-OAK-01': ('oakd', 1, 'oakd_unit_bench.launch.py', 'oakd.analyze_oakd_depth',
                   'stream', 60),
    'UT-OAK-02': ('oakd', 1, 'oakd_unit_bench.launch.py', 'oakd.analyze_oakd_depth',
                   'accuracy', 60),
    'DT-OAK-01': ('oakd', 2, 'oakd_detection_bench.launch.py', 'oakd.score_oakd_detector',
                   None, 0),
    'UT-THM-01': ('thermal', 1, 'thermal_unit_bench.launch.py',
                   'thermal.analyze_thermal_frame', 'stream', 60),
    'UT-THM-02': ('thermal', 1, 'thermal_unit_bench.launch.py',
                   'thermal.analyze_thermal_frame', 'contrast', 90),
    'DT-THM-01': ('thermal', 2, 'thermal_detection_bench.launch.py',
                   'thermal.score_thermal_blob', None, 0),
    'UT-NIR-01': ('noir', 1, 'noir_unit_bench.launch.py', 'noir.analyze_noir_image',
                   'stream', 60),
    'UT-NIR-02': ('noir', 1, 'noir_unit_bench.launch.py', 'noir.analyze_noir_image',
                   'quality', 15),
    'UT-AUD-01': ('audio', 1, 'audio_capture_bench.launch.py', 'audio.analyze_audio',
                   None, 4),
    'DT-AUD-01': ('audio', 2, 'audio_classifier_bench.launch.py',
                   'audio.score_audio_classifier', 'classification', 0),
    'DT-AUD-02': ('audio', 2, 'audio_classifier_bench.launch.py',
                   'audio.score_audio_classifier', 'doa', 0),
}

_NEW_SENSOR_NANO_IDS = ('UT-IMU-01', 'UT-IMU-02', 'UT-BAT-01', 'UT-BAT-02', 'UT-BAT-02B')


# --- regression: the original eleven tests are untouched ----------------------------------

@pytest.mark.parametrize('test_id', sorted(_PRE_EXISTING))
def test_pre_existing_test_ids_still_resolve(test_id):
    assert test_id in TEST_REGISTRY


@pytest.mark.parametrize('test_id', sorted(_PRE_EXISTING))
def test_pre_existing_specs_are_unchanged(test_id):
    sensor, test_type, launch_file, module_suffix, profile, duration = _PRE_EXISTING[test_id]
    spec = TEST_REGISTRY[test_id]
    assert spec.sensor == sensor
    assert spec.test_type == test_type
    assert spec.launch_file == launch_file
    assert spec.analysis_module == f'billiebot_sensor_tests.{module_suffix}'
    assert spec.profile == profile
    assert spec.default_duration_sec == duration


@pytest.mark.parametrize('test_id', sorted(_PRE_EXISTING))
def test_pre_existing_specs_keep_the_default_values_for_the_new_fields(test_id):
    # The new TestSpec fields were appended with defaults precisely so the original
    # positional constructions keep their old meaning.
    spec = TEST_REGISTRY[test_id]
    assert spec.operator_paced is False
    assert spec.hardware_required is True
    assert spec.required_extra_args == ()


def test_test_spec_can_still_be_constructed_positionally_with_seven_fields():
    spec = TestSpec('X-1', 'oakd', 1, 'a.launch.py', None, None, 30)
    assert spec.operator_paced is False and spec.required_extra_args == ()


def test_test_spec_is_not_collected_as_a_pytest_class():
    assert TestSpec.__test__ is False


# --- the five new tests ------------------------------------------------------------------

@pytest.mark.parametrize('test_id', _NEW_SENSOR_NANO_IDS)
def test_new_test_ids_resolve(test_id):
    assert test_id in TEST_REGISTRY
    assert TEST_REGISTRY[test_id].test_id == test_id


@pytest.mark.parametrize('test_id,launch_file,module,profile', [
    ('UT-IMU-01', 'sensor_nano_imu_bench.launch.py', 'sensor_nano.analyze_imu',
     'acquisition'),
    ('UT-IMU-02', 'sensor_nano_imu_ekf_bench.launch.py', 'sensor_nano.analyze_imu', 'ekf'),
    ('UT-BAT-01', 'sensor_nano_battery_bench.launch.py', 'sensor_nano.analyze_battery',
     None),
    ('UT-BAT-02', 'sensor_nano_battery_safe_bench.launch.py',
     'sensor_nano.score_battery_safe', 'physical'),
    ('UT-BAT-02B', 'sensor_nano_battery_threshold_bench.launch.py',
     'sensor_nano.score_battery_safe', 'threshold'),
])
def test_new_specs_point_at_the_right_launch_file_and_analyzer(test_id, launch_file, module,
                                                                profile):
    spec = TEST_REGISTRY[test_id]
    assert spec.launch_file == launch_file
    assert spec.analysis_module == f'billiebot_sensor_tests.{module}'
    assert spec.profile == profile


def test_nominal_durations_match_the_approved_test_plan():
    assert TEST_REGISTRY['UT-IMU-01'].default_duration_sec == 180
    assert TEST_REGISTRY['UT-IMU-02'].default_duration_sec == 120
    assert TEST_REGISTRY['UT-BAT-02'].default_duration_sec == 90


def test_ut_bat_01_is_operator_paced_with_no_shutdown_timer():
    # duration_sec 0 is what duration_shutdown_action() reads as "no timer", which is how the
    # PSU sweep stays alive until the operator finishes it.
    spec = TEST_REGISTRY['UT-BAT-01']
    assert spec.operator_paced is True
    assert spec.default_duration_sec == 0


def test_ut_bat_02b_is_the_only_new_test_needing_no_hardware():
    assert TEST_REGISTRY['UT-BAT-02B'].hardware_required is False
    for test_id in ('UT-IMU-01', 'UT-IMU-02', 'UT-BAT-01', 'UT-BAT-02'):
        assert TEST_REGISTRY[test_id].hardware_required is True


def test_hardware_sensor_nano_tests_declare_a_required_serial_port():
    for test_id in ('UT-IMU-01', 'UT-IMU-02', 'UT-BAT-01', 'UT-BAT-02'):
        assert 'sensor_port' in TEST_REGISTRY[test_id].required_extra_args
    assert TEST_REGISTRY['UT-BAT-02B'].required_extra_args == ()


def test_ut_bat_02b_uses_the_software_only_preflight():
    assert TEST_REGISTRY['UT-BAT-02B'].sensor == 'mission_software'


def test_every_spec_has_a_unique_test_id_matching_its_key():
    for key, spec in TEST_REGISTRY.items():
        assert key == spec.test_id


def test_registry_holds_exactly_the_expected_sixteen_tests():
    assert set(TEST_REGISTRY) == set(_PRE_EXISTING) | set(_NEW_SENSOR_NANO_IDS)
    assert len(TEST_REGISTRY) == 16


# --- run_sensor_test argument forwarding --------------------------------------------------

def test_run_sensor_test_accepts_every_registry_id_as_a_choice():
    assert 'choices=sorted(TEST_REGISTRY.keys())' in _RUN_SENSOR_TEST_SOURCE


def test_serial_arguments_are_forwarded_only_when_non_empty():
    # Unconditional forwarding would break the OAK-D / thermal / NoIR / audio launch files,
    # which never declare sensor_port or baudrate: IncludeLaunchDescription rejects an
    # argument the target launch file did not declare.
    assert 'if args.sensor_port:' in _RUN_SENSOR_TEST_SOURCE
    assert "launch_arguments['sensor_port'] = args.sensor_port" in _RUN_SENSOR_TEST_SOURCE
    assert 'if args.baudrate:' in _RUN_SENSOR_TEST_SOURCE
    assert "launch_arguments['baudrate'] = args.baudrate" in _RUN_SENSOR_TEST_SOURCE


def test_extra_launch_arg_pass_through_is_preserved():
    assert "pair.partition(':=')" in _RUN_SENSOR_TEST_SOURCE


def test_analyzers_are_only_ever_given_results_dir_config_file_and_profile():
    # Any analyzer that required another argument would be unusable through the
    # orchestrator, which is why the battery scorer's voltage arguments are optional with
    # config-backed defaults.
    assert "analysis_argv = ['--results-dir', results_dir, '--config-file', config_file]" \
        in _RUN_SENSOR_TEST_SOURCE


def test_a_missing_sensor_port_is_rejected_before_any_launch_starts():
    assert "if 'sensor_port' in spec.required_extra_args and not args.sensor_port:" \
        in _RUN_SENSOR_TEST_SOURCE
    # The guard must return before LaunchService is built, otherwise the bridge would start
    # with an empty port and fail deep inside the launch instead of at the CLI.
    guard_index = _RUN_SENSOR_TEST_SOURCE.index('required_extra_args and not args.sensor_port')
    launch_index = _RUN_SENSOR_TEST_SOURCE.index('launch_service = LaunchService()')
    assert guard_index < launch_index


def test_a_missing_sensor_port_returns_exit_code_two():
    run_sensor_test = pytest.importorskip(
        'billiebot_sensor_tests.orchestrate.run_sensor_test',
        reason='needs a sourced ROS install for launch/ament_index_python',
    )
    assert run_sensor_test.main(['--test-id', 'UT-IMU-01', '--results-dir', '/tmp/x']) == 2
