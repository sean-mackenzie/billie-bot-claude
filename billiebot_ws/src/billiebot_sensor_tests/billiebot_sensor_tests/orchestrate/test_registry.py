"""Maps bench test IDs to their launch file, analysis module, and defaults."""

from dataclasses import dataclass
from typing import Optional


@dataclass
class TestSpec:
    __test__ = False  # not a pytest test class, despite the name

    test_id: str
    sensor: str
    test_type: int  # 1 (acquisition) or 2 (detector/classifier)
    launch_file: str
    analysis_module: Optional[str]
    profile: Optional[str]
    default_duration_sec: int
    # Fields below are appended with defaults so the original eleven positional
    # constructions keep working unchanged.
    #: True when the run ends on operator Ctrl-C rather than a timer (default_duration_sec
    #: 0). Descriptive metadata for the operator and the registry tests -- the actual
    #: no-timer behaviour comes from duration_shutdown_action() seeing the literal '0'.
    operator_paced: bool = False
    #: False for tests that need no sensor hardware at all (UT-BAT-02B is software-only).
    hardware_required: bool = True
    #: Launch arguments the operator normally must supply, surfaced in error messages.
    required_extra_args: tuple = ()


TEST_REGISTRY = {
    'UT-OAK-01': TestSpec(
        'UT-OAK-01', 'oakd', 1, 'oakd_unit_bench.launch.py',
        'billiebot_sensor_tests.oakd.analyze_oakd_depth', 'stream', 60),
    'UT-OAK-02': TestSpec(
        'UT-OAK-02', 'oakd', 1, 'oakd_unit_bench.launch.py',
        'billiebot_sensor_tests.oakd.analyze_oakd_depth', 'accuracy', 60),
    'DT-OAK-01': TestSpec(
        'DT-OAK-01', 'oakd', 2, 'oakd_detection_bench.launch.py',
        'billiebot_sensor_tests.oakd.score_oakd_detector', None, 0),
    'UT-THM-01': TestSpec(
        'UT-THM-01', 'thermal', 1, 'thermal_unit_bench.launch.py',
        'billiebot_sensor_tests.thermal.analyze_thermal_frame', 'stream', 60),
    'UT-THM-02': TestSpec(
        'UT-THM-02', 'thermal', 1, 'thermal_unit_bench.launch.py',
        'billiebot_sensor_tests.thermal.analyze_thermal_frame', 'contrast', 90),
    'DT-THM-01': TestSpec(
        'DT-THM-01', 'thermal', 2, 'thermal_detection_bench.launch.py',
        'billiebot_sensor_tests.thermal.score_thermal_blob', None, 0),
    'UT-NIR-01': TestSpec(
        'UT-NIR-01', 'noir', 1, 'noir_unit_bench.launch.py',
        'billiebot_sensor_tests.noir.analyze_noir_image', 'stream', 60),
    'UT-NIR-02': TestSpec(
        'UT-NIR-02', 'noir', 1, 'noir_unit_bench.launch.py',
        'billiebot_sensor_tests.noir.analyze_noir_image', 'quality', 15),
    'UT-AUD-01': TestSpec(
        'UT-AUD-01', 'audio', 1, 'audio_capture_bench.launch.py',
        'billiebot_sensor_tests.audio.analyze_audio', None, 4),
    'DT-AUD-01': TestSpec(
        'DT-AUD-01', 'audio', 2, 'audio_classifier_bench.launch.py',
        'billiebot_sensor_tests.audio.score_audio_classifier', 'classification', 0),
    'DT-AUD-02': TestSpec(
        'DT-AUD-02', 'audio', 2, 'audio_classifier_bench.launch.py',
        'billiebot_sensor_tests.audio.score_audio_classifier', 'doa', 0),
    # -- Sensor Nano (DFRobot SEN0253 + battery divider on a dedicated Arduino Nano V3) --
    'UT-IMU-01': TestSpec(
        'UT-IMU-01', 'sensor_nano', 1, 'sensor_nano_imu_bench.launch.py',
        'billiebot_sensor_tests.sensor_nano.analyze_imu', 'acquisition', 180,
        required_extra_args=('sensor_port',)),
    'UT-IMU-02': TestSpec(
        'UT-IMU-02', 'sensor_nano', 1, 'sensor_nano_imu_ekf_bench.launch.py',
        'billiebot_sensor_tests.sensor_nano.analyze_imu', 'ekf', 120,
        required_extra_args=('sensor_port',)),
    # Operator-paced: the PSU sweep is driven by hand with a DMM, so the launch runs until
    # Ctrl-C and record_battery_point is invoked once per setpoint from a second terminal.
    'UT-BAT-01': TestSpec(
        'UT-BAT-01', 'sensor_nano', 1, 'sensor_nano_battery_bench.launch.py',
        'billiebot_sensor_tests.sensor_nano.analyze_battery', None, 0,
        operator_paced=True, required_extra_args=('sensor_port',)),
    'UT-BAT-02': TestSpec(
        'UT-BAT-02', 'sensor_nano', 2, 'sensor_nano_battery_safe_bench.launch.py',
        'billiebot_sensor_tests.sensor_nano.score_battery_safe', 'physical', 90,
        required_extra_args=('sensor_port',)),
    # Software-only boundary check; needs no Sensor Nano, no divider and no PSU.
    'UT-BAT-02B': TestSpec(
        'UT-BAT-02B', 'mission_software', 2, 'sensor_nano_battery_threshold_bench.launch.py',
        'billiebot_sensor_tests.sensor_nano.score_battery_safe', 'threshold', 90,
        hardware_required=False),
}
