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
}
