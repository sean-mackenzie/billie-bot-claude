"""Import + generate_launch_description() smoke test for all 7 bench launch files.

Proves each one constructs without error (import mistakes, argument-wiring bugs) without
spawning any process, executing any Node, or touching hardware -- LaunchDescription
construction is pure Python object-graph building.
"""

import importlib.util
import os

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

LAUNCH_FILES = [
    'oakd_unit_bench.launch.py',
    'oakd_detection_bench.launch.py',
    'thermal_unit_bench.launch.py',
    'thermal_detection_bench.launch.py',
    'noir_unit_bench.launch.py',
    'audio_capture_bench.launch.py',
    'audio_classifier_bench.launch.py',
]


def _load_launch_module(filename):
    share = get_package_share_directory('billiebot_sensor_tests')
    path = os.path.join(share, 'launch', filename)
    spec = importlib.util.spec_from_file_location(filename, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('filename', LAUNCH_FILES)
def test_launch_file_constructs(filename):
    module = _load_launch_module(filename)
    assert hasattr(module, 'generate_launch_description')
    ld = module.generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    assert len(ld.entities) > 0
