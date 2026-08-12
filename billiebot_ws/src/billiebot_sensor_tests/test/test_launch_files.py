"""Import + generate_launch_description() smoke test for all 7 bench launch files.

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
