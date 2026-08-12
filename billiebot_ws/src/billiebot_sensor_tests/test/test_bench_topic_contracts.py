"""Regression tests for the authoritative-vs-visualization split.

The visualization work must be provably incapable of changing what gets recorded, what gets
rate-gated, or what any analysis reads. These tests encode that as a contract so a future edit
that quietly routes a preview topic into the bag, or relaxes a raw rate threshold, fails here.
"""

import ast
import importlib.util
import inspect
import json
import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

_SHARE = get_package_share_directory('billiebot_sensor_tests')

_AUTHORITATIVE_OAK_TOPICS = {
    '/bench/oakd/rgb/image_raw',
    '/bench/oakd/rgb/camera_info',
    '/bench/oakd/depth/image_raw',
    '/bench/oakd/depth/camera_info',
    '/bench/oakd/points',
    '/bench/oakd/diagnostics',
}


def _load_launch_module(filename):
    path = os.path.join(_SHARE, 'launch', filename)
    spec = importlib.util.spec_from_file_location(filename, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _bench_config() -> dict:
    with open(os.path.join(_SHARE, 'config', 'sensor_bench.yaml')) as f:
        return yaml.safe_load(f)


def _foxglove_layout() -> dict:
    with open(os.path.join(_SHARE, 'foxglove', 'billiebot_sensor_bench.json')) as f:
        return json.load(f)


def _recorded_topics(ld) -> list:
    """Topic list handed to `ros2 bag record` -- the tail of the ExecuteProcess cmd."""
    for entity in ld.entities:
        if isinstance(entity, ExecuteProcess):
            cmd = [c for c in entity.cmd if isinstance(c, list) and len(c) == 1]
            literals = [c[0].text for c in cmd if hasattr(c[0], 'text')]
            if 'record' in literals:
                return [t for t in literals if t.startswith('/')]
    raise AssertionError('no `ros2 bag record` ExecuteProcess found in the launch description')


def _node_named(ld, name):
    for entity in ld.entities:
        if isinstance(entity, Node) and entity._Node__node_name == name:
            return entity
    return None


def _context(ld, **overrides) -> LaunchContext:
    """A LaunchContext seeded with every declared argument's default, so substitutions and
    conditions resolve exactly as they would on a bare `ros2 launch` with no extra arguments."""
    context = LaunchContext()
    for entity in ld.entities:
        if not isinstance(entity, DeclareLaunchArgument):
            continue
        if entity.default_value is None:
            # `results_dir` is deliberately mandatory; a placeholder keeps PathJoinSubstitution
            # resolvable so the rest of the description can be inspected.
            context.launch_configurations[entity.name] = '/tmp/bench_contract_test'
        else:
            context.launch_configurations[entity.name] = perform_substitutions(
                context, list(entity.default_value)
            )
    context.launch_configurations.update(overrides)
    return context


def _node_params(node, context) -> dict:
    """launch_ros normalizes `parameters=[{...}]` into substitution tuples; this performs them
    back into a plain dict of strings."""
    resolved = {}
    for params in node._Node__parameters:
        if not isinstance(params, dict):
            continue
        for key, value in params.items():
            name = perform_substitutions(context, list(key))
            try:
                resolved[name] = perform_substitutions(context, list(value))
            except TypeError:
                resolved[name] = value
    return resolved


def _json_param(params: dict, key: str):
    """launch_ros YAML-quotes string parameter values (so YAML type inference keeps them
    strings); unwrap that before parsing the JSON payload back out."""
    assert key in params, f'expected parameter {key} is missing'
    return json.loads(yaml.safe_load(params[key]))


def _rate_monitor_config(ld) -> list:
    node = _node_named(ld, 'topic_rate_monitor')
    assert node is not None, 'topic_rate_monitor node missing from launch description'
    return _json_param(_node_params(node, _context(ld)), 'topics_config_json')


# --- 1. authoritative topics still exist -----------------------------------------------


def test_unit_bench_still_declares_every_authoritative_topic():
    module = _load_launch_module('oakd_unit_bench.launch.py')
    assert set(module._TOPICS.values()) == _AUTHORITATIVE_OAK_TOPICS


def test_sensor_bench_config_still_names_the_authoritative_topics():
    topics = _bench_config()['oakd']['topics']
    assert topics['rgb'] == '/bench/oakd/rgb/image_raw'
    assert topics['depth'] == '/bench/oakd/depth/image_raw'
    assert topics['points'] == '/bench/oakd/points'


# --- 2. raw encodings unchanged --------------------------------------------------------


def test_raw_encodings_are_unchanged():
    encodings = _bench_config()['oakd']['encodings']
    assert encodings['rgb'] == 'bgr8'
    assert encodings['depth'] == '16UC1'  # millimetres, lossless -- never a preview


def test_preview_settings_are_not_thresholds():
    """A visualization knob must never be reachable through the threshold accessors, or a
    Foxglove tweak could move a pass/fail boundary."""
    oakd = _bench_config()['oakd']
    assert 'visualization' in oakd
    flat = json.dumps(oakd['thresholds'])
    for term in ('preview', 'jpeg', 'visualization', 'stride'):
        assert term not in flat


# --- 3. UT-OAK rate requirements still apply to the authoritative streams ---------------


def test_rate_monitor_still_gates_the_raw_streams_at_4_5_hz():
    config = _rate_monitor_config(_load_launch_module('oakd_unit_bench.launch.py')
                                  .generate_launch_description())
    by_topic = {entry['name']: entry for entry in config}
    assert by_topic['/bench/oakd/rgb/image_raw']['min_rate_hz'] == 4.5
    assert by_topic['/bench/oakd/depth/image_raw']['min_rate_hz'] == 4.5
    assert by_topic['/bench/oakd/rgb/image_raw']['required'] is True
    assert by_topic['/bench/oakd/points']['required'] is True


def test_config_rate_thresholds_are_unchanged():
    required = _bench_config()['oakd']['thresholds']['required']
    assert required['ut_oak_01_min_avg_rate_hz'] == 4.5
    assert required['ut_oak_01_max_gap_s'] == 1.0
    assert required['dt_oak_01_min_found_rate_hz'] == 4.5


# --- 4 + 5. previews cannot change recording, rate gating, or analysis ------------------


@pytest.mark.parametrize('filename', ['oakd_unit_bench.launch.py',
                                       'oakd_detection_bench.launch.py'])
def test_no_preview_topic_is_ever_recorded(filename):
    module = _load_launch_module(filename)
    recorded = _recorded_topics(module.generate_launch_description())
    assert recorded == list(module._TOPICS.values())
    assert not any('preview' in t for t in module._TOPICS.values() if t.startswith('/bench'))
    for preview_topic in module._PREVIEW_TOPICS.values():
        assert preview_topic not in recorded


def test_no_preview_topic_is_rate_gated():
    module = _load_launch_module('oakd_unit_bench.launch.py')
    config = _rate_monitor_config(module.generate_launch_description())
    gated = {entry['name'] for entry in config}
    assert gated.isdisjoint(set(module._PREVIEW_TOPICS.values()))
    assert gated <= _AUTHORITATIVE_OAK_TOPICS


@pytest.mark.parametrize('filename', ['oakd_unit_bench.launch.py',
                                       'oakd_detection_bench.launch.py'])
@pytest.mark.parametrize('previews', ['true', 'false'])
def test_recording_and_rate_gating_are_identical_with_previews_on_or_off(filename, previews):
    """`generate_launch_description()` builds the bag/rate-monitor wiring from `_TOPICS` alone,
    so the preview argument cannot reach them by construction. Asserted explicitly because that
    invariant is the whole safety argument for enabling previews by default."""
    module = _load_launch_module(filename)
    ld = module.generate_launch_description()
    _context(ld, start_visualization_previews=previews)
    assert _recorded_topics(ld) == list(module._TOPICS.values())


def test_disabling_previews_leaves_acquisition_and_analysis_nodes_running():
    module = _load_launch_module('oakd_unit_bench.launch.py')
    ld = module.generate_launch_description()
    for node_name in ('oakd_bench_publisher', 'topic_rate_monitor'):
        node = _node_named(ld, node_name)
        assert node is not None
        # Unconditional: acquisition/analysis must never be gated on a visualization argument.
        assert node.condition is None


def test_detection_bench_preview_node_is_the_only_condition_gated_node():
    module = _load_launch_module('oakd_detection_bench.launch.py')
    ld = module.generate_launch_description()
    preview_node = _node_named(ld, 'bench_preview_node')
    assert preview_node is not None and preview_node.condition is not None

    for node_name in ('oakd_dog_detector', 'topic_rate_monitor',
                      'oakd_preview_overlay', 'ground_truth_marker_node'):
        node = _node_named(ld, node_name)
        assert node is not None and node.condition is None


@pytest.mark.parametrize('previews,expected', [('true', True), ('false', False)])
def test_preview_node_condition_follows_the_launch_argument(previews, expected):
    module = _load_launch_module('oakd_detection_bench.launch.py')
    ld = module.generate_launch_description()
    preview_node = _node_named(ld, 'bench_preview_node')
    context = _context(ld, start_visualization_previews=previews)
    assert preview_node.condition.evaluate(context) is expected


def test_unit_bench_previews_reach_the_publisher_as_parameters_only():
    """UT-OAK has no conditioned node: previews are a parameter on the acquisition node, so the
    process topology is identical either way and only the extra publishers appear."""
    module = _load_launch_module('oakd_unit_bench.launch.py')
    ld = module.generate_launch_description()
    params = _node_params(_node_named(ld, 'oakd_bench_publisher'),
                          _context(ld, start_visualization_previews='false'))
    assert params['publish_previews'] == 'false'
    assert params['preview_format'] == 'jpeg'
    assert params['points_preview_stride'] == '16'

    on = _node_params(_node_named(ld, 'oakd_bench_publisher'), _context(ld))
    assert on['publish_previews'] == 'true'  # default-on for the bench launch


def test_unit_bench_preview_argument_defaults_on():
    ld = _load_launch_module('oakd_unit_bench.launch.py').generate_launch_description()
    declared = {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}
    assert declared['start_visualization_previews'].default_value[0].text == 'true'


# --- 6. Foxglove layout uses previews, not the large raw streams ------------------------


def test_oakd_layout_panels_point_at_preview_topics_only():
    config = _foxglove_layout()['configById']
    oak_image_panels = [k for k in config if k.startswith('ImageViewPanel!oak')]
    assert oak_image_panels, 'OAK-D image panels missing from the layout'
    for key in oak_image_panels:
        topic = config[key]['cameraTopic']
        assert 'preview' in topic, f'{key} still subscribes to {topic}'

    cloud_topics = list(config['3D!oak_points']['topics'])
    assert cloud_topics == ['/bench/oakd/points_preview']


def test_no_high_bandwidth_raw_topic_is_in_the_default_layout():
    blob = json.dumps(_foxglove_layout())
    for raw_topic in ('/bench/oakd/rgb/image_raw', '/bench/oakd/depth/image_raw'):
        assert raw_topic not in blob
    assert '"/bench/oakd/points"' not in blob


def test_layout_covers_both_ut_oak_and_dt_oak_previews():
    blob = json.dumps(_foxglove_layout())
    for topic in ('/bench/oakd/rgb/preview/compressed',
                  '/bench/oakd/depth/preview/compressed',
                  '/bench/oakd/points_preview',
                  '/bench/oakd_detector/rgb/preview/compressed',
                  '/bench/oakd_detector/annotated/preview/compressed',
                  '/bench/oakd_detector/depth/preview/compressed'):
        assert topic in blob
    # DT-OAK-01's lightweight non-image evidence stays in the layout.
    for topic in ('/dog/found', '/dog/detections_3d', '/bench/oakd_detector/diagnostics'):
        assert topic in blob


# --- 7. preview topics use the expected message types ----------------------------------


def test_compressed_previews_are_declared_as_compressed_image():
    from billiebot_sensor_tests.common import preview_node as pn
    source = inspect.getsource(pn)
    assert 'CompressedImage' in source

    module = _load_launch_module('oakd_unit_bench.launch.py')
    for topic in (module._PREVIEW_TOPICS['rgb_preview'],
                  module._PREVIEW_TOPICS['depth_preview']):
        assert topic.endswith('/compressed')
    assert module._PREVIEW_TOPICS['points_preview'] == '/bench/oakd/points_preview'


def test_publisher_creates_compressed_and_pointcloud_preview_publishers():
    from billiebot_sensor_tests.oakd import oakd_bench_publisher as pub
    source = inspect.getsource(pub)
    assert "create_publisher(preview_msg_type, rgb_topic, 1)" in source
    assert "create_publisher(PointCloud2, '/bench/oakd/points_preview', 1)" in source


def test_detection_bench_preview_sources_are_the_detector_streams():
    module = _load_launch_module('oakd_detection_bench.launch.py')
    ld = module.generate_launch_description()
    params = _node_params(_node_named(ld, 'bench_preview_node'), _context(ld))
    image_sources = _json_param(params, 'image_sources_json')
    depth_sources = _json_param(params, 'depth_sources_json')
    assert {s['in'] for s in image_sources} == {'/oak/rgb/preview', '/oak/rgb/annotated'}
    assert {s['in'] for s in depth_sources} == {'/oak/depth/preview'}
    assert {s['out'] for s in image_sources + depth_sources} == set(module._PREVIEW_TOPICS.values())


# --- 8. production OAK-D deployment defaults unchanged ----------------------------------


def _declared_parameter_defaults(module) -> dict:
    """Reads `declare_parameter(name, default)` literals straight out of the source, so this
    check needs no ROS context and cannot be fooled by a launch-time override."""
    tree = ast.parse(inspect.getsource(module))
    defaults = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'declare_parameter' and len(node.args) == 2):
            try:
                defaults[ast.literal_eval(node.args[0])] = ast.literal_eval(node.args[1])
            except ValueError:
                continue
    return defaults


def test_production_detector_bench_outputs_remain_default_off():
    from billiebot_perception import oakd_dog_detector
    defaults = _declared_parameter_defaults(oakd_dog_detector)
    assert defaults['publish_preview'] is False
    assert defaults['publish_depth_preview'] is False
    assert defaults['publish_diagnostics'] is False


def test_production_detector_gained_no_visualization_parameters():
    """DT-OAK-01's Foxglove path is entirely bench-side. If a preview/compression parameter ever
    appears on the production node, deployment behaviour is no longer provably untouched."""
    from billiebot_perception import oakd_dog_detector
    defaults = _declared_parameter_defaults(oakd_dog_detector)
    assert not [k for k in defaults if 'compress' in k or 'jpeg' in k or 'quality' in k]


def test_bench_publisher_previews_default_off_outside_a_launch_file():
    from billiebot_sensor_tests.oakd import oakd_bench_publisher
    defaults = _declared_parameter_defaults(oakd_bench_publisher)
    assert defaults['publish_previews'] is False
    # Authoritative acquisition defaults are untouched by this work.
    assert defaults['fps'] == 5.0
    assert defaults['point_cloud_stride'] == 4
    assert defaults['points_preview_stride'] == 16
