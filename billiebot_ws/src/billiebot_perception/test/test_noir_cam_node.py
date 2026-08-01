import pytest
import rclpy

from billiebot_perception.noir_cam_node import NoirCamNode, build_camera_config


def test_build_camera_config_matches_current_default():
    # Exact dict equality with the original hardcoded
    # create_still_configuration(main={"size": (w, h), "format": "RGB888"}) call --
    # proves every new bench-only control defaults to a true no-op.
    cfg = build_camera_config(640, 480)
    assert cfg == {"main": {"size": (640, 480), "format": "RGB888"}}
    assert 'controls' not in cfg


def test_build_camera_config_adds_controls_when_set():
    cfg = build_camera_config(
        640, 480, af_mode='continuous', af_trigger=True, lens_position=2.0,
        exposure_time_us=10000, analogue_gain=4.0, frame_duration_limits_us=(20000, 20000),
    )
    assert cfg['controls'] == {
        'AfMode': 2,
        'AfTrigger': 0,
        'LensPosition': 2.0,
        'ExposureTime': 10000,
        'AnalogueGain': 4.0,
        'FrameDurationLimits': (20000, 20000),
    }


def test_build_camera_config_ignores_unknown_af_mode():
    cfg = build_camera_config(640, 480, af_mode='not-a-real-mode')
    assert 'controls' not in cfg


@pytest.fixture()
def node():
    rclpy.init(args=['--ros-args', '-p', 'mock:=true'])
    n = NoirCamNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_metadata_publisher_off_by_default(node):
    assert node.publish_metadata is False
    assert node.metadata_pub is None


def test_mock_publish_unaffected(node):
    node.image_pub.publish = lambda msg: setattr(node, '_last_msg', msg)
    node.mock_publish()
    assert node._last_msg.encoding == 'rgb8'
    assert node._last_msg.width == 640
    assert node._last_msg.height == 480
    # rclpy stores Image.data as array.array('B', ...), not plain bytes -- compare the
    # actual byte content, not the container type.
    assert bytes(node._last_msg.data) == bytes([40, 40, 40] * 640 * 480)
