import random
from unittest.mock import MagicMock

import pytest
import rclpy

from billiebot_perception.oakd_dog_detector import OakdDogDetector, scale_bbox


def test_scale_bbox_produces_positive_dims():
    x, y, w, h = scale_bbox(0.2, 0.3, 0.6, 0.8, frame_w=416, frame_h=416)
    assert w > 0
    assert h > 0
    assert x == round(0.2 * 416)
    assert y == round(0.3 * 416)


def test_scale_bbox_clamps_to_frame():
    x, y, w, h = scale_bbox(-0.1, -0.1, 1.2, 1.2, frame_w=416, frame_h=416)
    assert x == 0
    assert y == 0
    assert x + w <= 416
    assert y + h <= 416


def test_scale_bbox_fixes_the_original_zero_width_defect():
    # A typical near-full-frame dog detection: normalized xmin/ymin/xmax/ymax all < 1.0.
    # The pre-fix code (`int(det.xmin)` etc.) truncated every such box to bbox_w=bbox_h=0
    # -- this is the regression the fix (Appendix B-11) proves is closed.
    x, y, w, h = scale_bbox(0.3, 0.25, 0.75, 0.9)
    assert w > 0
    assert h > 0


@pytest.fixture()
def node():
    rclpy.init(args=['--ros-args', '-p', 'mock:=true'])
    n = OakdDogDetector()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_preview_publishers_off_by_default(node):
    assert node.publish_preview is False
    assert node.publish_depth_preview is False
    assert node.publish_diagnostics is False
    assert node.preview_pub is None
    assert node.depth_preview_pub is None
    assert node.diag_pub is None


def test_mock_detect_unaffected(node):
    node.detection_pub.publish = MagicMock()
    node.found_pub.publish = MagicMock()
    random.seed(0)
    for _ in range(20):  # 70% detection chance per tick -- 20 ticks makes a miss unlikely
        node.mock_detect()
    assert node.detection_pub.publish.called
    published_msg = node.detection_pub.publish.call_args[0][0]
    assert (
        published_msg.bbox_x, published_msg.bbox_y, published_msg.bbox_w, published_msg.bbox_h
    ) == (150, 200, 120, 80)
