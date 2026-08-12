#!/usr/bin/env python3
"""Test-only bbox overlay for DT-OAK-01: subscribes /oak/rgb/preview + /dog/detections_3d
(both only published when the production oakd_dog_detector's bench-only
publish_preview/publish_diagnostics params are enabled -- plan section 5a), draws a
rectangle outline in pure numpy (no cv2 dependency), and publishes /oak/rgb/annotated.
Never touches the production detector itself.
"""

import array
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from billiebot_interfaces.msg import DogDetection3D


def draw_bbox_outline(image: np.ndarray, x: int, y: int, w: int, h: int,
                       color=(0, 255, 0), thickness: int = 2) -> np.ndarray:
    """Pure numpy rectangle-outline draw -- no cv2 dependency."""
    out = image.copy()
    h_img, w_img = out.shape[0], out.shape[1]
    x0, y0 = max(0, x), max(0, y)
    x1, y1 = min(w_img, x + w), min(h_img, y + h)
    if y1 <= y0 or x1 <= x0:
        return out
    t = thickness
    out[y0:min(y0 + t, y1), x0:x1] = color
    out[max(y1 - t, y0):y1, x0:x1] = color
    out[y0:y1, x0:min(x0 + t, x1)] = color
    out[y0:y1, max(x1 - t, x0):x1] = color
    return out


class OakdPreviewOverlay(Node):
    def __init__(self):
        super().__init__('oakd_preview_overlay')
        self.declare_parameter('preview_topic', '/oak/rgb/preview')
        self.declare_parameter('detections_topic', '/dog/detections_3d')
        self.declare_parameter('annotated_topic', '/oak/rgb/annotated')
        self.declare_parameter('detection_ttl_sec', 0.5)

        preview_topic = str(self.get_parameter('preview_topic').value)
        detections_topic = str(self.get_parameter('detections_topic').value)
        annotated_topic = str(self.get_parameter('annotated_topic').value)
        self.detection_ttl_sec = float(self.get_parameter('detection_ttl_sec').value)

        self._lock = threading.Lock()
        self._latest_detections = []  # list of (t_sec, DogDetection3D)

        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.preview_sub = self.create_subscription(Image, preview_topic, self._on_preview, 10)
        self.detection_sub = self.create_subscription(
            DogDetection3D, detections_topic, self._on_detection, 10
        )

    def _on_detection(self, msg: DogDetection3D):
        now = self.get_clock().now().nanoseconds / 1e9
        with self._lock:
            self._latest_detections.append((now, msg))
            cutoff = now - self.detection_ttl_sec
            self._latest_detections = [
                (t, m) for t, m in self._latest_detections if t >= cutoff
            ]

    def _on_preview(self, msg: Image):
        channels = 3
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        arr = arr[: msg.width * msg.height * channels]
        image = arr.reshape(msg.height, msg.width, channels).copy()

        with self._lock:
            detections = list(self._latest_detections)

        for _, det in detections:
            image = draw_bbox_outline(image, det.bbox_x, det.bbox_y, det.bbox_w, det.bbox_h)

        out = Image()
        out.header = msg.header
        out.width = msg.width
        out.height = msg.height
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        # array.array('B', ...) rather than raw bytes -- rclpy's `data` setter validates a bytes
        # payload byte-by-byte in pure Python (~20 ms for this 416x416 frame, vs 0.02 ms here).
        # Same mechanism as 990a99a.
        out.data = array.array('B', image.tobytes())
        self.annotated_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OakdPreviewOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
