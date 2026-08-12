#!/usr/bin/env python3
"""Test-only colorizer: /thermal/image (32FC1, raw degrees Celsius) -> RGB visualization.

The production thermal_node is never modified -- see plan section 5b. Pure numpy
hand-rolled colormap, no cv2/matplotlib dependency at node runtime (matplotlib is only
used offline by analysis CLIs for saved plots). Raw float data is never touched by this
node -- it only reads /thermal/image, never republishes over it.

The colormap itself lives in common/preview.py, shared with the OAK-D depth preview so the
bench has one colorization implementation rather than a per-sensor copy. A 32x24 thermal frame
is ~3 kB, so unlike the OAK-D streams this output needs no compression to be Foxglove-safe.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from billiebot_sensor_tests.common.preview import blue_green_red_colormap


class ThermalColorizerNode(Node):
    def __init__(self):
        super().__init__('thermal_colorizer')
        self.declare_parameter('min_display_c', 18.0)
        self.declare_parameter('max_display_c', 40.0)
        self.declare_parameter('publish_normalized', True)
        self.declare_parameter('source_topic', '/thermal/image')

        self.min_c = float(self.get_parameter('min_display_c').value)
        self.max_c = float(self.get_parameter('max_display_c').value)
        self.publish_normalized = bool(self.get_parameter('publish_normalized').value)
        source_topic = str(self.get_parameter('source_topic').value)

        self.color_pub = self.create_publisher(Image, '/bench/thermal/image_color', 10)
        self.norm_pub = (
            self.create_publisher(Image, '/bench/thermal/image_normalized', 10)
            if self.publish_normalized else None
        )
        self.sub = self.create_subscription(Image, source_topic, self._on_frame, 10)

    def _on_frame(self, msg: Image):
        frame = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)
        span = max(self.max_c - self.min_c, 1e-6)
        normalized = (frame - self.min_c) / span
        normalized = np.nan_to_num(normalized, nan=0.0, posinf=1.0, neginf=0.0)

        rgb = blue_green_red_colormap(normalized)
        color_msg = Image()
        color_msg.header = msg.header
        color_msg.width = msg.width
        color_msg.height = msg.height
        color_msg.encoding = 'rgb8'
        color_msg.is_bigendian = False
        color_msg.step = msg.width * 3
        color_msg.data = rgb.tobytes()
        self.color_pub.publish(color_msg)

        if self.norm_pub is not None:
            mono = (np.clip(normalized, 0.0, 1.0) * 255).astype(np.uint8)
            norm_msg = Image()
            norm_msg.header = msg.header
            norm_msg.width = msg.width
            norm_msg.height = msg.height
            norm_msg.encoding = 'mono8'
            norm_msg.is_bigendian = False
            norm_msg.step = msg.width
            norm_msg.data = mono.tobytes()
            self.norm_pub.publish(norm_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalColorizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
