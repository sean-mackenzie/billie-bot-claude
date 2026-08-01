#!/usr/bin/env python3
"""Online image-quality monitor for /noir/image: brightness, black/white clipping, and
repeated-frame detection, published as diagnostics for live Foxglove viewing. The heavier
CNR/sharpness analysis happens offline in analyze_noir_image.py against the recorded bag.
"""

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import Image

from billiebot_sensor_tests.common.image_hash import repeated_frame_hash


class ImageQualityMonitorNode(Node):
    def __init__(self):
        super().__init__('image_quality_monitor')
        self.declare_parameter('source_topic', '/noir/image')
        self.declare_parameter('diagnostics_topic', '/bench/noir/diagnostics')

        source_topic = str(self.get_parameter('source_topic').value)
        diagnostics_topic = str(self.get_parameter('diagnostics_topic').value)

        self._last_hash = None
        self._repeated_count = 0
        self._frame_count = 0

        self.diag_pub = self.create_publisher(DiagnosticArray, diagnostics_topic, 10)
        self.sub = self.create_subscription(Image, source_topic, self._on_frame, 10)

    def _on_frame(self, msg: Image):
        self._frame_count += 1
        channels = 3 if msg.encoding in ('rgb8', 'bgr8') else 1
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        arr = arr[:msg.width * msg.height * channels].reshape(msg.height, msg.width, channels)
        gray = arr.mean(axis=2)

        black_clip = float(np.mean(gray <= 1))
        white_clip = float(np.mean(gray >= 254))
        mean_luma = float(np.mean(gray))

        frame_hash = repeated_frame_hash(bytes(msg.data), msg.width, msg.height, channels)
        repeated = self._last_hash is not None and frame_hash == self._last_hash
        if repeated:
            self._repeated_count += 1
        self._last_hash = frame_hash

        arr_out = DiagnosticArray()
        arr_out.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'image_quality_monitor'
        status.level = DiagnosticStatus.WARN if repeated else DiagnosticStatus.OK
        status.message = 'repeated frame detected' if repeated else 'OK'
        status.values = [
            KeyValue(key='mean_luminance', value=str(mean_luma)),
            KeyValue(key='black_clipping_fraction', value=str(black_clip)),
            KeyValue(key='white_clipping_fraction', value=str(white_clip)),
            KeyValue(key='repeated_frame_count', value=str(self._repeated_count)),
            KeyValue(key='frame_count', value=str(self._frame_count)),
        ]
        arr_out.status.append(status)
        self.diag_pub.publish(arr_out)


def main(args=None):
    rclpy.init(args=args)
    node = ImageQualityMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
