#!/usr/bin/env python3
"""Pi Camera 3 NoIR node for low-light dog detection.

Real mode: captures via libcamera/picamera2.
Mock mode: publishes synthetic test images.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class NoirCamNode(Node):
    def __init__(self):
        super().__init__('noir_cam_node')

        self.declare_parameter('mock', False)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('camera_frame', 'noir_link_optical')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        self.mock = bool(self.get_parameter('mock').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)

        self.image_pub = self.create_publisher(Image, '/noir/image', 10)

        if self.mock:
            self.get_logger().info('NoIR camera running in MOCK mode')
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.mock_publish
            )
        else:
            self.get_logger().info('NoIR camera starting real capture')
            self.init_camera()

    def init_camera(self):
        """Initialize Pi Camera 3 NoIR via picamera2."""
        try:
            from picamera2 import Picamera2

            self._picam = Picamera2()
            config = self._picam.create_still_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            self._picam.configure(config)
            self._picam.start()
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.real_publish
            )
        except ImportError:
            self.get_logger().error(
                'picamera2 not installed — run with mock:=true'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to init NoIR camera: {e}')

    def real_publish(self):
        """Capture and publish a frame."""
        try:
            frame = self._picam.capture_array()
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.camera_frame
            msg.width = self.width
            msg.height = self.height
            msg.encoding = 'rgb8'
            msg.is_bigendian = False
            msg.step = self.width * 3
            msg.data = frame.tobytes()
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().warning(f'NoIR capture error: {e}')

    def mock_publish(self):
        """Publish a synthetic dark-grey test image."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.width = self.width
        msg.height = self.height
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = self.width * 3
        # Dark grey frame to simulate low-light
        msg.data = bytes([40, 40, 40] * self.width * self.height)
        self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NoirCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
