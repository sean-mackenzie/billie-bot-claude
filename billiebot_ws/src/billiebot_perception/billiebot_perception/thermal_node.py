#!/usr/bin/env python3
"""MLX90640 thermal camera node.

Real mode: reads 32x24 thermal array via I2C.
Mock mode: publishes synthetic warm blob data.
"""

import random
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from billiebot_interfaces.msg import ThermalBlob


class ThermalNode(Node):
    def __init__(self):
        super().__init__('thermal_node')

        self.declare_parameter('mock', False)
        self.declare_parameter('publish_rate_hz', 4.0)
        self.declare_parameter('thermal_frame', 'thermal_link_optical')
        self.declare_parameter('dog_temp_min', 30.0)
        self.declare_parameter('dog_temp_max', 40.0)
        self.declare_parameter('min_blob_area', 8)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x33)

        self.mock = bool(self.get_parameter('mock').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.thermal_frame = str(self.get_parameter('thermal_frame').value)
        self.dog_temp_min = float(self.get_parameter('dog_temp_min').value)
        self.dog_temp_max = float(self.get_parameter('dog_temp_max').value)
        self.min_blob_area = int(self.get_parameter('min_blob_area').value)

        self.image_pub = self.create_publisher(Image, '/thermal/image', 10)
        self.blob_pub = self.create_publisher(ThermalBlob, '/thermal/blob', 10)

        if self.mock:
            self.get_logger().info('Thermal node running in MOCK mode')
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.mock_publish
            )
        else:
            self.get_logger().info('Thermal node starting MLX90640')
            self.init_mlx90640()

    def init_mlx90640(self):
        """Initialize MLX90640 sensor via I2C."""
        try:
            import board
            import adafruit_mlx90640

            i2c = board.I2C()
            self._mlx = adafruit_mlx90640.MLX90640(i2c)
            self._mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ
            self._frame = [0.0] * 768  # 32x24
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.real_publish
            )
        except ImportError:
            self.get_logger().error(
                'adafruit_mlx90640 not installed — run with mock:=true'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to init MLX90640: {e}')

    def real_publish(self):
        """Read thermal frame and publish."""
        try:
            self._mlx.getFrame(self._frame)
        except Exception as e:
            self.get_logger().warning(f'MLX90640 read error: {e}')
            return

        self.publish_thermal_image(self._frame)
        self.detect_warm_blob(self._frame)

    def mock_publish(self):
        """Publish synthetic thermal data."""
        # Create a 32x24 frame with ambient ~22C and a warm blob
        frame = [22.0 + random.uniform(-0.5, 0.5) for _ in range(768)]

        # Add a warm blob (dog-like) at a random position
        blob_cx, blob_cy = 16, 12
        for y in range(24):
            for x in range(32):
                dist = ((x - blob_cx) ** 2 + (y - blob_cy) ** 2) ** 0.5
                if dist < 4:
                    frame[y * 32 + x] = 35.0 + random.uniform(-1.0, 1.0)

        self.publish_thermal_image(frame)
        self.detect_warm_blob(frame)

    def publish_thermal_image(self, frame: list):
        """Publish thermal frame as 32FC1 image."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.thermal_frame
        msg.width = 32
        msg.height = 24
        msg.encoding = '32FC1'
        msg.is_bigendian = False
        msg.step = 32 * 4
        msg.data = struct.pack(f'{len(frame)}f', *frame)
        self.image_pub.publish(msg)

    def detect_warm_blob(self, frame: list):
        """Simple warm blob detection by temperature thresholding."""
        warm_pixels = []
        for y in range(24):
            for x in range(32):
                temp = frame[y * 32 + x]
                if self.dog_temp_min <= temp <= self.dog_temp_max:
                    warm_pixels.append((x, y, temp))

        if len(warm_pixels) < self.min_blob_area:
            return

        # Compute blob centroid and stats
        cx = sum(p[0] for p in warm_pixels) / len(warm_pixels)
        cy = sum(p[1] for p in warm_pixels) / len(warm_pixels)
        max_temp = max(p[2] for p in warm_pixels)
        mean_temp = sum(p[2] for p in warm_pixels) / len(warm_pixels)

        blob_msg = ThermalBlob()
        blob_msg.header.stamp = self.get_clock().now().to_msg()
        blob_msg.header.frame_id = self.thermal_frame
        blob_msg.cx = cx
        blob_msg.cy = cy
        blob_msg.area = len(warm_pixels)
        blob_msg.max_temp = max_temp
        blob_msg.mean_temp = mean_temp
        blob_msg.is_dog_candidate = len(warm_pixels) >= self.min_blob_area
        self.blob_pub.publish(blob_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
