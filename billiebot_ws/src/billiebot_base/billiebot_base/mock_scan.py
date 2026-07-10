#!/usr/bin/env python3
"""Mock lidar — publishes a synthetic /scan for hardware-free testing.

Simulates an RPLidar A1 seeing a rectangular room, so AMCL receives scans
and publishes the map->odom transform (required for Nav2 costmap activation
in mock mode).
"""

import math
import random

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

NUM_SAMPLES = 360
ROOM_HALF_X = 2.5  # metres to the walls of the fake room
ROOM_HALF_Y = 2.0


class MockScan(Node):
    def __init__(self):
        super().__init__('mock_scan')

        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('rate_hz', 10.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_scan)

        self.get_logger().info('Mock scan publisher started (MOCK)')

    def publish_scan(self) -> None:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / NUM_SAMPLES
        scan.time_increment = 0.0
        scan.scan_time = float(self.timer.timer_period_ns) / 1e9
        scan.range_min = 0.15
        scan.range_max = 12.0

        ranges = []
        for i in range(NUM_SAMPLES):
            angle = scan.angle_min + i * scan.angle_increment
            # Ray-cast from the room centre to an axis-aligned rectangle.
            tx = ROOM_HALF_X / abs(math.cos(angle)) if math.cos(angle) else float('inf')
            ty = ROOM_HALF_Y / abs(math.sin(angle)) if math.sin(angle) else float('inf')
            ranges.append(min(tx, ty) + random.gauss(0.0, 0.01))
        scan.ranges = ranges

        self.scan_pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
