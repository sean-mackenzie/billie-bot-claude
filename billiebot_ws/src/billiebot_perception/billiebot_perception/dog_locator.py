#!/usr/bin/env python3
"""Dog locator node — transforms detections from camera frame to map frame.

Subscribes to /dog/detections_3d and publishes /dog/pose_map.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, TransformException
from billiebot_interfaces.msg import DogDetection3D


class DogLocator(Node):
    def __init__(self):
        super().__init__('dog_locator')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('min_confidence', 0.5)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detection_sub = self.create_subscription(
            DogDetection3D, '/dog/detections_3d', self.detection_callback, 10
        )

        self.pose_pub = self.create_publisher(PoseStamped, '/dog/pose_map', 10)
        self.found_pub = self.create_publisher(Bool, '/dog/found', 10)

        self.get_logger().info('DogLocator started')

    def detection_callback(self, msg: DogDetection3D):
        if msg.confidence < self.min_confidence:
            return

        # Create point in camera frame
        point_camera = PointStamped()
        point_camera.header = msg.header
        point_camera.point = msg.position

        try:
            # Transform to map frame
            point_map = self.tf_buffer.transform(
                point_camera, self.map_frame, timeout=rclpy.duration.Duration(seconds=0.1)
            )

            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.map_frame
            pose_msg.pose.position = point_map.point
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)

            found_msg = Bool()
            found_msg.data = True
            self.found_pub.publish(found_msg)

        except TransformException as e:
            self.get_logger().debug(f'TF lookup failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DogLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
