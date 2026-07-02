#!/usr/bin/env python3
"""OAK-D Lite spatial dog detector node.

Real mode: runs depthai pipeline with YOLOv8n for spatial object detection.
Mock mode: publishes synthetic dog detections at 5 Hz for testing.
"""

import math
import random

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from billiebot_interfaces.msg import DogDetection3D


class OakdDogDetector(Node):
    def __init__(self):
        super().__init__('oakd_dog_detector')

        self.declare_parameter('mock', False)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_frame', 'oakd_link_optical')
        self.declare_parameter('publish_rate_hz', 5.0)

        self.mock = bool(self.get_parameter('mock').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.detection_pub = self.create_publisher(
            DogDetection3D, '/dog/detections_3d', 10
        )
        self.found_pub = self.create_publisher(Bool, '/dog/found', 10)

        if self.mock:
            self.get_logger().info('OAK-D detector running in MOCK mode')
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.mock_detect
            )
        else:
            self.get_logger().info('OAK-D detector starting real pipeline')
            self.init_depthai_pipeline()

    def init_depthai_pipeline(self):
        """Initialize DepthAI pipeline for spatial YOLOv8n detection."""
        try:
            import depthai as dai

            pipeline = dai.Pipeline()

            # RGB camera
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(416, 416)
            cam_rgb.setResolution(
                dai.ColorCameraProperties.SensorResolution.THE_1080_P
            )
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

            # Stereo depth
            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            stereo = pipeline.create(dai.node.StereoDepth)

            mono_left.setResolution(
                dai.MonoCameraProperties.SensorResolution.THE_400_P
            )
            mono_left.setCamera("left")
            mono_right.setResolution(
                dai.MonoCameraProperties.SensorResolution.THE_400_P
            )
            mono_right.setCamera("right")

            stereo.setDefaultProfilePreset(
                dai.node.StereoDepth.PresetMode.HIGH_DENSITY
            )
            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)

            # Spatial detection network
            model_path = str(self.get_parameter('model_path').value)
            if model_path:
                spatial_nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
                spatial_nn.setBlobPath(model_path)
                spatial_nn.setConfidenceThreshold(self.confidence_threshold)
                spatial_nn.input.setBlocking(False)
                spatial_nn.setBoundingBoxScaleFactor(0.5)
                spatial_nn.setDepthLowerThreshold(100)
                spatial_nn.setDepthUpperThreshold(5000)

                cam_rgb.preview.link(spatial_nn.input)
                stereo.depth.link(spatial_nn.inputDepth)

                xout_nn = pipeline.create(dai.node.XLinkOut)
                xout_nn.setStreamName("detections")
                spatial_nn.out.link(xout_nn.input)

                self._device = dai.Device(pipeline)
                self._det_queue = self._device.getOutputQueue(
                    name="detections", maxSize=4, blocking=False
                )
                self.timer = self.create_timer(
                    1.0 / self.publish_rate, self.real_detect
                )
            else:
                self.get_logger().error('No model_path specified for OAK-D detector')

        except ImportError:
            self.get_logger().error(
                'depthai not installed — run with mock:=true or install depthai'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to init OAK-D pipeline: {e}')

    def real_detect(self):
        """Process real OAK-D detections."""
        detections = self._det_queue.get()
        dog_found = False

        for det in detections.detections:
            # COCO class 16 = dog
            if det.label != 16:
                continue
            if det.confidence < self.confidence_threshold:
                continue

            dog_found = True
            msg = DogDetection3D()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.camera_frame
            msg.bbox_x = int(det.xmin)
            msg.bbox_y = int(det.ymin)
            msg.bbox_w = int(det.xmax - det.xmin)
            msg.bbox_h = int(det.ymax - det.ymin)
            msg.confidence = det.confidence
            msg.position.x = det.spatialCoordinates.x / 1000.0
            msg.position.y = det.spatialCoordinates.y / 1000.0
            msg.position.z = det.spatialCoordinates.z / 1000.0
            msg.depth = det.spatialCoordinates.z / 1000.0
            msg.label = 'dog'
            self.detection_pub.publish(msg)

        found_msg = Bool()
        found_msg.data = dog_found
        self.found_pub.publish(found_msg)

    def mock_detect(self):
        """Publish synthetic detections for testing."""
        # Simulate intermittent dog detection
        if random.random() < 0.7:  # 70% detection rate
            msg = DogDetection3D()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.camera_frame
            msg.bbox_x = 150
            msg.bbox_y = 200
            msg.bbox_w = 120
            msg.bbox_h = 80
            msg.confidence = 0.85 + random.uniform(-0.1, 0.1)
            msg.position.x = 0.05 + random.uniform(-0.1, 0.1)
            msg.position.y = -0.1 + random.uniform(-0.05, 0.05)
            msg.position.z = 2.0 + random.uniform(-0.3, 0.3)
            msg.depth = msg.position.z
            msg.label = 'dog'
            self.detection_pub.publish(msg)

            found_msg = Bool()
            found_msg.data = True
            self.found_pub.publish(found_msg)
        else:
            found_msg = Bool()
            found_msg.data = False
            self.found_pub.publish(found_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OakdDogDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
