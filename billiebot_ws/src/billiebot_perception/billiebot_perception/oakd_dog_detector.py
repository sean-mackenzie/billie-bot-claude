#!/usr/bin/env python3
"""OAK-D Lite spatial dog detector node.

Real mode: runs depthai pipeline with YOLOv8n for spatial object detection.
Mock mode: publishes synthetic dog detections at 5 Hz for testing.
"""

import array
import math
import os
import random
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from billiebot_interfaces.msg import DogDetection3D

# DepthAI's YoloSpatialDetectionNetwork reports xmin/ymin/xmax/ymax as normalized [0,1]
# floats, not pixel coordinates -- see scale_bbox() below (Appendix B-11).
PREVIEW_SIZE_PX = 416


def scale_bbox(xmin: float, ymin: float, xmax: float, ymax: float,
                frame_w: int = PREVIEW_SIZE_PX, frame_h: int = PREVIEW_SIZE_PX):
    """Scales DepthAI's normalized [0,1] bbox coordinates to pixel space and clamps to the
    frame bounds. This is a correctness fix, not a visualization feature: the previous
    `int(det.xmin)` etc. truncated every real bbox to ~0 because xmin/xmax are always in
    [0,1] (Appendix B-11, BRINGUP_LADDER_ANALYSIS.md). Returns (x, y, w, h) as ints."""
    x0 = min(max(round(xmin * frame_w), 0), frame_w)
    y0 = min(max(round(ymin * frame_h), 0), frame_h)
    x1 = min(max(round(xmax * frame_w), 0), frame_w)
    y1 = min(max(round(ymax * frame_h), 0), frame_h)
    return x0, y0, max(x1 - x0, 0), max(y1 - y0, 0)


class OakdDogDetector(Node):
    def __init__(self):
        super().__init__('oakd_dog_detector')

        self.declare_parameter('mock', False)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_frame', 'oakd_link_optical')
        self.declare_parameter('publish_rate_hz', 5.0)
        # Bench-only outputs, all OFF by default -- deployment behavior/topology is
        # unchanged unless a bench launch explicitly enables one of these.
        self.declare_parameter('publish_preview', False)
        self.declare_parameter('publish_depth_preview', False)
        self.declare_parameter('publish_diagnostics', False)

        self.mock = bool(self.get_parameter('mock').value)
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.publish_preview = bool(self.get_parameter('publish_preview').value)
        self.publish_depth_preview = bool(self.get_parameter('publish_depth_preview').value)
        self.publish_diagnostics = bool(self.get_parameter('publish_diagnostics').value)

        self.detection_pub = self.create_publisher(
            DogDetection3D, '/dog/detections_3d', 10
        )
        self.found_pub = self.create_publisher(Bool, '/dog/found', 10)

        self._preview_w = PREVIEW_SIZE_PX
        self._preview_h = PREVIEW_SIZE_PX
        self._preview_queue = None
        self._depth_preview_queue = None
        self.preview_pub = None
        self.depth_preview_pub = None
        self.diag_pub = None
        if self.publish_preview:
            from sensor_msgs.msg import Image
            self.preview_pub = self.create_publisher(Image, '/oak/rgb/preview', 10)
        if self.publish_depth_preview:
            from sensor_msgs.msg import Image
            self.depth_preview_pub = self.create_publisher(Image, '/oak/depth/preview', 10)
        if self.publish_diagnostics:
            from diagnostic_msgs.msg import DiagnosticArray
            self.diag_pub = self.create_publisher(
                DiagnosticArray, '/bench/oakd_detector/diagnostics', 10
            )

        if self.mock:
            self.get_logger().info('OAK-D detector running in MOCK mode')
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.mock_detect
            )
        else:
            self.get_logger().info('OAK-D detector starting real pipeline')
            self.init_depthai_pipeline()

    def init_depthai_pipeline(self):
        """Initialize DepthAI pipeline for spatial YOLOv8n detection.

        Real mode has no fallback: a missing model, a missing library, or a
        pipeline/device failure all leave the node silently alive with no
        timer (GAP-11) unless we exit non-zero here so the launch supervisor
        surfaces the failure instead of the perception chain coming up
        "green" but blind.
        """
        model_path = str(self.get_parameter('model_path').value)
        if not model_path or not os.path.isfile(model_path):
            self.get_logger().error(
                f"OAK-D model_path '{model_path}' is empty or does not exist — "
                "real mode requires a staged YOLOv8n .blob "
                "(see INSTALLATION_AND_SETUP.md Appendix B)"
            )
            sys.exit(1)

        try:
            import depthai as dai

            pipeline = dai.Pipeline()

            # RGB camera
            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setPreviewSize(self._preview_w, self._preview_h)
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

            # Bench-only preview streams -- these XLinkOuts are only created (and add zero
            # bandwidth/topology change otherwise) when explicitly enabled. The preview
            # comes off the same cam_rgb.preview link that feeds the spatial detector, so
            # it shares the same frame sequence as the detections (multiple consumers of
            # one DepthAI output are supported natively).
            if self.publish_preview:
                xout_preview = pipeline.create(dai.node.XLinkOut)
                xout_preview.setStreamName("preview")
                cam_rgb.preview.link(xout_preview.input)

            if self.publish_depth_preview:
                xout_depth_preview = pipeline.create(dai.node.XLinkOut)
                xout_depth_preview.setStreamName("depth_preview")
                stereo.depth.link(xout_depth_preview.input)

            self._device = dai.Device(pipeline)
            self._det_queue = self._device.getOutputQueue(
                name="detections", maxSize=4, blocking=False
            )
            if self.publish_preview:
                self._preview_queue = self._device.getOutputQueue(
                    name="preview", maxSize=4, blocking=False
                )
            if self.publish_depth_preview:
                self._depth_preview_queue = self._device.getOutputQueue(
                    name="depth_preview", maxSize=4, blocking=False
                )
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.real_detect
            )
            if self.publish_diagnostics:
                self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        except ImportError:
            self.get_logger().error(
                'depthai not installed — run with mock:=true or install depthai'
            )
            sys.exit(1)
        except Exception as e:
            self.get_logger().error(f'Failed to init OAK-D pipeline: {e}')
            sys.exit(1)

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
            msg.bbox_x, msg.bbox_y, msg.bbox_w, msg.bbox_h = scale_bbox(
                det.xmin, det.ymin, det.xmax, det.ymax, self._preview_w, self._preview_h
            )
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

        if self.preview_pub is not None and self._preview_queue is not None:
            self._publish_preview_frame(self.preview_pub, self._preview_queue, 'bgr8')
        if self.depth_preview_pub is not None and self._depth_preview_queue is not None:
            self._publish_preview_frame(self.depth_preview_pub, self._depth_preview_queue, 'mono16')

    def _publish_preview_frame(self, publisher, queue_, encoding):
        try:
            from sensor_msgs.msg import Image
            frame = queue_.tryGet()
            if frame is None:
                return
            cv_frame = frame.getCvFrame() if encoding == 'bgr8' else frame.getFrame()
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.camera_frame
            msg.height = cv_frame.shape[0]
            msg.width = cv_frame.shape[1]
            msg.encoding = encoding
            msg.is_bigendian = False
            bytes_per_pixel = 3 if encoding == 'bgr8' else 2
            msg.step = msg.width * bytes_per_pixel
            # array.array('B', ...) rather than the raw bytes: rclpy's generated `data` setter
            # short-circuits on an array.array and otherwise walks every byte twice in pure
            # Python to validate it. This method is called twice per detector cycle (RGB +
            # depth preview), so the bytes form costs ~40 ms of the 200 ms budget -- inside
            # real_detect(), which also publishes /dog/found on its 4.5-5.5 Hz gate. Same
            # mechanism as the fix in 990a99a.
            msg.data = array.array('B', cv_frame.tobytes())
            publisher.publish(msg)
        except Exception as e:
            self.get_logger().warning(f'Failed to publish preview frame: {e}')

    def _publish_diagnostics(self):
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'oakd_dog_detector'
        status.level = DiagnosticStatus.OK
        status.message = 'OK'
        values = []
        for key, getter in (('usb_speed', 'getUsbSpeed'), ('device_name', 'getDeviceName')):
            try:
                values.append(KeyValue(key=key, value=str(getattr(self._device, getter)())))
            except Exception:
                pass
        status.values = values
        arr.status.append(status)
        self.diag_pub.publish(arr)

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
    try:
        node = OakdDogDetector()
    except SystemExit:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
