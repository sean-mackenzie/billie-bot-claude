#!/usr/bin/env python3
"""Pi Camera 3 NoIR node for low-light dog detection.

Real mode: captures via libcamera/picamera2.
Mock mode: publishes synthetic test images.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

_AF_MODE_MAP = {'manual': 0, 'auto': 1, 'continuous': 2}


def build_camera_config(width: int, height: int, af_mode: str = '', af_trigger: bool = False,
                         lens_position: float = -1.0, exposure_time_us: int = -1,
                         analogue_gain: float = -1.0, frame_duration_limits_us=None) -> dict:
    """Builds the kwargs for Picamera2.create_still_configuration(). Every sentinel
    ('' / False / -1 / None) means "don't touch". When every argument is at its sentinel
    this returns exactly today's hardcoded call, byte-for-byte (no 'controls' key at all
    -- not even an empty one), so real-mode capture behavior is unchanged unless a bench
    launch explicitly sets one of these (plan section 5c)."""
    cfg = {"main": {"size": (width, height), "format": "RGB888"}}
    controls = {}
    if af_mode and af_mode.lower() in _AF_MODE_MAP:
        controls['AfMode'] = _AF_MODE_MAP[af_mode.lower()]
    if af_trigger:
        controls['AfTrigger'] = 0  # picamera2 AfTriggerEnum.Start
    if lens_position >= 0:
        controls['LensPosition'] = lens_position
    if exposure_time_us >= 0:
        controls['ExposureTime'] = exposure_time_us
    if analogue_gain >= 0:
        controls['AnalogueGain'] = analogue_gain
    if frame_duration_limits_us is not None:
        controls['FrameDurationLimits'] = frame_duration_limits_us
    if controls:
        cfg['controls'] = controls
    return cfg


class NoirCamNode(Node):
    def __init__(self):
        super().__init__('noir_cam_node')

        self.declare_parameter('mock', False)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('camera_frame', 'noir_link_optical')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        # Bench-only optional controls, all sentinel-default ('' / False / -1) -- deployment
        # behavior/config is unchanged unless a bench launch explicitly sets one of these.
        self.declare_parameter('af_mode', '')
        self.declare_parameter('af_trigger', False)
        self.declare_parameter('lens_position', -1.0)
        self.declare_parameter('exposure_time_us', -1)
        self.declare_parameter('analogue_gain', -1.0)
        self.declare_parameter('frame_duration_us', -1)
        self.declare_parameter('publish_metadata', False)

        self.mock = bool(self.get_parameter('mock').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.af_mode = str(self.get_parameter('af_mode').value)
        self.af_trigger = bool(self.get_parameter('af_trigger').value)
        self.lens_position = float(self.get_parameter('lens_position').value)
        self.exposure_time_us = int(self.get_parameter('exposure_time_us').value)
        self.analogue_gain = float(self.get_parameter('analogue_gain').value)
        self.frame_duration_us = int(self.get_parameter('frame_duration_us').value)
        self.publish_metadata = bool(self.get_parameter('publish_metadata').value)

        self.image_pub = self.create_publisher(Image, '/noir/image', 10)
        self.metadata_pub = None
        if self.publish_metadata:
            from diagnostic_msgs.msg import DiagnosticArray
            self.metadata_pub = self.create_publisher(
                DiagnosticArray, '/bench/noir/diagnostics', 10
            )

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
            frame_duration_limits_us = (
                (self.frame_duration_us, self.frame_duration_us)
                if self.frame_duration_us >= 0 else None
            )
            cfg_kwargs = build_camera_config(
                self.width, self.height, self.af_mode, self.af_trigger,
                self.lens_position, self.exposure_time_us, self.analogue_gain,
                frame_duration_limits_us,
            )
            config = self._picam.create_still_configuration(**cfg_kwargs)
            self._picam.configure(config)
            self._picam.start()
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.real_publish
            )
            if self.publish_metadata:
                self.metadata_timer = self.create_timer(1.0, self._publish_metadata)
        except ImportError:
            self.get_logger().error(
                'picamera2 not installed — run with mock:=true'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to init NoIR camera: {e}')

    def _publish_metadata(self):
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        try:
            metadata = self._picam.capture_metadata()
        except Exception as e:
            self.get_logger().warning(f'Failed to capture NoIR metadata: {e}')
            return
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'noir_cam_node'
        status.level = DiagnosticStatus.OK
        status.message = 'OK'
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in metadata.items()]
        arr.status.append(status)
        self.metadata_pub.publish(arr)

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
