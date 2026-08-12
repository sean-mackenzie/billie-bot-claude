#!/usr/bin/env python3
"""Real-mode OAK-D Lite bench acquisition: RGB, registered depth (16UC1, millimetres),
point cloud, camera info, and diagnostics -- topics the production `oakd_dog_detector`
does not publish. `depthai` is imported lazily so this module (and the package's pytest
collection) never requires DepthAI hardware/library on a dev machine.

Optionally also emits low-bandwidth Foxglove preview topics (`publish_previews:=true`). Those
are strictly visualization: the raw topics above remain the only authoritative source for
rosbag2, the rate monitor, and every analysis CLI. The compression happens here rather than in a
downstream node because the raw RGB frame is ~6.2 MB -- shipping it over DDS to a separate
process would cost far more than the ~2.6 ms JPEG encode it would save (see common/preview.py).
"""

import sys

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import Header

from billiebot_sensor_tests.common.preview import (
    PreviewConfig,
    RateLimiter,
    colorize_depth,
    decimate_image,
    depth_to_points,
    encode_compressed,
)

try:
    from sensor_msgs_py import point_cloud2
    _HAVE_POINT_CLOUD2 = True
except ImportError:
    _HAVE_POINT_CLOUD2 = False


def _codec_available() -> bool:
    """Pillow presence check, run once at startup so a host missing it degrades to uncompressed
    previews with a clear log line instead of raising per frame."""
    try:
        import PIL  # noqa: F401
    except ImportError:
        return False
    return True


class OakdBenchPublisher(Node):
    def __init__(self):
        super().__init__('oakd_bench_publisher')

        self.declare_parameter('mock', False)
        self.declare_parameter('camera_frame', 'oakd_link_optical')
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('lr_check', True)
        self.declare_parameter('subpixel', True)
        self.declare_parameter('extended_disparity', False)
        self.declare_parameter('depth_align', True)
        self.declare_parameter('sensor_serial', '')
        self.declare_parameter('test_mode', '')  # '' or 'flat_target' -- informational only
        self.declare_parameter('fail_on_missing_device', True)
        self.declare_parameter('point_cloud_stride', 4)

        # Visualization-only outputs, OFF by default so a bare `ros2 run` of this node keeps its
        # original topology. The bench launch files turn them on via
        # start_visualization_previews. None of these can affect the raw topics below.
        self.declare_parameter('publish_previews', False)
        self.declare_parameter('preview_width', 640)
        self.declare_parameter('preview_height', 360)
        self.declare_parameter('preview_rate_hz', 5.0)
        self.declare_parameter('preview_jpeg_quality', 70)
        self.declare_parameter('preview_format', 'jpeg')
        self.declare_parameter('depth_preview_width', 320)
        self.declare_parameter('depth_preview_height', 200)
        self.declare_parameter('depth_preview_min_m', 0.1)
        self.declare_parameter('depth_preview_max_m', 5.0)
        self.declare_parameter('points_preview_stride', 16)
        self.declare_parameter('points_preview_rate_hz', 2.0)

        self.mock = bool(self.get_parameter('mock').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.fps = float(self.get_parameter('fps').value)
        self.sensor_serial = str(self.get_parameter('sensor_serial').value)
        self.fail_on_missing_device = bool(self.get_parameter('fail_on_missing_device').value)
        self.point_cloud_stride = int(self.get_parameter('point_cloud_stride').value)

        self.rgb_pub = self.create_publisher(Image, '/bench/oakd/rgb/image_raw', 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, '/bench/oakd/rgb/camera_info', 10)
        self.depth_pub = self.create_publisher(Image, '/bench/oakd/depth/image_raw', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/bench/oakd/depth/camera_info', 10)
        self.points_pub = self.create_publisher(PointCloud2, '/bench/oakd/points', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/bench/oakd/diagnostics', 10)

        self._rgb_intrinsics = None
        self._device = None
        self._init_previews()

        if self.mock:
            self.get_logger().info('OAK-D bench publisher running in MOCK mode')
            self.timer = self.create_timer(1.0 / self.fps, self._mock_publish)
        else:
            self.get_logger().info('OAK-D bench publisher starting real DepthAI pipeline')
            self._init_pipeline()

    def _init_previews(self):
        """Sets up the visualization-only publishers. Every attribute this touches is read only
        by `_publish_previews`, which is called *after* the raw publishes, so any failure here
        leaves acquisition completely intact."""
        self.publish_previews = bool(self.get_parameter('publish_previews').value)
        self.rgb_preview_pub = None
        self.depth_preview_pub = None
        self.points_preview_pub = None
        self._preview_warned = False
        if not self.publish_previews:
            return

        preview_format = str(self.get_parameter('preview_format').value).lower()
        if preview_format != 'raw' and not _codec_available():
            self.get_logger().error(
                'Pillow is not importable -- OAK-D previews fall back to uncompressed '
                "preview_format:='raw'. Install python3-pil to restore compressed previews."
            )
            preview_format = 'raw'

        self.rgb_preview_config = PreviewConfig(
            width=int(self.get_parameter('preview_width').value),
            height=int(self.get_parameter('preview_height').value),
            rate_hz=float(self.get_parameter('preview_rate_hz').value),
            quality=int(self.get_parameter('preview_jpeg_quality').value),
            format=preview_format,
        )
        self.depth_preview_config = PreviewConfig(
            width=int(self.get_parameter('depth_preview_width').value),
            height=int(self.get_parameter('depth_preview_height').value),
            rate_hz=float(self.get_parameter('preview_rate_hz').value),
            quality=int(self.get_parameter('preview_jpeg_quality').value),
            format=preview_format,
            min_m=float(self.get_parameter('depth_preview_min_m').value),
            max_m=float(self.get_parameter('depth_preview_max_m').value),
        )
        self.points_preview_stride = int(self.get_parameter('points_preview_stride').value)

        self._rgb_preview_limiter = RateLimiter(self.rgb_preview_config.rate_hz)
        self._depth_preview_limiter = RateLimiter(self.depth_preview_config.rate_hz)
        self._points_preview_limiter = RateLimiter(
            float(self.get_parameter('points_preview_rate_hz').value)
        )

        # Depth 1: a visualization consumer should always see the newest frame rather than a
        # queued backlog, and a slow Foxglove link must never apply back-pressure here.
        preview_msg_type = Image if preview_format == 'raw' else CompressedImage
        rgb_topic = ('/bench/oakd/rgb/preview' if preview_format == 'raw'
                     else '/bench/oakd/rgb/preview/compressed')
        depth_topic = ('/bench/oakd/depth/preview' if preview_format == 'raw'
                       else '/bench/oakd/depth/preview/compressed')
        self.rgb_preview_pub = self.create_publisher(preview_msg_type, rgb_topic, 1)
        self.depth_preview_pub = self.create_publisher(preview_msg_type, depth_topic, 1)
        self.points_preview_pub = self.create_publisher(PointCloud2, '/bench/oakd/points_preview', 1)

        self.get_logger().info(
            f'OAK-D visualization previews enabled: RGB {rgb_topic} '
            f'{self.rgb_preview_config.width}x{self.rgb_preview_config.height}, '
            f'depth {depth_topic} '
            f'{self.depth_preview_config.width}x{self.depth_preview_config.height} '
            f'({self.depth_preview_config.min_m}-{self.depth_preview_config.max_m} m), '
            f'points /bench/oakd/points_preview stride {self.points_preview_stride}. '
            'These are visualization only -- analysis always reads the raw topics.'
        )

    def _init_pipeline(self):
        try:
            import depthai as dai
        except ImportError:
            self.get_logger().error(
                'depthai not installed -- run with mock:=true or install depthai'
            )
            if self.fail_on_missing_device:
                sys.exit(1)
            return

        try:
            pipeline = dai.Pipeline()

            cam_rgb = pipeline.create(dai.node.ColorCamera)
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
            cam_rgb.setFps(self.fps)

            mono_left = pipeline.create(dai.node.MonoCamera)
            mono_right = pipeline.create(dai.node.MonoCamera)
            stereo = pipeline.create(dai.node.StereoDepth)
            mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_left.setCamera('left')
            mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            mono_right.setCamera('right')
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            stereo.setLeftRightCheck(bool(self.get_parameter('lr_check').value))
            stereo.setSubpixel(bool(self.get_parameter('subpixel').value))
            stereo.setExtendedDisparity(bool(self.get_parameter('extended_disparity').value))
            if bool(self.get_parameter('depth_align').value):
                stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            mono_left.out.link(stereo.left)
            mono_right.out.link(stereo.right)

            xout_rgb = pipeline.create(dai.node.XLinkOut)
            xout_rgb.setStreamName('rgb')
            cam_rgb.video.link(xout_rgb.input)

            xout_depth = pipeline.create(dai.node.XLinkOut)
            xout_depth.setStreamName('depth')
            stereo.depth.link(xout_depth.input)

            device_info = dai.DeviceInfo(self.sensor_serial) if self.sensor_serial else None
            self._device = dai.Device(pipeline, device_info) if device_info else dai.Device(pipeline)
            self._rgb_queue = self._device.getOutputQueue('rgb', maxSize=4, blocking=False)
            self._depth_queue = self._device.getOutputQueue('depth', maxSize=4, blocking=False)

            calib = self._device.readCalibration()
            self._rgb_intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A)

            self.timer = self.create_timer(1.0 / self.fps, self._real_publish)
            self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        except Exception as e:
            self.get_logger().error(f'Failed to init OAK-D bench pipeline: {e}')
            if self.fail_on_missing_device:
                sys.exit(1)

    def _camera_info(self, width, height, frame_id) -> CameraInfo:
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = width
        info.height = height
        if self._rgb_intrinsics is not None:
            fx, fy = self._rgb_intrinsics[0][0], self._rgb_intrinsics[1][1]
            cx, cy = self._rgb_intrinsics[0][2], self._rgb_intrinsics[1][2]
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    def _real_publish(self):
        stamp = self.get_clock().now().to_msg()
        try:
            rgb_frame = self._rgb_queue.get()
            depth_frame = self._depth_queue.get()
        except Exception as e:
            self.get_logger().warning(f'OAK-D queue read error: {e}')
            return

        rgb_cv = rgb_frame.getCvFrame()
        h, w = rgb_cv.shape[0], rgb_cv.shape[1]
        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.camera_frame
        rgb_msg.width = w
        rgb_msg.height = h
        rgb_msg.encoding = 'bgr8'
        rgb_msg.is_bigendian = False
        rgb_msg.step = w * 3
        rgb_msg.data = rgb_cv.tobytes()
        self.rgb_pub.publish(rgb_msg)
        self.rgb_info_pub.publish(self._camera_info(w, h, self.camera_frame))

        depth_arr = depth_frame.getFrame()  # uint16, millimetres, DepthAI native units
        dh, dw = depth_arr.shape[0], depth_arr.shape[1]
        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.camera_frame
        depth_msg.width = dw
        depth_msg.height = dh
        depth_msg.encoding = '16UC1'
        depth_msg.is_bigendian = False
        depth_msg.step = dw * 2
        depth_msg.data = depth_arr.astype(np.uint16).tobytes()
        self.depth_pub.publish(depth_msg)
        self.depth_info_pub.publish(self._camera_info(dw, dh, self.camera_frame))

        self._publish_point_cloud(depth_arr, stamp)

        # Strictly after every authoritative publish above, so a preview fault can never delay or
        # suppress raw data.
        self._publish_previews(rgb_cv, depth_arr, stamp)

    def _publish_point_cloud(self, depth_arr, stamp):
        if not _HAVE_POINT_CLOUD2 or self._rgb_intrinsics is None:
            return
        points = self._depth_to_points(depth_arr, max(1, self.point_cloud_stride))
        self.points_pub.publish(self._cloud_msg(points, stamp))

    def _depth_to_points(self, depth_arr, stride):
        fx, fy = self._rgb_intrinsics[0][0], self._rgb_intrinsics[1][1]
        cx, cy = self._rgb_intrinsics[0][2], self._rgb_intrinsics[1][2]
        return depth_to_points(depth_arr, fx, fy, cx, cy, stride)

    def _cloud_msg(self, points, stamp) -> PointCloud2:
        header = Header()
        header.stamp = stamp
        header.frame_id = self.camera_frame
        return point_cloud2.create_cloud_xyz32(header, points.tolist())

    def _publish_previews(self, rgb_cv, depth_arr, stamp):
        """Visualization-only republish of data already published raw. Wrapped whole: a preview
        exception is logged once and then swallowed, because losing a Foxglove panel must never
        cost the run a frame of authoritative data."""
        if not self.publish_previews:
            return
        now_s = stamp.sec + stamp.nanosec / 1e9
        try:
            if rgb_cv is not None and self._rgb_preview_limiter.should_emit(now_s):
                small = decimate_image(
                    rgb_cv, self.rgb_preview_config.width, self.rgb_preview_config.height
                )
                self.rgb_preview_pub.publish(
                    self._preview_msg(small, self.rgb_preview_config, stamp, source_order='bgr')
                )

            if depth_arr is not None and self._depth_preview_limiter.should_emit(now_s):
                small_depth = decimate_image(
                    depth_arr, self.depth_preview_config.width, self.depth_preview_config.height
                )
                colorized = colorize_depth(
                    small_depth,
                    self.depth_preview_config.min_m,
                    self.depth_preview_config.max_m,
                )
                self.depth_preview_pub.publish(
                    self._preview_msg(
                        colorized, self.depth_preview_config, stamp, source_order='rgb'
                    )
                )

            if (depth_arr is not None and _HAVE_POINT_CLOUD2 and self._rgb_intrinsics is not None
                    and self._points_preview_limiter.should_emit(now_s)):
                preview_points = self._depth_to_points(depth_arr, self.points_preview_stride)
                self.points_preview_pub.publish(self._cloud_msg(preview_points, stamp))

        except Exception as e:
            if not self._preview_warned:
                self._preview_warned = True
                self.get_logger().warning(
                    f'OAK-D preview publishing failed and is now silent: {e} '
                    '(raw acquisition is unaffected)'
                )

    def _preview_msg(self, rgb, config, stamp, source_order):
        if config.format == 'raw':
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.camera_frame
            msg.height, msg.width = rgb.shape[0], rgb.shape[1]
            msg.encoding = 'bgr8' if source_order == 'bgr' else 'rgb8'
            msg.is_bigendian = False
            msg.step = msg.width * 3
            msg.data = np.ascontiguousarray(rgb).tobytes()
            return msg
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        msg.format = config.format
        msg.data = encode_compressed(rgb, config.format, config.quality, source_order=source_order)
        return msg

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'oakd_bench_publisher'
        status.level = DiagnosticStatus.OK
        status.message = 'OK'
        values = []
        if self._device is not None:
            for key, getter in (('usb_speed', 'getUsbSpeed'), ('device_name', 'getDeviceName')):
                try:
                    values.append(KeyValue(key=key, value=str(getattr(self._device, getter)())))
                except Exception:
                    pass
        status.values = values
        arr.status.append(status)
        self.diag_pub.publish(arr)

    def _mock_publish(self):
        stamp = self.get_clock().now().to_msg()
        w, h = 640, 400

        rgb_arr = np.full((h, w, 3), 60, dtype=np.uint8)
        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.camera_frame
        rgb_msg.width = w
        rgb_msg.height = h
        rgb_msg.encoding = 'bgr8'
        rgb_msg.is_bigendian = False
        rgb_msg.step = w * 3
        rgb_msg.data = rgb_arr.tobytes()
        self.rgb_pub.publish(rgb_msg)

        depth_arr = np.full((h, w), 2000, dtype=np.uint16)  # flat mock plane at 2.0 m
        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.camera_frame
        depth_msg.width = w
        depth_msg.height = h
        depth_msg.encoding = '16UC1'
        depth_msg.is_bigendian = False
        depth_msg.step = w * 2
        depth_msg.data = depth_arr.tobytes()
        self.depth_pub.publish(depth_msg)

        # Mock mode exercises the same preview path as real mode so the previews-on/previews-off
        # comparison is meaningful without hardware. The points preview stays silent here for the
        # same reason the authoritative cloud does: mock has no device calibration.
        self._publish_previews(rgb_arr, depth_arr, stamp)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = OakdBenchPublisher()
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
