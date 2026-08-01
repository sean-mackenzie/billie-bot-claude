#!/usr/bin/env python3
"""Real-mode OAK-D Lite bench acquisition: RGB, registered depth (16UC1, millimetres),
point cloud, camera info, and diagnostics -- topics the production `oakd_dog_detector`
does not publish. `depthai` is imported lazily so this module (and the package's pytest
collection) never requires DepthAI hardware/library on a dev machine.
"""

import sys

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header

try:
    from sensor_msgs_py import point_cloud2
    _HAVE_POINT_CLOUD2 = True
except ImportError:
    _HAVE_POINT_CLOUD2 = False


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

        if self.mock:
            self.get_logger().info('OAK-D bench publisher running in MOCK mode')
            self.timer = self.create_timer(1.0 / self.fps, self._mock_publish)
        else:
            self.get_logger().info('OAK-D bench publisher starting real DepthAI pipeline')
            self._init_pipeline()

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

    def _publish_point_cloud(self, depth_arr, stamp):
        if not _HAVE_POINT_CLOUD2 or self._rgb_intrinsics is None:
            return
        fx, fy = self._rgb_intrinsics[0][0], self._rgb_intrinsics[1][1]
        cx, cy = self._rgb_intrinsics[0][2], self._rgb_intrinsics[1][2]
        h, w = depth_arr.shape
        stride = max(1, self.point_cloud_stride)
        ys, xs = np.mgrid[0:h:stride, 0:w:stride]
        z = depth_arr[0:h:stride, 0:w:stride].astype(np.float32) / 1000.0  # mm -> m
        valid = z > 0
        x = (xs - cx) * z / fx
        y = (ys - cy) * z / fy
        points = np.stack([x[valid], y[valid], z[valid]], axis=-1)

        header = Header()
        header.stamp = stamp
        header.frame_id = self.camera_frame
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self.points_pub.publish(cloud_msg)

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

        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.camera_frame
        rgb_msg.width = w
        rgb_msg.height = h
        rgb_msg.encoding = 'bgr8'
        rgb_msg.is_bigendian = False
        rgb_msg.step = w * 3
        rgb_msg.data = bytes([60, 60, 60] * w * h)
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
