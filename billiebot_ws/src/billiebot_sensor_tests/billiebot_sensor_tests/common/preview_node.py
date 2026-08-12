#!/usr/bin/env python3
"""Generic bench-side visualization node: subscribes to raw/large image topics and republishes
low-bandwidth `sensor_msgs/CompressedImage` previews for Foxglove.

Sensor-agnostic on purpose. It is configured entirely through two JSON parameters (the same
pattern `common/rate_monitor.py` uses for `topics_config_json`), so a future thermal/NoIR bench
launch can reuse it without a line of new code.

It never publishes over a source topic, never subscribes on behalf of the analysis path, and its
outputs are never recorded to the bag -- see `common/preview.py` for the authoritative-vs-
visualization rule this implements.

Used by DT-OAK-01, where the sources (`/oak/rgb/preview`, `/oak/rgb/annotated`,
`/oak/depth/preview`) come from the *production* detector and the bench-side overlay, so no
production node needs a new parameter. UT-OAK-01/02 instead compress inside
`oakd/oakd_bench_publisher.py`, because there the source frame is 6.2 MB and a subscriber hop
would cost more than the compression saves.
"""

import array
import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from billiebot_sensor_tests.common.preview import (
    PreviewConfig,
    PreviewConfigError,
    RateLimiter,
    colorize_depth,
    decimate_image,
    encode_compressed,
)

_COLOR_CHANNELS = {'rgb8': 3, 'bgr8': 3}
_DEPTH_DTYPES = {'16UC1': np.uint16, 'mono16': np.uint16}


class _PreviewStream:
    """One source topic -> one preview topic.

    Per-stream JSON keys win over the node-level `default_*` parameters, so a launch file can
    pass the shared knobs as ordinary typed launch arguments and only spell out in JSON what
    genuinely differs between streams (topic names and, where relevant, size).
    """

    def __init__(self, node: Node, spec: dict, defaults: dict, is_depth: bool):
        self.node = node
        self.is_depth = is_depth
        self.source_topic = spec['in']
        self.out_topic = spec['out']
        size_key = 'depth' if is_depth else 'image'
        self.config = PreviewConfig(
            width=int(spec.get('width', defaults[f'{size_key}_width'])),
            height=int(spec.get('height', defaults[f'{size_key}_height'])),
            rate_hz=float(spec.get('rate_hz', defaults['rate_hz'])),
            quality=int(spec.get('quality', defaults['quality'])),
            format=str(spec.get('format', defaults['format'])).lower(),
            min_m=float(spec.get('min_m', defaults['depth_min_m'])),
            max_m=float(spec.get('max_m', defaults['depth_max_m'])),
        )
        self.limiter = RateLimiter(self.config.rate_hz)
        self._warned = False

        if self.config.format == 'raw':
            self.publisher = node.create_publisher(Image, self.out_topic, 1)
        else:
            self.publisher = node.create_publisher(CompressedImage, self.out_topic, 1)
        # Depth 1 on both ends: a visualization stream should drop stale frames rather than queue
        # them, so a slow link never builds a backlog that lags behind the live scene.
        self.subscription = node.create_subscription(Image, self.source_topic, self._on_image, 1)

    def _on_image(self, msg: Image):
        now_s = self.node.get_clock().now().nanoseconds / 1e9
        if not self.limiter.should_emit(now_s):
            return
        try:
            self._publish_preview(msg)
        except Exception as e:  # visualization must never take the bench down
            if not self._warned:
                self._warned = True
                self.node.get_logger().warning(
                    f'preview for {self.source_topic} failed and is now silent: {e}'
                )

    def _publish_preview(self, msg: Image):
        if self.is_depth:
            dtype = _DEPTH_DTYPES.get(msg.encoding)
            if dtype is None:
                raise ValueError(f"depth source '{self.source_topic}' has encoding '{msg.encoding}'")
            frame = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.width)
            small = decimate_image(frame, self.config.width, self.config.height)
            rgb = colorize_depth(small, self.config.min_m, self.config.max_m)
            source_order = 'rgb'
        else:
            channels = _COLOR_CHANNELS.get(msg.encoding)
            if channels is None:
                raise ValueError(f"image source '{self.source_topic}' has encoding '{msg.encoding}'")
            frame = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = frame[: msg.height * msg.width * channels]
            frame = frame.reshape(msg.height, msg.width, channels)
            rgb = decimate_image(frame, self.config.width, self.config.height)
            source_order = 'bgr' if msg.encoding == 'bgr8' else 'rgb'

        # array.array rather than bytes: rclpy converts a bytes payload element-by-element, which
        # costs ~1900x more than handing it a typed array (26.6 ms vs 0.01 ms for a 691 kB frame).
        # Same defect class as 990a99a.
        if self.config.format == 'raw':
            out = Image()
            out.header = msg.header
            out.height, out.width = rgb.shape[0], rgb.shape[1]
            out.encoding = 'rgb8' if (self.is_depth or source_order == 'rgb') else 'bgr8'
            out.is_bigendian = False
            out.step = out.width * 3
            out.data = array.array('B', np.ascontiguousarray(rgb).tobytes())
        else:
            out = CompressedImage()
            out.header = msg.header
            out.format = self.config.format
            out.data = array.array('B', encode_compressed(
                rgb, self.config.format, self.config.quality, source_order=source_order
            ))
        self.publisher.publish(out)


class BenchPreviewNode(Node):
    def __init__(self):
        super().__init__('bench_preview_node')
        self.declare_parameter('image_sources_json', '[]')
        self.declare_parameter('depth_sources_json', '[]')
        self.declare_parameter('default_image_width', 640)
        self.declare_parameter('default_image_height', 360)
        self.declare_parameter('default_depth_width', 320)
        self.declare_parameter('default_depth_height', 200)
        self.declare_parameter('default_rate_hz', 5.0)
        self.declare_parameter('default_quality', 70)
        self.declare_parameter('default_format', 'jpeg')
        self.declare_parameter('default_depth_min_m', 0.1)
        self.declare_parameter('default_depth_max_m', 5.0)

        defaults = {
            'image_width': int(self.get_parameter('default_image_width').value),
            'image_height': int(self.get_parameter('default_image_height').value),
            'depth_width': int(self.get_parameter('default_depth_width').value),
            'depth_height': int(self.get_parameter('default_depth_height').value),
            'rate_hz': float(self.get_parameter('default_rate_hz').value),
            'quality': int(self.get_parameter('default_quality').value),
            'format': str(self.get_parameter('default_format').value).lower(),
            'depth_min_m': float(self.get_parameter('default_depth_min_m').value),
            'depth_max_m': float(self.get_parameter('default_depth_max_m').value),
        }

        image_specs = self._parse('image_sources_json')
        depth_specs = self._parse('depth_sources_json')

        if defaults['format'] != 'raw' and not self._codec_available():
            self.get_logger().error(
                'Pillow is not importable -- preview streams are falling back to uncompressed '
                "format:='raw'. Install python3-pil on this host to restore compressed previews."
            )
            defaults['format'] = 'raw'
            for spec in image_specs + depth_specs:
                spec.pop('format', None)

        self.streams = [_PreviewStream(self, s, defaults, is_depth=False) for s in image_specs]
        self.streams += [_PreviewStream(self, s, defaults, is_depth=True) for s in depth_specs]

        if not self.streams:
            self.get_logger().warning('bench_preview_node started with no configured sources')
        for stream in self.streams:
            self.get_logger().info(
                f'preview {stream.source_topic} -> {stream.out_topic} '
                f'({stream.config.width}x{stream.config.height}, {stream.config.rate_hz} Hz, '
                f'{stream.config.format} q{stream.config.quality})'
            )

    def _parse(self, param_name: str) -> list:
        raw = str(self.get_parameter(param_name).value)
        try:
            specs = json.loads(raw) if raw else []
        except json.JSONDecodeError as e:
            raise PreviewConfigError(f"{param_name} is not valid JSON: {e}") from e
        if not isinstance(specs, list):
            raise PreviewConfigError(f'{param_name} must be a JSON list, got {type(specs).__name__}')
        for spec in specs:
            if not isinstance(spec, dict) or 'in' not in spec or 'out' not in spec:
                raise PreviewConfigError(f"{param_name} entries need 'in' and 'out' keys: {spec}")
        return specs

    @staticmethod
    def _codec_available() -> bool:
        try:
            import PIL  # noqa: F401
        except ImportError:
            return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = BenchPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
