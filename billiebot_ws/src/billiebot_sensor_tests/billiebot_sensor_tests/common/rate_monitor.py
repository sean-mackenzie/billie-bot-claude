"""ROS node: subscribes to N configured topics, tracks rate/gap/monotonicity/repeated-frame
stats per topic, publishes /bench/status (diagnostic_msgs/DiagnosticArray) at 1Hz, writes a
periodic JSON snapshot, and (when run standalone via `ros2 run`) exits nonzero if a required
topic never appears or violates its configured threshold.
"""

import json
import sys
import threading
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from billiebot_sensor_tests.common.image_hash import repeated_frame_hash
from billiebot_sensor_tests.common.rate_stats import compute_rate_stats

_IMAGE_CHANNELS = {
    'rgb8': 3, 'bgr8': 3, 'mono8': 1, '8UC1': 1, 'mono16': 1, '16UC1': 1, '32FC1': 1,
}


class _TopicTracker:
    def __init__(self, name, msg_type_str, required, min_rate_hz=None, hash_enabled=False):
        self.name = name
        self.msg_type_str = msg_type_str
        self.required = required
        self.min_rate_hz = min_rate_hz
        self.hash_enabled = hash_enabled
        self.receipt_timestamps_ns = []
        self.header_timestamps_ns = []
        self.repeated_frame_count = 0
        self._last_hash = None
        self._lock = threading.Lock()

    def record(self, receipt_ns, header_ns=None, image_msg=None):
        with self._lock:
            self.receipt_timestamps_ns.append(receipt_ns)
            if header_ns is not None:
                self.header_timestamps_ns.append(header_ns)
            if self.hash_enabled and image_msg is not None:
                try:
                    channels = _IMAGE_CHANNELS.get(image_msg.encoding, 1)
                    h = repeated_frame_hash(
                        bytes(image_msg.data), image_msg.width, image_msg.height, channels
                    )
                    if self._last_hash is not None and h == self._last_hash:
                        self.repeated_frame_count += 1
                    self._last_hash = h
                except (ValueError, AttributeError):
                    pass

    def snapshot(self) -> dict:
        with self._lock:
            receipt_stats = compute_rate_stats(list(self.receipt_timestamps_ns))
            header_stats = (
                compute_rate_stats(list(self.header_timestamps_ns))
                if self.header_timestamps_ns else None
            )
            return {
                'topic': self.name,
                'type': self.msg_type_str,
                'required': self.required,
                'min_rate_hz': self.min_rate_hz,
                'receipt_rate': receipt_stats.to_dict(),
                'header_rate': header_stats.to_dict() if header_stats else None,
                'repeated_frame_count': self.repeated_frame_count,
            }

    def passes(self) -> bool:
        with self._lock:
            stats = compute_rate_stats(list(self.receipt_timestamps_ns))
        if self.required and stats.count == 0:
            return False
        if stats.count == 0:
            return True
        if self.min_rate_hz is not None and stats.count > 1 and stats.mean_hz < self.min_rate_hz:
            return False
        if stats.count > 1 and not stats.monotonic:
            return False
        return True


class TopicRateMonitorNode(Node):
    def __init__(self):
        super().__init__('topic_rate_monitor')
        self.declare_parameter('topics_config_json', '[]')
        self.declare_parameter('output_path', '')
        self.declare_parameter('status_topic', '/bench/status')
        self.declare_parameter('status_hz', 1.0)
        self.declare_parameter('write_interval_sec', 5.0)

        topics_config = json.loads(str(self.get_parameter('topics_config_json').value))
        self._output_path = Path(str(self.get_parameter('output_path').value))
        status_topic = str(self.get_parameter('status_topic').value)
        status_hz = float(self.get_parameter('status_hz').value)
        write_interval = float(self.get_parameter('write_interval_sec').value)

        self._trackers = {}
        self._subs = []
        for cfg in topics_config:
            tracker = _TopicTracker(
                cfg['name'], cfg['type'], cfg.get('required', True),
                cfg.get('min_rate_hz'), cfg.get('hash_enabled', False),
            )
            self._trackers[cfg['name']] = tracker
            try:
                msg_cls = get_message(cfg['type'])
            except (AttributeError, ValueError, ModuleNotFoundError) as e:
                self.get_logger().error(f"cannot resolve message type '{cfg['type']}': {e}")
                continue
            sub = self.create_subscription(
                msg_cls, cfg['name'], self._make_callback(tracker), 10,
            )
            self._subs.append(sub)

        self._status_pub = self.create_publisher(DiagnosticArray, status_topic, 10)
        self._status_timer = self.create_timer(1.0 / status_hz, self._publish_status)
        self._write_timer = self.create_timer(write_interval, self._write_output)

    def _make_callback(self, tracker: _TopicTracker):
        def _cb(msg):
            receipt_ns = self.get_clock().now().nanoseconds
            header_ns = None
            if hasattr(msg, 'header'):
                header_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            image_msg = msg if hasattr(msg, 'data') and hasattr(msg, 'encoding') else None
            tracker.record(receipt_ns, header_ns, image_msg)
        return _cb

    def _publish_status(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for tracker in self._trackers.values():
            snap = tracker.snapshot()
            status = DiagnosticStatus()
            status.name = f'bench_rate_monitor: {tracker.name}'
            ok = tracker.passes()
            status.level = DiagnosticStatus.OK if ok else DiagnosticStatus.ERROR
            status.message = 'OK' if ok else 'threshold violated or no data'
            status.values = [
                KeyValue(key='count', value=str(snap['receipt_rate']['count'])),
                KeyValue(key='mean_hz', value=str(snap['receipt_rate']['mean_hz'])),
                KeyValue(key='max_gap_s', value=str(snap['receipt_rate']['max_gap_s'])),
                KeyValue(key='monotonic', value=str(snap['receipt_rate']['monotonic'])),
            ]
            arr.status.append(status)
        self._status_pub.publish(arr)

    def _write_output(self):
        if not self._output_path:
            return
        result = {
            'topics': {name: t.snapshot() for name, t in self._trackers.items()},
            'pass': self.all_pass(),
        }
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self._output_path, 'w') as f:
            json.dump(result, f, indent=2)

    def all_pass(self) -> bool:
        return all(t.passes() for t in self._trackers.values())


def main(args=None):
    rclpy.init(args=args)
    node = TopicRateMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write_output()
        exit_code = 0 if node.all_pass() else 1
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
