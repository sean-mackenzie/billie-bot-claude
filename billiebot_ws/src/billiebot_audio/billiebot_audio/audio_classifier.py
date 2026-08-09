#!/usr/bin/env python3
"""YAMNet-based audio classifier with Direction of Arrival (DoA).

Real mode: streams from ReSpeaker XVF3800 USB mic array, runs YAMNet TFLite,
           queries a DoA-capable USB device for direction of arrival.
Mock mode: publishes periodic synthetic bark events for testing.

continuous_capture (default True): the original implementation blocked a 0.5 s ROS timer
on a 0.975 s sd.rec() call every tick, which cannot sustain a genuine 2 Hz processing
cadence. This refactor opens one continuous sounddevice.InputStream feeding an
AudioRingBuffer on its own thread; the ROS timer then does a near-instant ring-buffer read
followed by inference (small, bounded, fine to run synchronously). continuous_capture:=
false reproduces the original blocking-capture behavior verbatim as an explicit rollback.
"""

import os
import random
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from billiebot_audio.audio_ring_buffer import AudioRingBuffer
from billiebot_interfaces.msg import AudioEvent

# YAMNet class indices for dog-related sounds
YAMNET_DOG_CLASSES = {
    'Bark': AudioEvent.BARK,
    'Bow-wow': AudioEvent.BARK,
    'Yip': AudioEvent.BARK,
    'Howl': AudioEvent.HOWL,
    'Whimper': AudioEvent.WHINE,
    'Whimper (dog)': AudioEvent.WHINE,
    'Growling': AudioEvent.BARK,
}

# Appendix B-14 (BRINGUP_LADDER_ANALYSIS.md): this VID:PID is the older ReSpeaker
# 4-Mic/XVF3000-era array, kept only as a compatibility fallback -- see resolve_doa_device().
LEGACY_RESPEAKER_VID = 0x2886
LEGACY_RESPEAKER_PID = 0x0018


@dataclass
class UsbDeviceInfo:
    vendor_id: int
    product_id: int
    product_name: str = ''


def resolve_doa_device(candidates: List[UsbDeviceInfo],
                        product_substring: str) -> Optional[UsbDeviceInfo]:
    """Pure resolver: prefers a device whose product name matches product_substring
    (case-insensitive) -- e.g. 'XVF3800' -- and only falls back to the legacy
    0x2886:0x0018 VID:PID if no name match is found, so hardware that happened to
    enumerate under the old ID never regresses (Appendix B-14)."""
    needle = product_substring.lower() if product_substring else ''
    if needle:
        for c in candidates:
            if needle in c.product_name.lower():
                return c
    for c in candidates:
        if c.vendor_id == LEGACY_RESPEAKER_VID and c.product_id == LEGACY_RESPEAKER_PID:
            return c
    return None


class AudioClassifier(Node):
    def __init__(self):
        super().__init__('audio_classifier')

        self.declare_parameter('mock', False)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('model_path', '')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_duration_sec', 0.975)
        self.declare_parameter('device_index', -1)
        self.declare_parameter('input_channels', 2)
        self.declare_parameter('classification_channel', 1)
        self.declare_parameter('energy_threshold_db', -40.0)
        # continuous_capture default True per plan decision (confirmed with user): the old
        # blocking path is a confirmed non-functional bug, not a working baseline worth
        # preserving as the default. Set to false for an explicit rollback.
        self.declare_parameter('continuous_capture', True)
        self.declare_parameter('publish_status', True)
        self.declare_parameter('device_name_substring', '')  # '' preserves device_index
        self.declare_parameter('doa_usb_product_substring', 'XVF3800')

        self.mock = bool(self.get_parameter('mock').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.energy_threshold_db = float(
            self.get_parameter('energy_threshold_db').value
        )
        
        self.input_channels = int(
            self.get_parameter('input_channels').value
            )
        self.classification_channel = int(
            self.get_parameter('classification_channel').value
        )

        if self.input_channels < 1:
            raise ValueError('input_channels must be at least 1')

        if not 0 <= self.classification_channel < self.input_channels:
            raise ValueError(
                f'classification_channel={self.classification_channel} is invalid '
                f'for input_channels={self.input_channels}'
            )
        
        self.continuous_capture = bool(self.get_parameter('continuous_capture').value)
        self.publish_status = bool(self.get_parameter('publish_status').value)
        self.device_name_substring = str(self.get_parameter('device_name_substring').value)
        self.doa_usb_product_substring = str(
            self.get_parameter('doa_usb_product_substring').value
        )

        self.event_pub = self.create_publisher(AudioEvent, '/audio/events', 10)
        self.status_pub = None
        self._ring_buffer = None
        self._stream = None

        if self.mock:
            self.get_logger().info('Audio classifier running in MOCK mode')
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.mock_classify
            )
        else:
            self.get_logger().info('Audio classifier starting real pipeline')
            self.init_real_pipeline()

    def init_real_pipeline(self):
        """Initialize YAMNet TFLite model and audio stream.

        Real mode is fail-loud on a missing/invalid model or class map (mirrors the
        already-merged GAP-11 pattern for oakd_dog_detector): silently degrading every
        class to LOUD_NOISE when the class map can't be loaded would defeat bark/whine/
        howl detection entirely, so this is now a hard failure rather than a warning.
        """
        model_path = str(self.get_parameter('model_path').value)
        if not model_path or not os.path.isfile(model_path):
            self.get_logger().error(
                f"YAMNet model_path '{model_path}' is empty or does not exist — "
                'download yamnet.tflite from TensorFlow Hub and set it in audio.yaml'
            )
            sys.exit(1)

        class_map_path = os.path.join(os.path.dirname(model_path), 'yamnet_class_map.csv')
        if not os.path.isfile(class_map_path):
            self.get_logger().error(
                f"YAMNet class map '{class_map_path}' does not exist — "
                'it must sit next to the .tflite model'
            )
            sys.exit(1)

        try:
            try:
                import tflite_runtime.interpreter as tflite
                self._interpreter = tflite.Interpreter(model_path=model_path)
            except ImportError:
                import tensorflow as tf
                self._interpreter = tf.lite.Interpreter(model_path=model_path)

            self._interpreter.allocate_tensors()
            self._input_details = self._interpreter.get_input_details()
            self._output_details = self._interpreter.get_output_details()

            self._class_names = self._load_class_names(class_map_path)

            if self.continuous_capture:
                self._start_continuous_capture()

            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.real_classify
            )
            if self.publish_status:
                self.status_pub = self.create_publisher(
                    DiagnosticArray, '/bench/audio_classifier/status', 10
                )
            self.get_logger().info('YAMNet model loaded successfully')

        except SystemExit:
            raise
        except Exception as e:
            self.get_logger().error(f'Failed to init audio pipeline: {e}')
            sys.exit(1)

    def _load_class_names(self, class_map_path: str) -> list:
        """Load YAMNet class names from an already-validated class_map_path."""
        import csv
        names = []
        with open(class_map_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for row in reader:
                names.append(row[2])
        return names

    def _resolve_input_device(self):
        device_idx = int(self.get_parameter('device_index').value)
        if self.device_name_substring:
            from billiebot_audio.audio_device import query_input_devices, resolve_input_device
            devices = query_input_devices()
            resolved = resolve_input_device(devices, self.device_name_substring)
            if resolved is not None:
                return resolved
            self.get_logger().warning(
                f"No audio input device matched '{self.device_name_substring}' — "
                'falling back to device_index'
            )
        return device_idx if device_idx >= 0 else None

    def _start_continuous_capture(self):
        """Opens a continuous sounddevice.InputStream feeding an AudioRingBuffer. Capture
        latency is now ~0 regardless of processing cadence -- this is the actual fix for
        the 2 Hz cadence bug (the old bug was blocking on the 0.975 s *capture*, not on
        inference, which is small/bounded and fine to run synchronously in the timer)."""
        import sounddevice as sd

        sample_rate = int(self.get_parameter('sample_rate').value)
        device = self._resolve_input_device()

        capacity_samples = sample_rate * 3  # a few seconds of headroom

        # YAMNet consumes a mono waveform, so keep the ring buffer mono.
        # The XVF3800 itself is opened with all configured input channels,
        # then classification_channel selects which one is sent to YAMNet.
        self._ring_buffer = AudioRingBuffer(capacity_samples, channels=1)

        def _callback(indata, _frames, _time_info, status):
            selected_channel = indata[:, self.classification_channel]
            self._ring_buffer.write(
                selected_channel.copy(),
                overflowed=bool(status)
            )

        self._stream = sd.InputStream(
            samplerate=sample_rate,
            channels=self.input_channels,
            dtype='float32',
            device=device,
            callback=_callback,
        )
        self._stream.start()


    def real_classify(self):
        """Capture audio chunk and run YAMNet classification. Dispatches to the
        continuous-capture path (default) or the legacy blocking path
        (continuous_capture:=false, an explicit rollback)."""
        cycle_start = time.monotonic()
        try:
            if self.continuous_capture:
                self._real_classify_continuous(cycle_start)
            else:
                self._real_classify_legacy_blocking()
        except Exception as e:
            self.get_logger().warning(f'Audio classify error: {e}')

    def _real_classify_continuous(self, cycle_start: float):
        sample_rate = int(self.get_parameter('sample_rate').value)
        duration = float(self.get_parameter('chunk_duration_sec').value)
        window_samples = int(sample_rate * duration)

        waveform_2d = self._ring_buffer.read_last(window_samples)
        if waveform_2d is None:
            return  # ring buffer still warming up
        waveform = waveform_2d[:, 0]

        inference_start = time.monotonic()
        result = self._classify_waveform(waveform)
        inference_duration = time.monotonic() - inference_start

        if self.publish_status:
            self._publish_status(cycle_start, inference_duration, result)
        if result['event_type'] is not None:
            self._publish_event(result)

    def _real_classify_legacy_blocking(self):
        """Kept as the original blocking-capture-in-timer behavior, verbatim, as an
        explicit rollback for continuous_capture:=false. This is the confirmed
        non-functional 2 Hz-cadence bug -- it exists only for compatibility."""
        import sounddevice as sd

        sample_rate = int(self.get_parameter('sample_rate').value)
        duration = float(self.get_parameter('chunk_duration_sec').value)
        device_idx = int(self.get_parameter('device_index').value)

        device = device_idx if device_idx >= 0 else None
        audio = sd.rec(
            int(sample_rate * duration), samplerate=sample_rate, channels=1,
            dtype='float32', device=device,
        )
        sd.wait()
        waveform = audio.flatten()

        result = self._classify_waveform(waveform)
        if result['event_type'] is not None:
            self._publish_event(result)

    def _classify_waveform(self, waveform) -> dict:
        """Energy gate -> YAMNet inference -> confidence gate -> event-type mapping,
        shared by both capture paths. Always returns a result dict (even when gated out)
        so status reporting can show the top label/score every cycle, per plan."""
        import numpy as np

        rms = float(np.sqrt(np.mean(waveform ** 2))) if waveform.size else 0.0
        energy_db = 20.0 * np.log10(max(rms, 1e-10))
        result = {
            'energy_db': energy_db, 'passed_energy_gate': False,
            'top_name': '', 'top_score': 0.0,
            'passed_confidence_gate': False, 'event_type': None,
        }
        if energy_db < self.energy_threshold_db:
            return result
        result['passed_energy_gate'] = True
        
        waveform = waveform.astype(np.float32)
        expected_shape = tuple(int(v) for v in self._input_details[0]['shape'])

        if waveform.shape == expected_shape:
            input_data = waveform
        elif (1, waveform.shape[0]) == expected_shape:
            input_data = waveform[np.newaxis, :]
        else:
            raise ValueError(
                f'YAMNet input shape mismatch: model expects {expected_shape}, '
                f'waveform is {waveform.shape}'
            )

        self._interpreter.set_tensor(
            self._input_details[0]['index'],
            input_data
        )
        
        self._interpreter.invoke()
        scores = self._interpreter.get_tensor(self._output_details[0]['index'])

        top_idx = int(np.argmax(scores[0]))
        top_score = float(scores[0][top_idx])
        top_name = (
            self._class_names[top_idx]
            if top_idx < len(self._class_names)
            else str(top_idx)
        )
        result['top_name'] = top_name
        result['top_score'] = top_score

        if top_score < self.confidence_threshold:
            return result
        result['passed_confidence_gate'] = True
        result['event_type'] = YAMNET_DOG_CLASSES.get(top_name, AudioEvent.LOUD_NOISE)
        return result

    def _publish_event(self, result: dict):
        doa = self._get_doa()
        msg = AudioEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.event_type = result['event_type']
        msg.confidence = result['top_score']
        msg.doa_deg = doa
        msg.yamnet_label = result['top_name']
        msg.energy_db = result['energy_db']
        self.event_pub.publish(msg)

    def _publish_status(self, cycle_start: float, inference_duration: float, result: dict):
        """/bench/audio_classifier/status -- published every processing cycle regardless
        of whether an /audio/events message was emitted, so classifier processing rate can
        be measured independently of the sparse event topic (plan section on this)."""
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'audio_classifier_status'
        status.level = DiagnosticStatus.OK
        status.message = 'OK'
        overrun_count = self._ring_buffer.overrun_count if self._ring_buffer is not None else 0
        status.values = [
            KeyValue(key='cycle_timestamp_monotonic_s', value=str(cycle_start)),
            KeyValue(key='inference_duration_s', value=str(inference_duration)),
            KeyValue(key='energy_db', value=str(result['energy_db'])),
            KeyValue(key='passed_energy_gate', value=str(result['passed_energy_gate'])),
            KeyValue(key='top_label', value=result['top_name']),
            KeyValue(key='top_score', value=str(result['top_score'])),
            KeyValue(key='buffer_overrun_count', value=str(overrun_count)),
        ]
        arr.status.append(status)
        self.status_pub.publish(arr)

    def _get_doa(self) -> float:
        """Query a DoA-capable USB device for Direction of Arrival. Enumerates connected
        devices instead of assuming a single hardcoded VID:PID (Appendix B-14) -- see
        resolve_doa_device()."""
        try:
            import usb.core
            import usb.util

            raw_devices = list(usb.core.find(find_all=True))
            candidates = []
            for dev in raw_devices:
                product_name = ''
                try:
                    product_name = usb.util.get_string(dev, dev.iProduct) or ''
                except Exception:
                    pass
                candidates.append(UsbDeviceInfo(dev.idVendor, dev.idProduct, product_name))

            chosen = resolve_doa_device(candidates, self.doa_usb_product_substring)
            if chosen is None:
                return 0.0
            dev = raw_devices[candidates.index(chosen)]
            doa = dev.ctrl_transfer(
                usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR
                | usb.util.CTRL_RECIPIENT_DEVICE,
                0, 21, 0, 8
            )
            if len(doa) >= 2:
                return float(int.from_bytes(doa[:2], 'little'))
        except Exception:
            pass
        return 0.0

    def mock_classify(self):
        """Publish synthetic audio events for testing."""
        # Randomly produce bark/silence events
        r = random.random()
        if r < 0.15:
            msg = AudioEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.event_type = AudioEvent.BARK
            msg.confidence = 0.75 + random.uniform(-0.1, 0.1)
            msg.doa_deg = random.uniform(0, 360)
            msg.yamnet_label = 'Bark'
            msg.energy_db = -15.0 + random.uniform(-5, 5)
            self.event_pub.publish(msg)
        elif r < 0.20:
            msg = AudioEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.event_type = AudioEvent.WHINE
            msg.confidence = 0.6 + random.uniform(-0.1, 0.1)
            msg.doa_deg = random.uniform(0, 360)
            msg.yamnet_label = 'Whimper'
            msg.energy_db = -20.0 + random.uniform(-5, 5)
            self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AudioClassifier()
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
