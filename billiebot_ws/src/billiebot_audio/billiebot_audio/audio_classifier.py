#!/usr/bin/env python3
"""YAMNet-based audio classifier with Direction of Arrival (DoA).

Real mode: streams from ReSpeaker XVF3800 USB mic array, runs YAMNet TFLite,
           queries a DoA-capable USB device for direction of arrival.
Mock mode: publishes periodic synthetic bark events for testing.

Real mode also enforces BillieBot's conservative XVF3800 power policy before opening the
input stream -- WS2812 LED-ring power off, onboard amplifier disabled, internal DoA LED mode
preserved -- via xvf3800_control.py, which owns all USB vendor-control access. See that
module's docstring for the exact invariant and why it is volatile.

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
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

from billiebot_audio import xvf3800_control
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


def _bool_str(value: Optional[bool]) -> str:
    """Lowercase 'true'/'false' for the XVF3800 diagnostic KeyValues, 'unknown' when the
    device state could not be read. Pre-existing KeyValues keep their str(bool) form so the
    schema DT-AUD-01 already scores against does not shift underneath it."""
    if value is None:
        return 'unknown'
    return 'true' if value else 'false'


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
        # XVF3800 power policy: LED ring unpowered, onboard amplifier disabled, LED_EFFECT
        # left at 4 (DoA). Volatile only -- nothing is written to device flash. strict=true
        # makes an unverifiable policy a startup failure rather than a silent power leak.
        self.declare_parameter('xvf3800_power_policy_enabled', True)
        self.declare_parameter('xvf3800_power_policy_strict', True)
        self.declare_parameter('xvf3800_power_verify_period_sec', 30.0)

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
        self.xvf3800_power_policy_enabled = bool(
            self.get_parameter('xvf3800_power_policy_enabled').value
        )
        self.xvf3800_power_policy_strict = bool(
            self.get_parameter('xvf3800_power_policy_strict').value
        )
        self.xvf3800_power_verify_period_sec = float(
            self.get_parameter('xvf3800_power_verify_period_sec').value
        )

        self.event_pub = self.create_publisher(AudioEvent, '/audio/events', 10)
        self.status_pub = None
        self._ring_buffer = None
        self._stream = None
        # XVF3800 state. Mock mode leaves all of it untouched -- no USB is ever enumerated.
        self._xvf_device = None
        self._xvf_policy = None
        self._xvf_last_error = ''
        self._xvf_verify_timer = None

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

            # Before any audio interface is opened: assert the conservative power policy so
            # the LED ring and onboard amplifier are provably off rather than assumed off.
            self._apply_xvf_power_policy(initial=True)

            if self.continuous_capture:
                self._start_continuous_capture()

            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.real_classify
            )
            if self.xvf3800_power_policy_enabled and self.xvf3800_power_verify_period_sec > 0:
                self._xvf_verify_timer = self.create_timer(
                    self.xvf3800_power_verify_period_sec, self._verify_xvf_power_policy
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

    # --- XVF3800 power policy and USB control -------------------------------------------

    def _ensure_xvf_device(self):
        """Return a live Xvf3800Device, opening one on demand.

        The handle is cached but never permanently: any control failure calls
        _invalidate_xvf_device(), so the next call re-enumerates the bus and recovers after
        a USB re-enumeration (unplug/replug, or a hub reset). This is strictly better than
        the previous behavior, which rescanned every device on the bus on every published
        event and still could not survive a handle going stale mid-read.
        """
        if self._xvf_device is None:
            self._xvf_device = xvf3800_control.open_xvf3800(self.doa_usb_product_substring)
        return self._xvf_device

    def _invalidate_xvf_device(self):
        if self._xvf_device is not None:
            self._xvf_device.close()
            self._xvf_device = None

    def _apply_xvf_power_policy(self, initial: bool = False):
        """Apply and verify the XVF3800 power policy (read-before-write, zero writes when
        the device is already compliant, nothing persisted to flash).

        Strict mode makes an unverifiable policy a hard startup failure -- consistent with
        this node's fail-loud handling of a missing YAMNet model -- because continuing would
        silently leave the LED ring and amplifier drawing power. After startup, a failure is
        only recorded and retried at the next verification tick: a transient USB error must
        not take down a working classifier.
        """
        if not self.xvf3800_power_policy_enabled:
            return

        try:
            device = self._ensure_xvf_device()
            result = xvf3800_control.ensure_billiebot_power_policy(device)
        except Exception as e:
            self._invalidate_xvf_device()
            result = xvf3800_control.PowerPolicyResult(
                ok=False, error=f'{type(e).__name__}: {e}'
            )

        self._xvf_policy = result
        self._xvf_last_error = '' if result.ok else result.error

        if result.ok:
            if initial or result.writes_performed:
                self.get_logger().info(xvf3800_control.describe_policy_result(result))
            if result.mic_muted:
                self.get_logger().warning(
                    'XVF3800 X0D30 is high: microphones are MUTED and the red mute LED is '
                    'on. This node does not write X0D30 -- clear it at the device if '
                    'capture is silent.'
                )
            return

        # Not ok: the handle may be stale, so drop it and let the next attempt re-resolve.
        self._invalidate_xvf_device()
        self.get_logger().error(xvf3800_control.describe_policy_result(result))
        if initial and self.xvf3800_power_policy_strict:
            self.get_logger().error(
                'xvf3800_power_policy_strict is true — refusing to start with an '
                'unverified XVF3800 power configuration'
            )
            sys.exit(1)
        if initial:
            self.get_logger().warning(
                'xvf3800_power_policy_strict is false — continuing with an unverified '
                'XVF3800 power configuration'
            )

    def _verify_xvf_power_policy(self):
        """Low-rate re-verification. Normally a pure read that performs no writes; only a
        device that has reverted (re-enumeration, firmware reset) triggers a repair."""
        self._apply_xvf_power_policy(initial=False)

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
        policy = self._xvf_policy
        policy_ok = bool(policy is not None and policy.ok)
        # 'disabled' rather than 'false' when enforcement was never attempted: false would
        # read as "verification failed" when in fact it was intentionally switched off.
        policy_ok_value = ('disabled' if not self.xvf3800_power_policy_enabled
                           else _bool_str(policy_ok))
        if self.xvf3800_power_policy_enabled and not policy_ok:
            status.level = DiagnosticStatus.WARN
            status.message = 'XVF3800 power policy not verified'
        else:
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
            # XVF3800 power policy. Lowercase true/false, 'disabled' when enforcement was
            # intentionally turned off, and 'unknown' rather than a misleading false when
            # the device state could not be read at all.
            KeyValue(key='xvf3800_power_policy_ok', value=policy_ok_value),
            KeyValue(key='xvf3800_led_power_off',
                     value=_bool_str(policy.led_power_off if policy else None)),
            KeyValue(key='xvf3800_amplifier_disabled',
                     value=_bool_str(policy.amplifier_disabled if policy else None)),
            KeyValue(key='xvf3800_mic_muted',
                     value=_bool_str(policy.mic_muted if policy else None)),
            KeyValue(key='xvf3800_led_effect',
                     value=str(policy.led_effect) if policy and policy.led_effect is not None
                     else 'unknown'),
            KeyValue(key='xvf3800_last_control_error', value=self._xvf_last_error),
        ]
        arr.status.append(status)
        self.status_pub.publish(arr)

    def _clear_stale_doa_error(self):
        """A successful DoA read proves the control path recovered, so drop a stale DoA
        error -- but never clear a live power-policy failure, which a good DoA read does not
        fix. When the policy is healthy, disabled, or never run, _xvf_last_error can only
        hold a stale DoA error, so clearing it is safe."""
        if self._xvf_policy is not None and not self._xvf_policy.ok:
            return
        self._xvf_last_error = ''

    def _read_doa_degrees(self) -> Optional[float]:
        """Read DOA_VALUE via xvf3800_control, or None if the read failed.

        Returning None rather than 0.0 is the point: 0 degrees is a legitimate physical
        bearing, and DT-AUD-02 gates on `dt_aud_02_doa_not_fixed_at_zero`, so a failed read
        must be distinguishable from the array genuinely pointing forward. A failure drops
        the cached handle so the next read re-resolves the device.
        """
        # A missing handle means the previous device was lost. A reconnected XVF3800 comes
        # back at firmware defaults -- LED ring powered, amplifier enabled -- so the policy
        # must be restored before this device is trusted for anything. Waiting for the 30 s
        # verification timer instead would leave the ring lit for up to a full period.
        if self._xvf_device is None and self.xvf3800_power_policy_enabled:
            self._apply_xvf_power_policy(initial=False)
            if self._xvf_policy is None or not self._xvf_policy.ok:
                # Error already recorded and logged; the next read and the periodic timer
                # both retry. Never fatal post-startup.
                return None

        try:
            reading = self._ensure_xvf_device().read_doa()
        except Exception as e:
            self._invalidate_xvf_device()
            self._xvf_last_error = f'DoA read failed: {type(e).__name__}: {e}'
            return None

        if not (xvf3800_control.DOA_MIN_DEG <= reading.degrees <= xvf3800_control.DOA_MAX_DEG):
            self._xvf_last_error = (
                f'DoA read out of range: {reading.degrees} deg'
            )
            return None

        self._clear_stale_doa_error()
        return float(reading.degrees)

    def _get_doa(self) -> float:
        """Direction of Arrival in degrees for AudioEvent.doa_deg.

        Keeps the historical contract -- always a float, 0.0 on failure -- so AudioEvent is
        unchanged; the failure itself is recorded in _xvf_last_error and surfaced in
        /bench/audio_classifier/status rather than being invisible.
        """
        doa = self._read_doa_degrees()
        if doa is None:
            self.get_logger().warning(f'DoA query failed: {self._xvf_last_error}')
            return 0.0
        return doa

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

    def destroy_node(self):
        """Release the XVF3800 control handle so a restarted node can re-open it. The USB
        audio interfaces are untouched here -- they belong to sounddevice/ALSA, not to us."""
        self._invalidate_xvf_device()
        return super().destroy_node()


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
