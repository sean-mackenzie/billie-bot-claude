#!/usr/bin/env python3
"""Bench-only continuous audio capture for UT-AUD-01: opens the resolved XVF3800 input via
a continuous sounddevice.InputStream (never a blocking sd.rec() call), feeds a bounded
queue so disk writes never block the audio callback, and saves one WAV file of the
configured duration per launch. Publishes running diagnostics (device identity, overflow
count, blocks written) to /bench/audio/diagnostics throughout the capture.

sounddevice is imported lazily so this module (and pytest collection) never requires
audio hardware/library on a dev machine. Device resolution reuses
billiebot_audio.audio_device so the bench and the production classifier agree on how the
XVF3800 is identified.
"""

import queue
import sys
import threading
import wave
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node


class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')

        self.declare_parameter('mock', False)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 2)
        self.declare_parameter('capture_duration_sec', 3.0)
        self.declare_parameter('capture_label', 'capture')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('device_name_substring', 'XVF3800')
        self.declare_parameter('device_index', -1)
        self.declare_parameter('fail_on_missing_device', True)

        self.mock = bool(self.get_parameter('mock').value)
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.channels = int(self.get_parameter('channels').value)
        self.capture_duration_sec = float(self.get_parameter('capture_duration_sec').value)
        self.capture_label = str(self.get_parameter('capture_label').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.device_name_substring = str(self.get_parameter('device_name_substring').value)
        self.device_index_param = int(self.get_parameter('device_index').value)
        self.fail_on_missing_device = bool(self.get_parameter('fail_on_missing_device').value)

        self.diag_pub = self.create_publisher(DiagnosticArray, '/bench/audio/diagnostics', 10)

        self._queue = queue.Queue(maxsize=200)
        self._overflow_count = 0
        self._blocks_written = 0
        self._resolved_device_name = ''
        self._stream = None

        self.diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        if self.mock:
            self.get_logger().info('Audio capture node running in MOCK mode')
            self._write_mock_wav()
        else:
            self._start_real_capture()

    def _resolve_device(self):
        if self.device_index_param >= 0:
            return self.device_index_param
        from billiebot_audio.audio_device import query_input_devices, resolve_input_device
        devices = query_input_devices()
        idx = resolve_input_device(devices, self.device_name_substring)
        if idx is not None:
            self._resolved_device_name = devices[idx].get('name', '')
        return idx

    def _start_real_capture(self):
        try:
            import sounddevice as sd
        except ImportError:
            self.get_logger().error(
                'sounddevice not installed -- run with mock:=true or install sounddevice'
            )
            if self.fail_on_missing_device:
                sys.exit(1)
            return

        device = self._resolve_device()
        if device is None and self.device_name_substring:
            self.get_logger().error(
                f"No audio input device matched '{self.device_name_substring}' -- "
                'resolve by stable name, not ALSA card/device index'
            )
            if self.fail_on_missing_device:
                sys.exit(1)
            return

        def _callback(indata, _frames, _time_info, status):
            if status:
                self._overflow_count += 1
            try:
                self._queue.put_nowait(indata.copy())
            except queue.Full:
                self._overflow_count += 1

        try:
            self._stream = sd.InputStream(
                samplerate=self.sample_rate, channels=self.channels, dtype='float32',
                device=device, callback=_callback,
            )
            self._stream.start()
        except Exception as e:
            self.get_logger().error(f'Failed to open audio input stream: {e}')
            if self.fail_on_missing_device:
                sys.exit(1)
            return

        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()

    def _writer_loop(self):
        blocks = []
        total_samples = 0
        target_samples = int(self.sample_rate * self.capture_duration_sec)
        while total_samples < target_samples:
            try:
                block = self._queue.get(timeout=1.0)
            except queue.Empty:
                continue
            blocks.append(block)
            total_samples += block.shape[0]
            self._blocks_written += 1
        try:
            self._stream.stop()
            self._stream.close()
        except Exception:
            pass
        waveform = (
            np.concatenate(blocks, axis=0)[:target_samples] if blocks
            else np.zeros((0, self.channels), dtype=np.float32)
        )
        self._write_wav(waveform)
        self.get_logger().info(
            f"Capture '{self.capture_label}' complete: {waveform.shape[0]} samples written"
        )

    def _write_wav(self, waveform: np.ndarray):
        out_dir = Path(self.output_dir) if self.output_dir else Path.cwd()
        out_dir.mkdir(parents=True, exist_ok=True)
        path = out_dir / f'{self.capture_label}.wav'
        pcm16 = np.clip(waveform * 32767.0, -32768, 32767).astype(np.int16)
        with wave.open(str(path), 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(pcm16.tobytes())
        self.get_logger().info(f'Wrote {path}')

    def _write_mock_wav(self):
        n = int(self.sample_rate * self.capture_duration_sec)
        t = np.arange(n) / self.sample_rate
        tone = 0.2 * np.sin(2 * np.pi * 440.0 * t)
        waveform = np.tile(tone[:, None], (1, self.channels)).astype(np.float32)
        self._write_wav(waveform)

    def _publish_diagnostics(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'audio_capture_node'
        status.level = DiagnosticStatus.OK if self._overflow_count == 0 else DiagnosticStatus.WARN
        status.message = 'OK' if self._overflow_count == 0 else 'overflow/underflow detected'
        status.values = [
            KeyValue(key='device_name', value=self._resolved_device_name),
            KeyValue(key='sample_rate', value=str(self.sample_rate)),
            KeyValue(key='channels', value=str(self.channels)),
            KeyValue(key='overflow_count', value=str(self._overflow_count)),
            KeyValue(key='blocks_written', value=str(self._blocks_written)),
        ]
        arr.status.append(status)
        self.diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AudioCaptureNode()
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
