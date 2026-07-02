#!/usr/bin/env python3
"""YAMNet-based audio classifier with Direction of Arrival (DoA).

Real mode: streams from ReSpeaker XVF3800 USB mic array, runs YAMNet TFLite,
           queries Seeed host-control API for DoA.
Mock mode: publishes periodic synthetic bark events for testing.
"""

import random
import time

import rclpy
from rclpy.node import Node

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
        self.declare_parameter('energy_threshold_db', -30.0)

        self.mock = bool(self.get_parameter('mock').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.energy_threshold_db = float(
            self.get_parameter('energy_threshold_db').value
        )

        self.event_pub = self.create_publisher(AudioEvent, '/audio/events', 10)

        if self.mock:
            self.get_logger().info('Audio classifier running in MOCK mode')
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.mock_classify
            )
        else:
            self.get_logger().info('Audio classifier starting real pipeline')
            self.init_real_pipeline()

    def init_real_pipeline(self):
        """Initialize YAMNet TFLite model and audio stream."""
        try:
            import numpy as np

            # Load YAMNet TFLite model
            model_path = str(self.get_parameter('model_path').value)
            if not model_path:
                self.get_logger().error(
                    'No model_path specified for YAMNet — '
                    'download yamnet.tflite from TensorFlow Hub'
                )
                return

            try:
                import tflite_runtime.interpreter as tflite
                self._interpreter = tflite.Interpreter(model_path=model_path)
            except ImportError:
                import tensorflow as tf
                self._interpreter = tf.lite.Interpreter(model_path=model_path)

            self._interpreter.allocate_tensors()
            self._input_details = self._interpreter.get_input_details()
            self._output_details = self._interpreter.get_output_details()

            # Load class map
            self._class_names = self._load_class_names()

            # Start audio capture timer
            self.timer = self.create_timer(
                1.0 / self.publish_rate, self.real_classify
            )
            self.get_logger().info('YAMNet model loaded successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to init audio pipeline: {e}')

    def _load_class_names(self) -> list:
        """Load YAMNet class names. Returns list of class name strings."""
        try:
            import csv
            import os
            model_path = str(self.get_parameter('model_path').value)
            class_map_path = os.path.join(
                os.path.dirname(model_path), 'yamnet_class_map.csv'
            )
            names = []
            with open(class_map_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # skip header
                for row in reader:
                    names.append(row[2])
            return names
        except Exception:
            self.get_logger().warning(
                'Could not load YAMNet class map — using indices'
            )
            return [str(i) for i in range(521)]

    def real_classify(self):
        """Capture audio chunk and run YAMNet classification."""
        try:
            import numpy as np
            import sounddevice as sd

            sample_rate = int(self.get_parameter('sample_rate').value)
            duration = float(self.get_parameter('chunk_duration_sec').value)
            device_idx = int(self.get_parameter('device_index').value)

            device = device_idx if device_idx >= 0 else None
            audio = sd.rec(
                int(sample_rate * duration),
                samplerate=sample_rate,
                channels=1,
                dtype='float32',
                device=device,
            )
            sd.wait()
            waveform = audio.flatten()

            # Compute energy
            rms = float(np.sqrt(np.mean(waveform ** 2)))
            energy_db = 20.0 * np.log10(max(rms, 1e-10))

            if energy_db < self.energy_threshold_db:
                return

            # Run YAMNet
            input_data = np.expand_dims(waveform, axis=0).astype(np.float32)
            self._interpreter.set_tensor(
                self._input_details[0]['index'], input_data
            )
            self._interpreter.invoke()
            scores = self._interpreter.get_tensor(
                self._output_details[0]['index']
            )

            # Find top class
            top_idx = int(np.argmax(scores[0]))
            top_score = float(scores[0][top_idx])
            top_name = (
                self._class_names[top_idx]
                if top_idx < len(self._class_names)
                else str(top_idx)
            )

            if top_score < self.confidence_threshold:
                return

            # Map to event type
            event_type = YAMNET_DOG_CLASSES.get(
                top_name, AudioEvent.LOUD_NOISE
            )

            # Get DoA from ReSpeaker
            doa = self._get_doa()

            msg = AudioEvent()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.event_type = event_type
            msg.confidence = top_score
            msg.doa_deg = doa
            msg.yamnet_label = top_name
            msg.energy_db = energy_db
            self.event_pub.publish(msg)

        except Exception as e:
            self.get_logger().warning(f'Audio classify error: {e}')

    def _get_doa(self) -> float:
        """Query ReSpeaker for Direction of Arrival."""
        try:
            import usb.core
            dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
            if dev is not None:
                # Read DOA from ReSpeaker firmware
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
    node = AudioClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
