#!/usr/bin/env python3
"""State fusion node — fuses visual, thermal, and audio evidence into a
single dog behavioral state estimate.

Uses a 10-second sliding window with hysteresis to prevent rapid state flapping.
Publishes DogState on /billie/state.
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from billiebot_interfaces.msg import (
    DogDetection3D,
    DogState,
    AudioEvent,
    ThermalBlob,
)
from billiebot_interfaces.srv import GetDogState

# State constants matching DogState.msg
NOT_FOUND = 0
SLEEPING = 1
RESTING = 2
ACTIVE = 3
BARKING = 4
EATING = 5

STATE_NAMES = {
    NOT_FOUND: 'NOT_FOUND',
    SLEEPING: 'SLEEPING',
    RESTING: 'RESTING',
    ACTIVE: 'ACTIVE',
    BARKING: 'BARKING',
    EATING: 'EATING',
}


class StateFusion(Node):
    def __init__(self):
        super().__init__('state_fusion')

        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('window_sec', 10.0)
        self.declare_parameter('hysteresis_sec', 3.0)
        self.declare_parameter('bark_rate_stress_threshold', 0.3)
        self.declare_parameter('thermal_only_timeout_sec', 30.0)

        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.window_sec = float(self.get_parameter('window_sec').value)
        self.hysteresis_sec = float(self.get_parameter('hysteresis_sec').value)
        self.bark_stress_thresh = float(
            self.get_parameter('bark_rate_stress_threshold').value
        )

        # Evidence buffers (timestamped)
        self._visual_detections = deque(maxlen=100)
        self._thermal_blobs = deque(maxlen=50)
        self._audio_events = deque(maxlen=50)
        self._dog_poses = deque(maxlen=50)

        # Current state
        self._current_state = NOT_FOUND
        self._state_confidence = 0.0
        self._state_start_time = time.monotonic()
        self._last_state_change = time.monotonic()
        self._last_dog_position = None
        self._last_dog_room = ''

        # Subscribers
        self.create_subscription(
            DogDetection3D, '/dog/detections_3d', self._on_detection, 10
        )
        self.create_subscription(
            ThermalBlob, '/thermal/blob', self._on_thermal, 10
        )
        self.create_subscription(
            AudioEvent, '/audio/events', self._on_audio, 10
        )
        self.create_subscription(
            PoseStamped, '/dog/pose_map', self._on_dog_pose, 10
        )

        # Publisher
        self.state_pub = self.create_publisher(DogState, '/billie/state', 10)

        # Service
        self.get_state_srv = self.create_service(
            GetDogState, '/get_dog_state', self._get_state_callback
        )

        # Timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.fuse_and_publish
        )

        self.get_logger().info('StateFusion started')

    def _on_detection(self, msg: DogDetection3D):
        self._visual_detections.append((time.monotonic(), msg))

    def _on_thermal(self, msg: ThermalBlob):
        self._thermal_blobs.append((time.monotonic(), msg))

    def _on_audio(self, msg: AudioEvent):
        self._audio_events.append((time.monotonic(), msg))

    def _on_dog_pose(self, msg: PoseStamped):
        self._dog_poses.append((time.monotonic(), msg))
        self._last_dog_position = msg.pose.position

    def _prune_window(self):
        """Remove evidence older than the sliding window."""
        cutoff = time.monotonic() - self.window_sec
        for buf in (self._visual_detections, self._thermal_blobs,
                    self._audio_events, self._dog_poses):
            while buf and buf[0][0] < cutoff:
                buf.popleft()

    def fuse_and_publish(self):
        """Fuse all evidence and publish state."""
        self._prune_window()
        now = time.monotonic()

        # Count evidence in window
        n_visual = len(self._visual_detections)
        n_thermal = len([b for _, b in self._thermal_blobs
                        if b.is_dog_candidate])
        n_barks = len([e for _, e in self._audio_events
                      if e.event_type == AudioEvent.BARK])
        n_whines = len([e for _, e in self._audio_events
                       if e.event_type == AudioEvent.WHINE])
        n_audio = n_barks + n_whines

        # Determine candidate state
        candidate_state = NOT_FOUND
        confidence = 0.0

        if n_barks >= 2:
            candidate_state = BARKING
            confidence = min(0.9, 0.5 + n_barks * 0.1)
        elif n_visual > 0:
            # Use average detection confidence
            avg_conf = sum(d.confidence for _, d in self._visual_detections) / n_visual

            if n_thermal > 0 and n_visual > 0:
                # Both visual + thermal: high confidence
                thermal_temps = [b.mean_temp for _, b in self._thermal_blobs
                                if b.is_dog_candidate]
                avg_temp = sum(thermal_temps) / len(thermal_temps) if thermal_temps else 0

                if avg_temp < 33.0 and n_audio == 0:
                    candidate_state = SLEEPING
                    confidence = min(0.95, avg_conf + 0.1)
                elif n_audio == 0 and avg_temp < 36.0:
                    candidate_state = RESTING
                    confidence = min(0.9, avg_conf + 0.05)
                else:
                    candidate_state = ACTIVE
                    confidence = avg_conf
            else:
                # Visual only
                candidate_state = ACTIVE
                confidence = avg_conf * 0.8
        elif n_thermal > 0:
            candidate_state = RESTING
            confidence = 0.4
        # else: NOT_FOUND

        # Apply hysteresis
        if candidate_state != self._current_state:
            elapsed_since_change = now - self._last_state_change
            if elapsed_since_change >= self.hysteresis_sec:
                self._current_state = candidate_state
                self._state_confidence = confidence
                self._state_start_time = now
                self._last_state_change = now
                self.get_logger().info(
                    f'State: {STATE_NAMES[self._current_state]} '
                    f'(conf={confidence:.2f})'
                )
        else:
            self._state_confidence = confidence

        # Compute stress proxy (SYS-EXT-4)
        bark_rate = n_barks / self.window_sec
        stress_proxy = min(1.0, bark_rate / self.bark_stress_thresh)

        # Build context vector (SYS-EXT-3)
        context = [
            float(n_visual),
            float(n_thermal),
            float(n_barks),
            float(n_whines),
            float(bark_rate),
            stress_proxy,
        ]

        # Publish
        msg = DogState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self._current_state
        msg.confidence = self._state_confidence
        msg.stress_proxy = stress_proxy
        msg.context = context
        msg.state_duration = now - self._state_start_time
        msg.room = self._last_dog_room

        if self._last_dog_position is not None:
            msg.position = self._last_dog_position

        self.state_pub.publish(msg)

    def _get_state_callback(self, request, response):
        response.state = DogState()
        response.state.state = self._current_state
        response.state.confidence = self._state_confidence
        response.state.room = self._last_dog_room
        if self._last_dog_position is not None:
            response.state.position = self._last_dog_position
        response.dog_found = self._current_state != NOT_FOUND
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StateFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
