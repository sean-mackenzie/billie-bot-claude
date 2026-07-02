#!/usr/bin/env python3
"""Mission controller — orchestrates BillieBot's operational modes.

Manages the IDLE -> PATROL -> INVESTIGATE -> TRACK_OBSERVE -> RETURN -> SAFE
state machine. Publishes MissionStatus. Interfaces with Nav2 for waypoint
navigation and monitors safety conditions.
"""

import math
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

from billiebot_interfaces.msg import (
    DogState,
    AudioEvent,
    MissionStatus,
)
from billiebot_interfaces.srv import EStop, SetMode


class Mode(IntEnum):
    IDLE = 0
    PATROL = 1
    INVESTIGATE = 2
    TRACK_OBSERVE = 3
    RETURN = 4
    SAFE = 5


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')

        self.declare_parameter('mock', False)
        self.declare_parameter('patrol_waypoints', ['living_room', 'kitchen',
                                                     'bedroom', 'hallway'])
        self.declare_parameter('battery_safe_voltage', 10.5)
        self.declare_parameter('max_nav_failures', 3)
        self.declare_parameter('tick_rate_hz', 2.0)

        self.mock = bool(self.get_parameter('mock').value)
        self.patrol_waypoints = list(
            self.get_parameter('patrol_waypoints').value
        )
        self.battery_safe_voltage = float(
            self.get_parameter('battery_safe_voltage').value
        )
        self.max_nav_failures = int(
            self.get_parameter('max_nav_failures').value
        )

        # State
        self._mode = Mode.IDLE
        self._current_wp_idx = 0
        self._nav_failure_count = 0
        self._estopped = False
        self._battery_voltage = 12.6
        self._dog_state = DogState()
        self._dog_found = False
        self._last_audio_event = None

        # Subscribers
        self.create_subscription(
            DogState, '/billie/state', self._on_dog_state, 10
        )
        self.create_subscription(
            BatteryState, '/battery_state', self._on_battery, 10
        )
        self.create_subscription(
            Bool, '/dog/found', self._on_dog_found, 10
        )
        self.create_subscription(
            AudioEvent, '/audio/events', self._on_audio, 10
        )

        # Publisher
        self.status_pub = self.create_publisher(
            MissionStatus, '/billiebot/mission_status', 10
        )

        # Services
        self.set_mode_srv = self.create_service(
            SetMode, '/set_mode', self._set_mode_callback
        )

        # Nav2 action client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        self._nav_active = False

        # Main tick
        tick_rate = float(self.get_parameter('tick_rate_hz').value)
        self.timer = self.create_timer(1.0 / tick_rate, self.tick)

        self.get_logger().info('MissionController started')

    def _on_dog_state(self, msg: DogState):
        self._dog_state = msg

    def _on_battery(self, msg: BatteryState):
        self._battery_voltage = msg.voltage

    def _on_dog_found(self, msg: Bool):
        self._dog_found = msg.data

    def _on_audio(self, msg: AudioEvent):
        self._last_audio_event = msg
        if msg.event_type == AudioEvent.BARK and self._mode == Mode.PATROL:
            # Re-sort patrol towards DoA (SYS-FND-2)
            self.get_logger().info(
                f'Audio event: bark at DoA={msg.doa_deg:.0f} deg'
            )

    def _set_mode_callback(self, request, response):
        prev_mode = int(self._mode)
        new_mode = request.mode
        if new_mode > 5:
            response.success = False
            response.message = f'Invalid mode: {new_mode}'
            response.previous_mode = prev_mode
            return response

        self._mode = Mode(new_mode)
        response.success = True
        response.message = f'Mode set to {Mode(new_mode).name}'
        response.previous_mode = prev_mode
        self.get_logger().info(
            f'Mode: {Mode(prev_mode).name} -> {Mode(new_mode).name}'
        )
        return response

    def tick(self):
        """Main mission tick — runs the state machine."""
        # Safety checks first
        if self._estopped:
            self._mode = Mode.SAFE

        if self._battery_voltage < self.battery_safe_voltage:
            if self._mode != Mode.SAFE:
                self.get_logger().warn(
                    f'Battery low ({self._battery_voltage:.1f}V) — entering SAFE'
                )
                self._mode = Mode.SAFE

        if self._nav_failure_count >= self.max_nav_failures:
            if self._mode != Mode.SAFE:
                self.get_logger().warn(
                    f'{self._nav_failure_count} nav failures — entering SAFE (SYS-NAV-4)'
                )
                self._mode = Mode.SAFE

        # State machine
        if self._mode == Mode.PATROL:
            if self._dog_found:
                self._mode = Mode.TRACK_OBSERVE
                self.get_logger().info('Dog found — TRACK_OBSERVE')

        elif self._mode == Mode.TRACK_OBSERVE:
            if not self._dog_found:
                self._mode = Mode.PATROL
                self.get_logger().info('Dog lost — resuming PATROL')

        # Publish status
        self._publish_status()

    def _publish_status(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = int(self._mode)
        msg.current_waypoint = (
            self.patrol_waypoints[self._current_wp_idx]
            if self._current_wp_idx < len(self.patrol_waypoints)
            else ''
        )
        msg.dog_state = self._dog_state.state
        msg.dog_confidence = self._dog_state.confidence
        msg.battery_voltage = self._battery_voltage
        msg.nav_active = self._nav_active
        msg.recovery_count = self._nav_failure_count
        msg.estopped = self._estopped
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
