#!/usr/bin/env python3
"""UT-BAT-02B stimulus node: drives the REAL production mission_controller across the exact
10.5 V SAFE boundary using synthetic sensor_msgs/BatteryState messages.

No Sensor Nano, no divider, no ADC. The physical path is UT-BAT-02's job; this test isolates
the *software inequality*, because analog uncertainty and 10-bit quantization make an exact
10.500 V boundary untestable through real hardware.

Per case the node:
  1. publishes a healthy `reset_voltage` for `reset_hold_sec`;
  2. calls /set_mode to leave SAFE (mission_controller latches SAFE with no exit path of its
     own -- mission_controller.py:147-152 sets the mode and nothing ever clears it);
  3. publishes the case voltage for `case_hold_sec` and records that window.

Expectations come from `safety_metrics.requirement_expects_safe()`, i.e. SYS-PLT-2's
"SAFE at <= 3.5 V/cell", NOT from what the code currently does. At exactly 10.5000 V the
requirement says SAFE and the production `<` comparison does not, so that case is expected
to FAIL until BLK-05 is fixed. That failure is the deliverable -- do not soften it here.

The node only publishes /battery_state and calls /set_mode. The mission controller performs
every threshold comparison itself; nothing in this file re-implements its state machine.
"""

import csv
import json
import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from billiebot_interfaces.srv import SetMode
from billiebot_interfaces.msg import MissionStatus

from billiebot_sensor_tests.sensor_nano.safety_metrics import requirement_expects_safe

FIELDNAMES = [
    'case_index', 'case_voltage_v', 'expected_safe', 'window_start_ns', 'window_end_ns',
    'reset_mode_requested', 'reset_mode_success', 'pre_case_mode',
]

CSV_NAME = 'threshold_cases.csv'

#: The three canonical UT-BAT-02B stimulus voltages (test plan section 18.2). 10.5 is exactly
#: representable in the float32 of BatteryState.voltage, and the neighbours land on distinct
#: float32 values either side of it, so the boundary is genuinely resolvable over the wire.
DEFAULT_CASE_VOLTAGES = [10.5001, 10.5000, 10.4999]

_STATE_RESET = 'reset'
_STATE_SET_MODE = 'set_mode'
_STATE_CASE = 'case'
_STATE_DONE = 'done'


class BatteryThresholdTest(Node):

    def __init__(self):
        super().__init__('battery_threshold_test')

        self.declare_parameter('case_voltages', DEFAULT_CASE_VOLTAGES)
        self.declare_parameter('safe_threshold_v', 10.5)
        self.declare_parameter('reset_voltage', 12.6)
        self.declare_parameter('reset_hold_sec', 3.0)
        self.declare_parameter('case_hold_sec', 6.0)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('reset_mode', int(MissionStatus.PATROL))
        self.declare_parameter('set_mode_service', '/set_mode')
        self.declare_parameter('set_mode_timeout_sec', 10.0)
        self.declare_parameter('battery_frame_id', 'base_link')
        self.declare_parameter('output_csv', '')

        self.case_voltages = [float(v) for v in self.get_parameter('case_voltages').value]
        self.safe_threshold_v = float(self.get_parameter('safe_threshold_v').value)
        self.reset_voltage = float(self.get_parameter('reset_voltage').value)
        self.reset_hold_sec = float(self.get_parameter('reset_hold_sec').value)
        self.case_hold_sec = float(self.get_parameter('case_hold_sec').value)
        self.reset_mode = int(self.get_parameter('reset_mode').value)
        self.set_mode_timeout_sec = float(self.get_parameter('set_mode_timeout_sec').value)
        self.battery_frame_id = str(self.get_parameter('battery_frame_id').value)
        self.output_csv = str(self.get_parameter('output_csv').value)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        service_name = str(self.get_parameter('set_mode_service').value)

        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.create_subscription(
            MissionStatus, '/billiebot/mission_status', self._on_status, 10
        )
        self.set_mode_client = self.create_client(SetMode, service_name)

        self._last_mode = None
        self._case_index = 0
        self._state = _STATE_RESET
        self._state_started_ns = self.get_clock().now().nanoseconds
        self._pending_future = None
        self._rows = []
        self._current_row = None
        self.finished = False

        self.create_timer(1.0 / rate_hz, self._tick)

        self.get_logger().info(
            f'UT-BAT-02B: {len(self.case_voltages)} cases at '
            f'{[f"{v:.4f}" for v in self.case_voltages]} V against the production '
            f'mission_controller; SYS-PLT-2 threshold <= {self.safe_threshold_v} V'
        )

    # -- helpers -------------------------------------------------------------------------

    def _on_status(self, msg: MissionStatus):
        self._last_mode = int(msg.mode)

    def _elapsed_sec(self) -> float:
        return (self.get_clock().now().nanoseconds - self._state_started_ns) / 1e9

    def _enter(self, state: str) -> None:
        self._state = state
        self._state_started_ns = self.get_clock().now().nanoseconds

    def _publish_voltage(self, voltage: float) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.battery_frame_id
        msg.voltage = float(voltage)
        # Same honesty rules as the real bridge: only voltage is being asserted here, and
        # mission_controller._on_battery reads only msg.voltage (mission_controller.py:110).
        msg.temperature = math.nan
        msg.current = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.percentage = math.nan
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        msg.cell_voltage = []
        msg.cell_temperature = []
        self.battery_pub.publish(msg)

    # -- state machine -------------------------------------------------------------------

    def _tick(self) -> None:
        if self._state == _STATE_DONE:
            return

        if self._case_index >= len(self.case_voltages):
            self._finish()
            return

        if self._state == _STATE_RESET:
            self._publish_voltage(self.reset_voltage)
            if self._elapsed_sec() >= self.reset_hold_sec:
                self._request_reset_mode()
            return

        if self._state == _STATE_SET_MODE:
            # Keep the healthy voltage flowing while the service call is in flight, so the
            # controller cannot re-trip on a stale low reading between the call and its reply.
            self._publish_voltage(self.reset_voltage)
            self._poll_reset_mode()
            return

        if self._state == _STATE_CASE:
            self._publish_voltage(self.case_voltages[self._case_index])
            if self._elapsed_sec() >= self.case_hold_sec:
                self._close_case()
            return

    def _request_reset_mode(self) -> None:
        if not self.set_mode_client.service_is_ready():
            if self._elapsed_sec() >= self.set_mode_timeout_sec:
                self.get_logger().error(
                    f'/set_mode did not become available within '
                    f'{self.set_mode_timeout_sec:.0f} s -- is the mission_controller running?'
                )
                self._enter(_STATE_SET_MODE)
                self._pending_future = None
            return
        request = SetMode.Request()
        request.mode = self.reset_mode
        self._pending_future = self.set_mode_client.call_async(request)
        self._enter(_STATE_SET_MODE)

    def _poll_reset_mode(self) -> None:
        success = False
        if self._pending_future is not None and self._pending_future.done():
            try:
                response = self._pending_future.result()
                success = bool(response.success)
            except Exception as exc:
                self.get_logger().error(f'/set_mode call failed: {exc}')
            self._pending_future = None
        elif self._pending_future is not None:
            if self._elapsed_sec() < self.set_mode_timeout_sec:
                return
            self.get_logger().error('/set_mode call timed out')
            self._pending_future = None
        elif self._elapsed_sec() < self.set_mode_timeout_sec:
            return

        voltage = self.case_voltages[self._case_index]
        expected = requirement_expects_safe(voltage, self.safe_threshold_v)
        self._current_row = {
            'case_index': self._case_index,
            'case_voltage_v': f'{voltage:.6f}',
            'expected_safe': expected,
            'reset_mode_requested': self.reset_mode,
            'reset_mode_success': success,
            'pre_case_mode': self._last_mode,
        }
        if self._last_mode == MissionStatus.SAFE:
            self.get_logger().error(
                f'case {self._case_index}: mission is still SAFE after /set_mode; the case '
                'window cannot distinguish a new transition from the previous one'
            )
        self._enter(_STATE_CASE)
        self._current_row['window_start_ns'] = self.get_clock().now().nanoseconds
        self.get_logger().info(
            f'case {self._case_index}: holding {voltage:.4f} V for {self.case_hold_sec:.0f} s '
            f'(SYS-PLT-2 expects SAFE={expected})'
        )

    def _close_case(self) -> None:
        self._current_row['window_end_ns'] = self.get_clock().now().nanoseconds
        observed_safe = self._last_mode == MissionStatus.SAFE
        expected = self._current_row['expected_safe']
        self.get_logger().info(
            f"case {self._case_index}: observed SAFE={observed_safe}, "
            f"required SAFE={expected} -> "
            f"{'as required' if observed_safe == expected else 'REQUIREMENT NOT MET'}"
        )
        self._rows.append(self._current_row)
        self._current_row = None
        self._case_index += 1
        self._enter(_STATE_RESET)

    def _finish(self) -> None:
        self._enter(_STATE_DONE)
        self._write_csv()
        self.finished = True
        self.get_logger().info(
            'UT-BAT-02B stimulus complete; scoring is performed by score_battery_safe '
            '--profile threshold against the recorded bag'
        )

    def _write_csv(self) -> None:
        if not self.output_csv:
            return
        path = Path(self.output_csv)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
                writer.writeheader()
                for row in self._rows:
                    writer.writerow({key: row.get(key, '') for key in FIELDNAMES})
        except OSError as exc:
            self.get_logger().error(f'could not write {path}: {exc}')

    def summary(self) -> str:
        return json.dumps(self._rows, indent=2, default=str)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryThresholdTest()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
