#!/usr/bin/env python3
"""CLI: `record_battery_point` -- capture one UT-BAT-01 calibration point.

Run once per PSU setpoint while the bench launch is streaming, in a second terminal:

    ros2 run billiebot_sensor_tests record_battery_point \\
      --results-dir "$RESULTS" \\
      --setpoint-v 10.50 --dmm-battery-v 10.497 --dmm-a0-v 1.749

It records the operator's DMM ground truth *and* samples the live /battery_state and
/bench/battery/adc topics for a short window, so each row pairs an independent physical
measurement with what ROS reported at that same moment. Rows are appended to
exports/battery_points.csv; the run's rosbag remains the authoritative time series and this
CSV is the operator-entered ground truth that the bag cannot contain.

The node subscribes only. It never publishes, never reconfigures the bridge, and never
writes back a calibration -- UT-BAT-01 measures the shipped conversion's error, and a tool
that tuned the conversion would erase the measurement.
"""

import argparse
import csv
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

from billiebot_sensor_tests.common.result_dir import BenchResultDir

FIELDNAMES = [
    't_start_ns', 't_end_ns', 'setpoint_v', 'dmm_battery_v', 'dmm_a0_v',
    'dmm_divider_ratio', 'ros_voltage_mean_v', 'ros_voltage_std_v', 'ros_voltage_count',
    'adc_mean', 'adc_std', 'adc_count', 'sample_window_sec', 'notes',
]

CSV_NAME = 'battery_points.csv'


def _mean(values):
    return sum(values) / len(values) if values else None


def _std(values):
    if len(values) < 2:
        return 0.0 if values else None
    mean = _mean(values)
    return (sum((v - mean) ** 2 for v in values) / len(values)) ** 0.5


class BatteryPointRecorder(Node):
    """Collects /battery_state and /bench/battery/adc for a fixed wall-clock window."""

    def __init__(self, battery_topic: str, adc_topic: str):
        super().__init__('battery_point_recorder')
        self.voltages = []
        self.adc_values = []
        self.create_subscription(BatteryState, battery_topic, self._on_battery, 50)
        self.create_subscription(Float32, adc_topic, self._on_adc, 50)

    def _on_battery(self, msg: BatteryState):
        self.voltages.append(float(msg.voltage))

    def _on_adc(self, msg: Float32):
        self.adc_values.append(float(msg.data))


def _append_row(csv_path: Path, row: dict) -> None:
    """Append one row, writing the header only when the file is new.

    Append mode (never rewrite) so an interrupted sweep keeps every point already captured --
    losing the 12.6 V row because the 9.9 V row crashed would waste a whole bench session.
    """
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    new_file = not csv_path.exists()
    with open(csv_path, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        if new_file:
            writer.writeheader()
        writer.writerow(row)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description='Record one UT-BAT-01 battery calibration point with DMM ground truth.'
    )
    parser.add_argument('--results-dir', required=True)
    parser.add_argument('--setpoint-v', type=float, required=True,
                         help='PSU setpoint as dialled in, in volts')
    parser.add_argument('--dmm-battery-v', type=float, required=True,
                         help='DMM reading at the divider input (V_BAT), in volts')
    parser.add_argument('--dmm-a0-v', type=float, required=True,
                         help='DMM reading at the divider midpoint / Nano A0, in volts')
    parser.add_argument('--sample-window-sec', type=float, default=3.0,
                         help='how long to sample the live ROS topics (default: 3.0)')
    parser.add_argument('--battery-topic', default='/battery_state')
    parser.add_argument('--adc-topic', default='/bench/battery/adc')
    parser.add_argument('--notes', default='')
    args = parser.parse_args(argv)

    if args.sample_window_sec <= 0:
        parser.error('--sample-window-sec must be positive')

    result_dir = BenchResultDir(args.results_dir)
    if not result_dir.path.exists():
        print(f'ERROR: results dir {result_dir.path} does not exist -- start the bench '
              'launch first so it can create the run directory', file=sys.stderr)
        return 1

    rclpy.init()
    node = BatteryPointRecorder(args.battery_topic, args.adc_topic)
    t_start_ns = node.get_clock().now().nanoseconds

    node.get_logger().info(
        f'sampling {args.battery_topic} and {args.adc_topic} for '
        f'{args.sample_window_sec:.1f} s at setpoint {args.setpoint_v} V'
    )
    deadline = time.monotonic() + args.sample_window_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    t_end_ns = node.get_clock().now().nanoseconds
    voltages = list(node.voltages)
    adc_values = list(node.adc_values)
    node.destroy_node()
    rclpy.shutdown()

    dmm_ratio = (args.dmm_battery_v / args.dmm_a0_v) if args.dmm_a0_v else None

    row = {
        't_start_ns': t_start_ns,
        't_end_ns': t_end_ns,
        'setpoint_v': args.setpoint_v,
        'dmm_battery_v': args.dmm_battery_v,
        'dmm_a0_v': args.dmm_a0_v,
        'dmm_divider_ratio': dmm_ratio,
        'ros_voltage_mean_v': _mean(voltages),
        'ros_voltage_std_v': _std(voltages),
        'ros_voltage_count': len(voltages),
        'adc_mean': _mean(adc_values),
        'adc_std': _std(adc_values),
        'adc_count': len(adc_values),
        'sample_window_sec': args.sample_window_sec,
        'notes': args.notes,
    }
    _append_row(result_dir.exports_dir / CSV_NAME, row)

    if not voltages:
        # Recorded anyway (the DMM values are real and hard-won), but called out loudly:
        # a row with no ROS samples cannot contribute to the accuracy regression.
        print(f'WARNING: no messages received on {args.battery_topic} during the window. '
              'Is the bench launch running, and is the divider connected to A0?',
              file=sys.stderr)

    print(f"[record_battery_point] setpoint={args.setpoint_v} V  "
          f"DMM V_BAT={args.dmm_battery_v} V  DMM A0={args.dmm_a0_v} V  "
          f"ROS mean={row['ros_voltage_mean_v']}  ADC mean={row['adc_mean']}  "
          f"({len(voltages)} battery / {len(adc_values)} ADC samples)")
    print(f"[record_battery_point] appended to {result_dir.exports_dir / CSV_NAME}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
