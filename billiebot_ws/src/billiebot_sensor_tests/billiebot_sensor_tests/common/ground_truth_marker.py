"""Operator-entered ground truth, stdin-driven (deliberately no new ROS service/message —
see plan assumption 8). Reads lines like:

    mark dog_present distance=1.5m orientation=front condition=normal_light notes=settled

from stdin on a background thread and appends timestamped rows to
exports/ground_truth_segments.csv. The `label`/`distance`/etc. free-form values are stored
with their operator-entered unit suffix intact (e.g. "1.5m", "45deg") — see
`parse_quantity` for the downstream numeric-parsing helper scoring CLIs use.
"""

import csv
import re
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node

_LINE_RE = re.compile(r'^mark\s+(\S+)(.*)$')
_KV_RE = re.compile(r'(\w+)=(\S+)')
_QUANTITY_RE = re.compile(r'^[-+]?\d*\.?\d+')

FIELDNAMES = ['t_start_ns', 'label', 'distance_m', 'orientation', 'condition', 'notes']


def parse_mark_line(line: str):
    """Pure parser: 'mark dog_present distance=1.5m orientation=front' -> dict, or None
    if the line doesn't match the `mark <label> [key=value ...]` grammar."""
    line = line.strip()
    m = _LINE_RE.match(line)
    if not m:
        return None
    label = m.group(1)
    kv = dict(_KV_RE.findall(m.group(2)))
    return {'label': label, **kv}


def parse_quantity(value) -> float:
    """Strips a trailing unit suffix (e.g. '1.5m' -> 1.5, '45deg' -> 45.0) and parses the
    leading numeric portion. Raises ValueError if no leading number is present."""
    text = str(value).strip()
    m = _QUANTITY_RE.match(text)
    if not m:
        raise ValueError(f"no numeric quantity found in '{value}'")
    return float(m.group(0))


def load_ground_truth_segments(result_dir, default_duration_ns: int = int(30e9)) -> list:
    """Read exports/ground_truth_segments.csv into `[{'label', 't_start_ns', 't_end_ns',
    'row'}, ...]`, or `[]` when the operator marked nothing.

    End-time resolution matches the rule score_thermal_blob.py already uses: an explicit
    `t_end_ns` column wins, otherwise a segment runs until the next mark, and the final
    segment gets `default_duration_ns`. Labels are lower-cased and stripped so the operator's
    typing is not part of the contract.
    """
    path = Path(result_dir.exports_dir) / 'ground_truth_segments.csv'
    if not path.exists():
        return []

    with open(path, 'r') as f:
        rows = list(csv.DictReader(f))

    segments = []
    for i, row in enumerate(rows):
        try:
            t_start = int(row['t_start_ns'])
        except (KeyError, TypeError, ValueError):
            continue
        if row.get('t_end_ns'):
            t_end = int(row['t_end_ns'])
        elif i + 1 < len(rows):
            t_end = int(rows[i + 1]['t_start_ns'])
        else:
            t_end = t_start + int(default_duration_ns)
        segments.append({
            'label': (row.get('label') or '').strip().lower(),
            't_start_ns': t_start,
            't_end_ns': t_end,
            'row': row,
        })
    return segments


class GroundTruthMarkerNode(Node):
    def __init__(self):
        super().__init__('ground_truth_marker_node')
        self.declare_parameter('output_csv', '')
        output_csv = str(self.get_parameter('output_csv').value)
        self._csv_path = Path(output_csv)
        self._csv_path.parent.mkdir(parents=True, exist_ok=True)
        if not self._csv_path.exists():
            with open(self._csv_path, 'w', newline='') as f:
                csv.DictWriter(f, fieldnames=FIELDNAMES).writeheader()

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_stdin_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(
            "Ground truth marker ready. Type e.g. 'mark dog_present distance=1.5m "
            "orientation=front condition=normal_light' and press Enter."
        )

    def _read_stdin_loop(self):
        for line in sys.stdin:
            if self._stop.is_set():
                return
            parsed = parse_mark_line(line)
            if parsed is None:
                if line.strip():
                    self.get_logger().warning(f'unrecognized ground-truth line: {line.strip()!r}')
                continue
            self._append_row(parsed)

    def _append_row(self, parsed: dict):
        row = {
            't_start_ns': self.get_clock().now().nanoseconds,
            'label': parsed.get('label', ''),
            'distance_m': parsed.get('distance', ''),
            'orientation': parsed.get('orientation', ''),
            'condition': parsed.get('condition', ''),
            'notes': parsed.get('notes', ''),
        }
        with open(self._csv_path, 'a', newline='') as f:
            csv.DictWriter(f, fieldnames=FIELDNAMES).writerow(row)
        self.get_logger().info(f'ground truth marked: {row}')

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
