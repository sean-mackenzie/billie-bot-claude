#!/usr/bin/env python3
"""SQLite dog event logger.

Logs every state transition and detection event with:
  timestamp, state, confidence, x, y, room, image_path, context_json, action, outcome

Uses WAL mode for crash safety (SYS-STL-4).
Captures snapshot JPEGs on state transitions.
Records (context, action, outcome) tuples for future bandit bootstrap (SYS-EXT-3).
"""

import json
import os
import sqlite3
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from billiebot_interfaces.msg import DogState


STATE_NAMES = {
    0: 'NOT_FOUND',
    1: 'SLEEPING',
    2: 'RESTING',
    3: 'ACTIVE',
    4: 'BARKING',
    5: 'EATING',
}

DB_SCHEMA = """
CREATE TABLE IF NOT EXISTS dog_events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TEXT NOT NULL,
    epoch REAL NOT NULL,
    state TEXT NOT NULL,
    state_id INTEGER NOT NULL,
    confidence REAL NOT NULL,
    x REAL,
    y REAL,
    room TEXT,
    image_path TEXT,
    context_json TEXT,
    action TEXT DEFAULT 'OBSERVE',
    outcome TEXT DEFAULT '',
    stress_proxy REAL DEFAULT 0.0
);

CREATE INDEX IF NOT EXISTS idx_timestamp ON dog_events(timestamp);
CREATE INDEX IF NOT EXISTS idx_state ON dog_events(state);
"""


class DogLogger(Node):
    def __init__(self):
        super().__init__('dog_logger')

        self.declare_parameter('db_path', '/var/lib/billiebot/billie_events.db')
        self.declare_parameter('snapshot_dir', '/var/lib/billiebot/snapshots')
        self.declare_parameter('rooms_config', '')
        self.declare_parameter('enable_snapshots', True)

        self.db_path = str(self.get_parameter('db_path').value)
        self.snapshot_dir = str(self.get_parameter('snapshot_dir').value)
        self.enable_snapshots = bool(
            self.get_parameter('enable_snapshots').value
        )

        # Ensure directories exist
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        if self.enable_snapshots:
            os.makedirs(self.snapshot_dir, exist_ok=True)

        # Initialize database
        self._conn = sqlite3.connect(self.db_path)
        self._conn.execute('PRAGMA journal_mode=WAL')
        self._conn.executescript(DB_SCHEMA)
        self._conn.commit()

        # State tracking
        self._last_state = None
        self._last_log_time = 0.0

        # Load room definitions
        self._rooms = self._load_rooms()

        # Subscribe to fused state
        self.create_subscription(
            DogState, '/billie/state', self._on_state, 10
        )

        self.get_logger().info(f'DogLogger started — DB: {self.db_path}')

    def _load_rooms(self) -> dict:
        """Load room boundary definitions from config."""
        rooms_config = str(self.get_parameter('rooms_config').value)
        if not rooms_config or not os.path.isfile(rooms_config):
            return {}
        try:
            import yaml
            with open(rooms_config, 'r') as f:
                data = yaml.safe_load(f)
            return data.get('rooms', {})
        except Exception as e:
            self.get_logger().warning(f'Failed to load rooms config: {e}')
            return {}

    def _position_to_room(self, x: float, y: float) -> str:
        """Map (x, y) position to a room name using configured boundaries."""
        for room_name, bounds in self._rooms.items():
            x_min = bounds.get('x_min', float('-inf'))
            x_max = bounds.get('x_max', float('inf'))
            y_min = bounds.get('y_min', float('-inf'))
            y_max = bounds.get('y_max', float('inf'))
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return room_name
        return 'unknown'

    def _on_state(self, msg: DogState):
        """Handle incoming state — log on transitions."""
        current_state = msg.state

        # Log on state transitions or periodic (every 60s)
        is_transition = current_state != self._last_state
        is_periodic = (time.monotonic() - self._last_log_time) > 60.0

        if not is_transition and not is_periodic:
            return

        self._last_state = current_state
        self._last_log_time = time.monotonic()

        # Determine room
        room = msg.room
        if not room and msg.position:
            room = self._position_to_room(msg.position.x, msg.position.y)

        # Snapshot on transitions
        image_path = ''
        if is_transition and self.enable_snapshots:
            image_path = self._capture_snapshot(current_state)

        # Build context JSON (SYS-EXT-3)
        context_json = json.dumps({
            'context': list(msg.context),
            'stress_proxy': msg.stress_proxy,
            'state_duration': msg.state_duration,
        })

        # Insert event
        now = datetime.now()
        try:
            self._conn.execute(
                """INSERT INTO dog_events
                   (timestamp, epoch, state, state_id, confidence, x, y,
                    room, image_path, context_json, action, outcome,
                    stress_proxy)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                (
                    now.isoformat(),
                    time.time(),
                    STATE_NAMES.get(current_state, 'UNKNOWN'),
                    current_state,
                    msg.confidence,
                    msg.position.x if msg.position else None,
                    msg.position.y if msg.position else None,
                    room,
                    image_path,
                    context_json,
                    'OBSERVE',
                    '',
                    msg.stress_proxy,
                ),
            )
            self._conn.commit()

            if is_transition:
                self.get_logger().info(
                    f'Logged transition → {STATE_NAMES.get(current_state)} '
                    f'(conf={msg.confidence:.2f}, room={room})'
                )
        except sqlite3.Error as e:
            self.get_logger().error(f'DB write error: {e}')

    def _capture_snapshot(self, state: int) -> str:
        """Capture a JPEG snapshot for the state transition."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        state_name = STATE_NAMES.get(state, 'unknown')
        filename = f'{timestamp}_{state_name}.jpg'
        filepath = os.path.join(self.snapshot_dir, filename)
        # In a real system, we'd subscribe to an image topic and save
        # For now, create a placeholder
        try:
            with open(filepath, 'wb') as f:
                f.write(b'')  # Placeholder
            return filepath
        except Exception:
            return ''

    def destroy_node(self):
        if hasattr(self, '_conn'):
            self._conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DogLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
