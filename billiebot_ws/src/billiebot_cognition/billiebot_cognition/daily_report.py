#!/usr/bin/env python3
"""Daily report generator — queries SQLite events for a given day,
aggregates durations/counts, renders Markdown+HTML via Jinja2 with
a matplotlib activity timeline.

Can be run as a ROS node (publishes on timer) or as a standalone CLI
(systemd-timer-friendly).
"""

import os
import sqlite3
import sys
from collections import defaultdict
from datetime import datetime, timedelta

import rclpy
from rclpy.node import Node


REPORT_TEMPLATE_MD = """# BillieBot Daily Report — {{ date }}

## Summary
- **Total observation time:** {{ total_hours }}h {{ total_minutes }}m
- **States observed:** {{ states_summary }}
- **Total bark events:** {{ bark_count }}
- **Rooms visited:** {{ rooms_visited }}

## State Durations
{% for state, duration in state_durations.items() %}
- **{{ state }}:** {{ duration }}
{% endfor %}

## Bark Log
{% if bark_events %}
| Time | Confidence | Room |
|------|-----------|------|
{% for event in bark_events %}
| {{ event.time }} | {{ event.confidence }} | {{ event.room }} |
{% endfor %}
{% else %}
No bark events recorded.
{% endif %}

## Activity Timeline
{{ timeline_note }}

## Rooms Summary
{% for room, count in room_counts.items() %}
- **{{ room }}:** {{ count }} observations
{% endfor %}

---
*Generated {{ generated_at }} by BillieBot v0.1.0*
"""


class DailyReport(Node):
    def __init__(self):
        super().__init__('daily_report')

        self.declare_parameter('db_path', '/var/lib/billiebot/billie_events.db')
        self.declare_parameter('output_dir', '/var/lib/billiebot/reports')
        self.declare_parameter('generate_hour', 23)
        self.declare_parameter('generate_minute', 55)

        self.db_path = str(self.get_parameter('db_path').value)
        self.output_dir = str(self.get_parameter('output_dir').value)

        os.makedirs(self.output_dir, exist_ok=True)

        # Check every minute if it's time to generate
        self.timer = self.create_timer(60.0, self.check_generate)
        self._last_generated_date = None

        self.get_logger().info('DailyReport node started')

    def check_generate(self):
        """Check if it's time to generate the daily report."""
        now = datetime.now()
        target_hour = int(self.get_parameter('generate_hour').value)
        target_minute = int(self.get_parameter('generate_minute').value)

        if (now.hour == target_hour and now.minute == target_minute
                and self._last_generated_date != now.date()):
            self.generate_report(now.date())
            self._last_generated_date = now.date()

    def generate_report(self, date=None):
        """Generate a daily report for the given date."""
        if date is None:
            date = datetime.now().date()

        date_str = date.isoformat()
        self.get_logger().info(f'Generating report for {date_str}')

        try:
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row

            # Query events for the day
            rows = conn.execute(
                """SELECT * FROM dog_events
                   WHERE date(timestamp) = ?
                   ORDER BY timestamp""",
                (date_str,)
            ).fetchall()
            conn.close()
        except sqlite3.Error as e:
            self.get_logger().error(f'DB read error: {e}')
            return

        if not rows:
            self.get_logger().info(f'No events for {date_str}')
            return

        # Aggregate state durations
        state_durations = defaultdict(float)
        bark_events = []
        room_counts = defaultdict(int)

        for i, row in enumerate(rows):
            state = row['state']
            room = row['room'] or 'unknown'
            room_counts[room] += 1

            # Estimate duration until next event
            if i + 1 < len(rows):
                t1 = datetime.fromisoformat(row['timestamp'])
                t2 = datetime.fromisoformat(rows[i + 1]['timestamp'])
                dt = (t2 - t1).total_seconds()
                state_durations[state] += min(dt, 300)  # cap at 5 min

            if state == 'BARKING':
                bark_events.append({
                    'time': row['timestamp'][11:19],
                    'confidence': f"{row['confidence']:.2f}",
                    'room': room,
                })

        # Format durations
        formatted_durations = {}
        for state, secs in state_durations.items():
            hours = int(secs // 3600)
            mins = int((secs % 3600) // 60)
            if hours > 0:
                formatted_durations[state] = f'{hours}h {mins}m'
            else:
                formatted_durations[state] = f'{mins}m'

        total_secs = sum(state_durations.values())
        total_hours = int(total_secs // 3600)
        total_minutes = int((total_secs % 3600) // 60)

        # Render report
        try:
            from jinja2 import Template
            template = Template(REPORT_TEMPLATE_MD)
        except ImportError:
            # Fallback without Jinja2
            template = None

        report_data = {
            'date': date_str,
            'total_hours': total_hours,
            'total_minutes': total_minutes,
            'states_summary': ', '.join(state_durations.keys()),
            'bark_count': len(bark_events),
            'rooms_visited': ', '.join(room_counts.keys()),
            'state_durations': formatted_durations,
            'bark_events': bark_events,
            'room_counts': dict(room_counts),
            'timeline_note': '(See timeline image)',
            'generated_at': datetime.now().isoformat(timespec='seconds'),
        }

        if template:
            report_md = template.render(**report_data)
        else:
            report_md = self._fallback_render(report_data)

        # Write markdown report
        md_path = os.path.join(self.output_dir, f'report_{date_str}.md')
        with open(md_path, 'w') as f:
            f.write(report_md)

        # Generate timeline plot
        self._generate_timeline(rows, date_str)

        self.get_logger().info(f'Report written to {md_path}')

    def _fallback_render(self, data: dict) -> str:
        """Simple string-format fallback if Jinja2 is unavailable."""
        lines = [
            f"# BillieBot Daily Report — {data['date']}",
            f"",
            f"## Summary",
            f"- Total observation time: {data['total_hours']}h {data['total_minutes']}m",
            f"- Total bark events: {data['bark_count']}",
            f"- Rooms visited: {data['rooms_visited']}",
            f"",
            f"## State Durations",
        ]
        for state, dur in data['state_durations'].items():
            lines.append(f"- {state}: {dur}")
        lines.append(f"")
        lines.append(f"Generated {data['generated_at']}")
        return '\n'.join(lines)

    def _generate_timeline(self, rows, date_str: str):
        """Generate a matplotlib timeline plot of states."""
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import matplotlib.dates as mdates

            state_colors = {
                'NOT_FOUND': '#cccccc',
                'SLEEPING': '#3366cc',
                'RESTING': '#66aaff',
                'ACTIVE': '#33cc33',
                'BARKING': '#ff3333',
                'EATING': '#ffaa00',
            }

            fig, ax = plt.subplots(figsize=(14, 2))

            for i, row in enumerate(rows):
                t1 = datetime.fromisoformat(row['timestamp'])
                if i + 1 < len(rows):
                    t2 = datetime.fromisoformat(rows[i + 1]['timestamp'])
                else:
                    t2 = t1 + timedelta(minutes=5)

                state = row['state']
                color = state_colors.get(state, '#999999')
                ax.axvspan(t1, t2, alpha=0.7, color=color, label=state)

            # Remove duplicate legend entries
            handles, labels = ax.get_legend_handles_labels()
            unique = dict(zip(labels, handles))
            ax.legend(unique.values(), unique.keys(), loc='upper right',
                     fontsize=8)

            ax.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M'))
            ax.set_yticks([])
            ax.set_title(f'BillieBot Activity Timeline — {date_str}')
            plt.tight_layout()

            plot_path = os.path.join(
                self.output_dir, f'timeline_{date_str}.png'
            )
            fig.savefig(plot_path, dpi=150)
            plt.close(fig)

        except ImportError:
            self.get_logger().warning(
                'matplotlib not available — skipping timeline plot'
            )


def main(args=None):
    # Support standalone CLI usage
    if '--standalone' in (args or sys.argv):
        date = datetime.now().date()
        if len(sys.argv) > 2:
            date = datetime.fromisoformat(sys.argv[-1]).date()

        rclpy.init(args=[])
        node = DailyReport()
        node.generate_report(date)
        node.destroy_node()
        rclpy.shutdown()
        return

    rclpy.init(args=args)
    node = DailyReport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
