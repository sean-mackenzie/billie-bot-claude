#!/usr/bin/env python3
"""Speaker action server node.

Real mode: plays audio files via I2S output through MAX98357A amplifier.
Mock mode: logs playback requests only.

Rate-limited per SYS-PLT-6 (dog-welfare constraint — no sudden loud sounds).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from billiebot_interfaces.action import Speak


class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker_node')

        self.declare_parameter('mock', False)
        self.declare_parameter('sounds_dir', '')
        self.declare_parameter('max_volume', 0.5)
        self.declare_parameter('min_interval_sec', 10.0)
        self.declare_parameter('fade_in_sec', 0.5)

        self.mock = bool(self.get_parameter('mock').value)
        self.max_volume = float(self.get_parameter('max_volume').value)
        self.min_interval = float(self.get_parameter('min_interval_sec').value)
        self.sounds_dir = str(self.get_parameter('sounds_dir').value)

        self._last_play_time = 0.0

        self._action_server = ActionServer(
            self,
            Speak,
            '/speak',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        mode_str = 'MOCK' if self.mock else 'REAL'
        self.get_logger().info(f'Speaker node started ({mode_str})')

    def goal_callback(self, goal_request):
        # Rate limiting (SYS-PLT-6)
        elapsed = time.monotonic() - self._last_play_time
        if elapsed < self.min_interval:
            self.get_logger().info(
                f'Speaker rate-limited: {self.min_interval - elapsed:.1f}s remaining'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        sound_id = request.sound_id
        volume = min(request.volume, self.max_volume)

        result = Speak.Result()
        feedback = Speak.Feedback()

        self.get_logger().info(
            f'Playing sound: {sound_id} at volume {volume:.2f}'
        )

        if self.mock:
            # Simulate playback over 1 second
            for i in range(10):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Playback cancelled'
                    return result

                feedback.progress = (i + 1) / 10.0
                goal_handle.publish_feedback(feedback)
                await self._sleep(0.1)

            self._last_play_time = time.monotonic()
            goal_handle.succeed()
            result.success = True
            result.message = f'Mock playback complete: {sound_id}'
            return result

        # Real playback
        try:
            import os
            import subprocess

            sound_path = os.path.join(self.sounds_dir, sound_id)
            if not os.path.isfile(sound_path):
                goal_handle.abort()
                result.success = False
                result.message = f'Sound file not found: {sound_path}'
                return result

            # Use aplay for WAV files via ALSA
            vol_pct = int(volume * 100)
            proc = subprocess.Popen(
                ['aplay', '-D', 'plughw:0,0', sound_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )

            feedback.progress = 0.5
            goal_handle.publish_feedback(feedback)

            proc.wait(timeout=30)

            self._last_play_time = time.monotonic()
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result.success = True
            result.message = f'Playback complete: {sound_id}'
            return result

        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.message = f'Playback error: {e}'
            return result

    async def _sleep(self, duration: float):
        """Async-compatible sleep."""
        import asyncio
        await asyncio.sleep(duration)


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
