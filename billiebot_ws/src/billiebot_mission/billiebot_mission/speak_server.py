#!/usr/bin/env python3
"""Speak action server — forwards requests to the speaker_node's /speak action.

Acts as a thin wrapper to maintain uniform action interface (SYS-EXT-1).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse

from billiebot_interfaces.action import Speak


class SpeakServer(Node):
    def __init__(self):
        super().__init__('speak_server')

        self._speaker_client = ActionClient(self, Speak, '/speak')

        self._action_server = ActionServer(
            self,
            Speak,
            '/mission/speak',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Speak action server (mission) started')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        result = Speak.Result()

        if not self._speaker_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result.success = False
            result.message = 'Speaker node not available'
            return result

        # Forward to speaker node
        speaker_goal = Speak.Goal()
        speaker_goal.sound_id = request.sound_id
        speaker_goal.volume = request.volume

        future = self._speaker_client.send_goal_async(speaker_goal)
        speaker_handle = await future

        if not speaker_handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = 'Speaker rejected request (rate-limited?)'
            return result

        speaker_result = await speaker_handle.get_result_async()

        goal_handle.succeed()
        result.success = speaker_result.result.success
        result.message = speaker_result.result.message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SpeakServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
