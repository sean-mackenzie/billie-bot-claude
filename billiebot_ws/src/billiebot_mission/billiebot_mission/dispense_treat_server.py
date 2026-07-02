#!/usr/bin/env python3
"""DispenseTreat action server — STUB for future hardware.

Always returns NOT_IMPLEMENTED. Provides the action interface so the
behavior tree and future Behavior AI can reference it (SYS-EXT-1).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from billiebot_interfaces.action import DispenseTreat


class DispenseTreatServer(Node):
    def __init__(self):
        super().__init__('dispense_treat_server')

        self._action_server = ActionServer(
            self,
            DispenseTreat,
            '/dispense_treat',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            'DispenseTreat action server started (STUB — not implemented)'
        )

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        result = DispenseTreat.Result()
        result.success = False
        result.message = 'NOT_IMPLEMENTED — treat dispenser hardware not installed'
        result.dispensed = 0
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DispenseTreatServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
