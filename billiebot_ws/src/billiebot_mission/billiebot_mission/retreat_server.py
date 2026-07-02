#!/usr/bin/env python3
"""Retreat action server — backs robot away from the dog by a specified distance."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import Twist
from billiebot_interfaces.action import Retreat


class RetreatServer(Node):
    def __init__(self):
        super().__init__('retreat_server')

        self.declare_parameter('retreat_speed', 0.1)

        self.retreat_speed = float(self.get_parameter('retreat_speed').value)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._action_server = ActionServer(
            self,
            Retreat,
            '/retreat',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('Retreat action server started')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        target_dist = request.retreat_distance
        result = Retreat.Result()
        feedback = Retreat.Feedback()

        # Simple time-based retreat (speed * time = distance)
        duration = target_dist / self.retreat_speed
        rate = self.create_rate(10)
        elapsed = 0.0
        dt = 0.1

        twist = Twist()
        twist.linear.x = -self.retreat_speed  # reverse

        while elapsed < duration:
            if goal_handle.is_cancel_requested:
                # Stop
                self.cmd_pub.publish(Twist())
                goal_handle.canceled()
                result.success = False
                result.message = 'Retreat cancelled'
                result.distance_retreated = elapsed * self.retreat_speed
                return result

            self.cmd_pub.publish(twist)
            feedback.distance_so_far = elapsed * self.retreat_speed
            goal_handle.publish_feedback(feedback)
            elapsed += dt
            await self._sleep(dt)

        # Stop
        self.cmd_pub.publish(Twist())
        goal_handle.succeed()
        result.success = True
        result.message = f'Retreated {target_dist:.2f}m'
        result.distance_retreated = target_dist
        return result

    async def _sleep(self, duration: float):
        import asyncio
        await asyncio.sleep(duration)


def main(args=None):
    rclpy.init(args=args)
    node = RetreatServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
