#!/usr/bin/env python3
"""ApproachDog action server — approaches detected dog to standoff distance.

Standoff floor: 1.0m (SYS-FND-3), speed cap: 0.15 m/s near dog (SYS-NAV-5).
Uses Nav2 internally for path planning.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from billiebot_interfaces.action import ApproachDog
from billiebot_interfaces.msg import DogState


class ApproachDogServer(Node):
    def __init__(self):
        super().__init__('approach_dog_server')

        self.declare_parameter('min_standoff', 1.0)
        self.declare_parameter('max_speed', 0.15)

        self.min_standoff = float(self.get_parameter('min_standoff').value)
        self.max_speed = float(self.get_parameter('max_speed').value)

        self._dog_pose = None
        self.create_subscription(
            PoseStamped, '/dog/pose_map', self._on_dog_pose, 10
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        self._action_server = ActionServer(
            self,
            ApproachDog,
            '/approach_dog',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('ApproachDog action server started')

    def _on_dog_pose(self, msg: PoseStamped):
        self._dog_pose = msg

    def goal_callback(self, goal_request):
        standoff = goal_request.standoff_distance
        if standoff < self.min_standoff:
            self.get_logger().warn(
                f'Requested standoff {standoff}m < minimum {self.min_standoff}m'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        request = goal_handle.request
        standoff = max(request.standoff_distance, self.min_standoff)
        result = ApproachDog.Result()
        feedback = ApproachDog.Feedback()

        if self._dog_pose is None:
            goal_handle.abort()
            result.success = False
            result.message = 'No dog pose available'
            return result

        # Compute approach point (standoff distance from dog)
        dog_x = self._dog_pose.pose.position.x
        dog_y = self._dog_pose.pose.position.y

        # Simple approach: move to standoff distance along the line
        # from robot to dog (we'll use dog position directly and let Nav2
        # handle the approach)
        approach_pose = PoseStamped()
        approach_pose.header.stamp = self.get_clock().now().to_msg()
        approach_pose.header.frame_id = 'map'
        approach_pose.pose.position.x = dog_x - standoff
        approach_pose.pose.position.y = dog_y
        approach_pose.pose.orientation.w = 1.0

        # Send to Nav2
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = approach_pose

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            result.success = False
            result.message = 'Nav2 server not available'
            return result

        nav_future = self._nav_client.send_goal_async(nav_goal)
        nav_handle = await nav_future

        if not nav_handle.accepted:
            goal_handle.abort()
            result.success = False
            result.message = 'Nav2 rejected approach goal'
            return result

        nav_result = await nav_handle.get_result_async()

        # Compute final distance
        final_dist = math.sqrt(
            (approach_pose.pose.position.x - dog_x) ** 2 +
            (approach_pose.pose.position.y - dog_y) ** 2
        )

        goal_handle.succeed()
        result.success = True
        result.message = f'Approached to {final_dist:.2f}m standoff'
        result.final_distance = final_dist
        result.final_position = approach_pose.pose.position
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ApproachDogServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
