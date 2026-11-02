#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        # Side length of the square in meters
        self.declare_parameter('side_length', 0.5)
        self.side_length = (
            self.get_parameter('side_length')
            .get_parameter_value()
            .double_value
        )

        # Action clients
        self.drive_client = ActionClient(
            self,
            DriveDistance,
            '/drive_distance'
        )

        self.rotate_client = ActionClient(
            self,
            Rotate,
            '/rotate'
        )

    def execute_goal(self, client, goal_msg, action_name, side_number):
        """
        Send one action goal and wait for its result.

        Returns True if the goal succeeds.
        Returns False if the goal is rejected or fails.
        """

        self.get_logger().info(
            f'Side {side_number}: Waiting for {action_name} server...'
        )

        # Wait until the action server is available
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                f'Side {side_number}: {action_name} server unavailable.'
            )
            return False

        self.get_logger().info(
            f'Side {side_number}: Sending {action_name} goal.'
        )

        # Send goal
        goal_future = client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        goal_handle = goal_future.result()

        # Check whether the server accepted the goal
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f'Side {side_number}: {action_name} goal rejected.'
            )
            return False

        self.get_logger().info(
            f'Side {side_number}: {action_name} goal accepted.'
        )

        # Wait for the action result
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        response = result_future.result()

        if response is None:
            self.get_logger().error(
                f'Side {side_number}: No result from {action_name}.'
            )
            return False

        # Check the action status
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Side {side_number}: {action_name} succeeded.'
            )
            return True

        self.get_logger().error(
            f'Side {side_number}: {action_name} failed '
            f'with status {response.status}.'
        )

        return False

    def drive_square(self):

        # Four sides, each followed by a 90-degree left turn
        for side_number in range(1, 5):

            self.get_logger().info(
                f'===== Starting side {side_number}/4 ====='
            )

            # 1. Drive forward one side length
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = self.side_length

            drive_success = self.execute_goal(
                self.drive_client,
                drive_goal,
                '/drive_distance',
                side_number
            )

            # Stop immediately if driving fails
            if not drive_success:
                self.get_logger().error(
                    f'Square aborted on side {side_number}: '
                    f'drive failed.'
                )
                return False

            # 2. Rotate left by 90 degrees
            rotate_goal = Rotate.Goal()
            rotate_goal.angle = math.pi / 2.0

            rotate_success = self.execute_goal(
                self.rotate_client,
                rotate_goal,
                '/rotate',
                side_number
            )

            # Stop immediately if rotation fails
            if not rotate_success:
                self.get_logger().error(
                    f'Square aborted on side {side_number}: '
                    f'rotation failed.'
                )
                return False

        self.get_logger().info(
            'Square completed successfully! '
            'The robot should be close to its starting position.'
        )

        return True


def main(args=None):

    rclpy.init(args=args)

    square_driver = SquareDriver()

    try:
        square_driver.drive_square()

    except KeyboardInterrupt:
        square_driver.get_logger().info(
            'Square driving interrupted by user.'
        )

    finally:
        square_driver.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()