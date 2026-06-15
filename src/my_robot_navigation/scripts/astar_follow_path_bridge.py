#!/usr/bin/env python3
import time

import rclpy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class AStarFollowPathBridge(Node):
    def __init__(self):
        super().__init__('astar_follow_path_bridge')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('follow_path_action', 'follow_path')
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'general_goal_checker')
        self.declare_parameter('min_path_poses', 2)
        self.declare_parameter('resend_min_interval', 0.5)

        self.client = ActionClient(
            self,
            FollowPath,
            self.get_parameter('follow_path_action').value,
        )
        self.goal_handle = None
        self.last_send_time = 0.0
        self.last_feedback_time = 0.0

        self.create_subscription(
            Path,
            self.get_parameter('plan_topic').value,
            self.plan_callback,
            10,
        )
        self.get_logger().info(
            f'Waiting for FollowPath action: {self.get_parameter("follow_path_action").value}'
        )

    def plan_callback(self, path):
        min_path_poses = int(self.get_parameter('min_path_poses').value)
        if len(path.poses) < min_path_poses:
            self.get_logger().warn(
                f'Ignoring path with {len(path.poses)} poses; need at least {min_path_poses}.'
            )
            return

        now = time.monotonic()
        resend_min_interval = float(self.get_parameter('resend_min_interval').value)
        if now - self.last_send_time < resend_min_interval:
            return

        if not self.client.wait_for_server(timeout_sec=0.1):
            self.get_logger().warn('FollowPath action server is not available yet.')
            return

        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.get_parameter('controller_id').value
        goal.goal_checker_id = self.get_parameter('goal_checker_id').value

        self.last_send_time = now
        self.get_logger().info(
            f'Sending path with {len(path.poses)} poses to Nav2 controller '
            f'({goal.controller_id}, {goal.goal_checker_id}).'
        )
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.goal_handle = None
            self.get_logger().error('FollowPath goal was rejected by controller_server.')
            return

        self.get_logger().info('FollowPath goal accepted.')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        now = time.monotonic()
        if now - self.last_feedback_time < 1.0:
            return
        self.last_feedback_time = now
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'FollowPath feedback: distance={feedback.distance_to_goal:.2f} m, '
            f'speed={feedback.speed:.2f} m/s'
        )

    def result_callback(self, future):
        status = future.result().status
        self.goal_handle = None
        if status == 4:
            self.get_logger().info('FollowPath completed.')
        else:
            self.get_logger().warn(f'FollowPath finished with status: {status}')


def main():
    rclpy.init()
    node = AStarFollowPathBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
