#!/usr/bin/env python3
import math
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class SendGoalNode(Node):
    def __init__(self):
        super().__init__('send_goal_node')
        self.declare_parameter('goal_x', 2.2)
        self.declare_parameter('goal_y', 1.8)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('delay_sec', 5.0)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sent = False
        self.last_feedback_log_time = 0.0
        self.timer = self.create_timer(0.5, self.try_send_goal)
        self.start_time = time.monotonic()

    def try_send_goal(self):
        if self.sent:
            return

        delay_sec = float(self.get_parameter('delay_sec').value)
        if time.monotonic() - self.start_time < delay_sec:
            return

        if not self.client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info('Waiting for navigate_to_pose action server...')
            return

        goal_x = float(self.get_parameter('goal_x').value)
        goal_y = float(self.get_parameter('goal_y').value)
        goal_yaw = float(self.get_parameter('goal_yaw').value)
        qz, qw = quaternion_from_yaw(goal_yaw)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_x
        goal.pose.pose.position.y = goal_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending Nav2 goal: x={goal_x:.2f}, y={goal_y:.2f}, yaw={goal_yaw:.2f}'
        )
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        self.sent = True

    def feedback_callback(self, feedback_msg):
        now = time.monotonic()
        if now - self.last_feedback_log_time < 1.0:
            return
        self.last_feedback_log_time = now

        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {distance:.2f} m')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by Nav2.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted by Nav2.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal reached.')
        else:
            self.get_logger().warn(f'Goal finished with status: {status}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = SendGoalNode()
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
