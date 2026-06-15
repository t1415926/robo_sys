#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class SendPoseGoalNode(Node):
    def __init__(self):
        super().__init__('send_pose_goal_node')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('goal_x', 2.2)
        self.declare_parameter('goal_y', 1.8)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('delay_sec', 2.0)

        self.publisher = self.create_publisher(
            PoseStamped,
            self.get_parameter('goal_topic').value,
            10,
        )
        self.start_time = time.monotonic()
        self.sent = False
        self.timer = self.create_timer(0.2, self.try_send)

    def try_send(self):
        if self.sent:
            return
        if time.monotonic() - self.start_time < float(self.get_parameter('delay_sec').value):
            return

        goal_x = float(self.get_parameter('goal_x').value)
        goal_y = float(self.get_parameter('goal_y').value)
        goal_yaw = float(self.get_parameter('goal_yaw').value)
        qz, qw = quaternion_from_yaw(goal_yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.publisher.publish(msg)
        self.sent = True
        self.get_logger().info(
            f'Published pose goal to {self.get_parameter("goal_topic").value}: '
            f'x={goal_x:.2f}, y={goal_y:.2f}, yaw={goal_yaw:.2f}'
        )


def main():
    rclpy.init()
    node = SendPoseGoalNode()
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
