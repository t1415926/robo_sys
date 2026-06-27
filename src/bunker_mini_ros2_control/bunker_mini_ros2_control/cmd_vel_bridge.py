#!/usr/bin/env python3
"""Bridge planner /cmd_vel commands to the BUNKER MINI base driver topic."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def clamp(value, limit):
    return max(-limit, min(limit, value))


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('bunker_mini_cmd_vel_bridge')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/smoother_cmd_vel')
        self.declare_parameter('max_linear_x', 0.30)
        self.declare_parameter('max_linear_y', 0.0)
        self.declare_parameter('max_angular_z', 0.60)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('invert_linear_x', False)
        self.declare_parameter('invert_angular_z', False)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self.max_linear_y = abs(float(self.get_parameter('max_linear_y').value))
        self.max_angular_z = abs(float(self.get_parameter('max_angular_z').value))
        self.cmd_timeout = max(0.1, float(self.get_parameter('cmd_timeout').value))
        self.invert_linear_x = bool(self.get_parameter('invert_linear_x').value)
        self.invert_angular_z = bool(self.get_parameter('invert_angular_z').value)

        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.warned_timeout = False

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist,
            self.input_topic,
            self.cmd_callback,
            10,
        )

        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_safe_cmd)

        self.get_logger().info(
            f'Listening to planner commands on {self.input_topic}, '
            f'publishing protected chassis commands to {self.output_topic}.'
        )

    def cmd_callback(self, msg):
        safe = Twist()
        safe.linear.x = clamp(float(msg.linear.x), self.max_linear_x)
        safe.linear.y = clamp(float(msg.linear.y), self.max_linear_y)
        safe.angular.z = clamp(float(msg.angular.z), self.max_angular_z)

        if self.invert_linear_x:
            safe.linear.x *= -1.0
        if self.invert_angular_z:
            safe.angular.z *= -1.0

        if not self.is_finite(safe.linear.x, safe.linear.y, safe.angular.z):
            self.get_logger().warn('Ignored non-finite /cmd_vel command.')
            return

        self.last_cmd = safe
        self.last_cmd_time = self.get_clock().now()
        self.warned_timeout = False

    def publish_safe_cmd(self):
        now = self.get_clock().now()
        age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self.publisher.publish(Twist())
            if not self.warned_timeout:
                self.get_logger().warn(
                    f'No {self.input_topic} command for {age:.2f}s; sending stop.'
                )
                self.warned_timeout = True
            return

        self.publisher.publish(self.last_cmd)

    def is_finite(self, *values):
        return all(math.isfinite(value) for value in values)

    def destroy_node(self):
        for _ in range(5):
            self.publisher.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
