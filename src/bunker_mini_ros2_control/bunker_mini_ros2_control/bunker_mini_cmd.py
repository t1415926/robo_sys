#!/usr/bin/env python3
"""Publish low-speed ROS2 Twist commands for a BUNKER MINI chassis.

This node does not talk to CAN directly. It publishes geometry_msgs/Twist to the
topic consumed by your ROS2 chassis driver or ROS1 bridge. BUNKER MINI field
tests should start with the vehicle lifted or in an empty area.
"""

import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MotionStage(Enum):
    FORWARD = 1
    STOP_AFTER_FORWARD = 2
    TURN_LEFT = 3
    FINAL_STOP = 4
    DONE = 5


class BunkerMiniCmd(Node):
    def __init__(self):
        super().__init__('bunker_mini_cmd')

        self.declare_parameter('cmd_vel_topic', '/smoother_cmd_vel')
        self.declare_parameter('linear_speed', 0.05)
        self.declare_parameter('angular_speed', 0.20)
        self.declare_parameter('forward_duration', 2.0)
        self.declare_parameter('turn_duration', 2.0)
        self.declare_parameter('stop_duration', 1.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('repeat_demo', False)

        topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = abs(float(self.get_parameter('linear_speed').value))
        self.angular_speed = abs(float(self.get_parameter('angular_speed').value))
        self.forward_duration = max(0.0, float(self.get_parameter('forward_duration').value))
        self.turn_duration = max(0.0, float(self.get_parameter('turn_duration').value))
        self.stop_duration = max(0.2, float(self.get_parameter('stop_duration').value))
        self.repeat_demo = bool(self.get_parameter('repeat_demo').value)

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.stage = MotionStage.FORWARD
        self.stage_start = self.get_clock().now()

        publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / publish_rate, self.on_timer)

        self.get_logger().info(
            f'Publishing safe BUNKER MINI demo commands to {topic}. '
            'Keep emergency stop reachable.'
        )

    def on_timer(self):
        elapsed = (self.get_clock().now() - self.stage_start).nanoseconds * 1e-9

        if self.stage == MotionStage.FORWARD:
            self.publish_cmd(linear_x=self.linear_speed)
            if elapsed >= self.forward_duration:
                self.next_stage(MotionStage.STOP_AFTER_FORWARD)
        elif self.stage == MotionStage.STOP_AFTER_FORWARD:
            self.publish_stop()
            if elapsed >= self.stop_duration:
                self.next_stage(MotionStage.TURN_LEFT)
        elif self.stage == MotionStage.TURN_LEFT:
            self.publish_cmd(angular_z=self.angular_speed)
            if elapsed >= self.turn_duration:
                self.next_stage(MotionStage.FINAL_STOP)
        elif self.stage == MotionStage.FINAL_STOP:
            self.publish_stop()
            if elapsed >= self.stop_duration:
                if self.repeat_demo:
                    self.next_stage(MotionStage.FORWARD)
                else:
                    self.next_stage(MotionStage.DONE)
        else:
            self.publish_stop()

    def next_stage(self, stage):
        self.stage = stage
        self.stage_start = self.get_clock().now()
        self.get_logger().info(f'Motion stage: {stage.name.lower()}')

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def publish_stop(self):
        self.publisher.publish(Twist())

    def destroy_node(self):
        for _ in range(5):
            self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BunkerMiniCmd()
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
