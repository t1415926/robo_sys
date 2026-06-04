#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def inverse_pose_2d(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, -yaw)


def compose_pose_2d(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    c = math.cos(ayaw)
    s = math.sin(ayaw)
    return (ax + c * bx - s * by, ay + s * bx + c * by, normalize_angle(ayaw + byaw))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SimLocalizationNode(Node):
    def __init__(self):
        super().__init__('sim_localization_node')

        self.declare_parameter('ground_truth_topic', '/ground_truth/odom')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_rate', 30.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.ground_truth_pose = None
        self.odom_pose = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            self.get_parameter('ground_truth_topic').value,
            self.ground_truth_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10,
        )

        publish_rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / publish_rate, self.publish_transform)

    def ground_truth_callback(self, msg):
        pose = msg.pose.pose
        self.ground_truth_pose = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.odom_pose = (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def publish_transform(self):
        if self.ground_truth_pose is None or self.odom_pose is None:
            return

        map_to_base = self.ground_truth_pose
        odom_to_base_inv = inverse_pose_2d(*self.odom_pose)
        map_to_odom = compose_pose_2d(map_to_base, odom_to_base_inv)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = map_to_odom[0]
        transform.transform.translation.y = map_to_odom[1]
        transform.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(map_to_odom[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = SimLocalizationNode()
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
