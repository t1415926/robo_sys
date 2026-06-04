#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class DiffDriveInterfaceBridge(Node):
    def __init__(self):
        super().__init__('diff_drive_interface_bridge')

        self.declare_parameter('cmd_vel_in', '/cmd_vel')
        self.declare_parameter('cmd_vel_out', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('controller_odom_in', '/diff_drive_controller/odom')
        self.declare_parameter('odom_out', '/odom')

        self.cmd_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_out').value,
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_out').value,
            10,
        )

        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_in').value,
            self.cmd_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter('controller_odom_in').value,
            self.odom_callback,
            10,
        )

    def cmd_callback(self, msg):
        self.cmd_pub.publish(msg)

    def odom_callback(self, msg):
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = DiffDriveInterfaceBridge()
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
