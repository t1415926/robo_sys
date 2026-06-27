#!/usr/bin/env python3
"""BUNKER MINI ROS2 SocketCAN driver.

Protocol from the BUNKER MINI user manual:
- 0x421: control mode setting frame, byte0 = 0x01 for CAN command mode.
- 0x111: motion command frame, sent every 20 ms.
  byte0..1: signed int16 linear velocity, mm/s, big endian.
  byte2..3: signed int16 angular velocity, 0.001 rad/s, big endian.
  byte4..7: reserved 0x00.
"""

import socket
import struct

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000


def clamp(value, limit):
    return max(-limit, min(limit, value))


class BunkerMiniCanDriver(Node):
    def __init__(self):
        super().__init__('bunker_mini_can_driver')

        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_linear_x', 0.30)
        self.declare_parameter('max_angular_z', 0.60)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('send_rate', 50.0)
        self.declare_parameter('mode_refresh_rate', 1.0)
        self.declare_parameter('invert_linear_x', False)
        self.declare_parameter('invert_angular_z', False)

        self.can_interface = self.get_parameter('can_interface').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.max_linear_x = abs(float(self.get_parameter('max_linear_x').value))
        self.max_angular_z = abs(float(self.get_parameter('max_angular_z').value))
        self.cmd_timeout = max(0.1, float(self.get_parameter('cmd_timeout').value))
        self.invert_linear_x = bool(self.get_parameter('invert_linear_x').value)
        self.invert_angular_z = bool(self.get_parameter('invert_angular_z').value)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_mode_time = self.get_clock().now()

        self.can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can_socket.bind((self.can_interface,))

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10,
        )

        send_rate = max(1.0, float(self.get_parameter('send_rate').value))
        self.mode_refresh_period = 1.0 / max(
            0.1,
            float(self.get_parameter('mode_refresh_rate').value),
        )
        self.timer = self.create_timer(1.0 / send_rate, self.send_control_frame)

        self.send_mode_frame()
        self.send_motion_frame(0.0, 0.0)
        self.get_logger().info(
            f'BUNKER MINI CAN driver listening on {self.cmd_vel_topic}, '
            f'sending commands to {self.can_interface}.'
        )

    def cmd_callback(self, msg):
        linear_x = clamp(float(msg.linear.x), self.max_linear_x)
        angular_z = clamp(float(msg.angular.z), self.max_angular_z)

        if self.invert_linear_x:
            linear_x *= -1.0
        if self.invert_angular_z:
            angular_z *= -1.0

        self.linear_x = linear_x
        self.angular_z = angular_z
        self.last_cmd_time = self.get_clock().now()

    def send_control_frame(self):
        now = self.get_clock().now()
        mode_age = (now - self.last_mode_time).nanoseconds * 1e-9
        if mode_age >= self.mode_refresh_period:
            self.send_mode_frame()

        cmd_age = (now - self.last_cmd_time).nanoseconds * 1e-9
        if cmd_age > self.cmd_timeout:
            self.send_motion_frame(0.0, 0.0)
        else:
            self.send_motion_frame(self.linear_x, self.angular_z)

    def send_mode_frame(self):
        self.send_can_frame(0x421, bytes([0x01]))
        self.last_mode_time = self.get_clock().now()

    def send_motion_frame(self, linear_x, angular_z):
        linear_mm_s = int(round(linear_x * 1000.0))
        angular_mrad_s = int(round(angular_z * 1000.0))

        linear_mm_s = int(clamp(linear_mm_s, 32767))
        angular_mrad_s = int(clamp(angular_mrad_s, 32767))

        payload = struct.pack('>hhBBBB', linear_mm_s, angular_mrad_s, 0, 0, 0, 0)
        self.send_can_frame(0x111, payload)

    def send_can_frame(self, can_id, payload):
        payload = payload[:8]
        can_dlc = len(payload)
        data = payload.ljust(8, b'\x00')
        frame = struct.pack('=IB3x8s', can_id & 0x1FFFFFFF, can_dlc, data)
        self.can_socket.send(frame)

    def destroy_node(self):
        try:
            self.send_motion_frame(0.0, 0.0)
            self.can_socket.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BunkerMiniCanDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
