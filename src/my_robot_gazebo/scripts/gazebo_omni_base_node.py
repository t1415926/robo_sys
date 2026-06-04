#!/usr/bin/env python3
import math

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboOmniBaseNode(Node):
    def __init__(self):
        super().__init__('gazebo_omni_base_node')
        self.declare_parameter('model_name', 'simple_omni_robot')
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_stamped_topic', '/cmd_vel_stamped')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('set_entity_state_service', '/set_entity_state')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.08)
        self.declare_parameter('initial_yaw', 0.0)

        self.model_name = self.get_parameter('model_name').value
        self.reference_frame = self.get_parameter('reference_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.z = float(self.get_parameter('initial_z').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.pending_state_future = None

        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.set_state_client = self.create_client(
            SetEntityState,
            self.get_parameter('set_entity_state_service').value,
        )

        self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            self.cmd_callback,
            10,
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter('cmd_vel_stamped_topic').value,
            self.cmd_stamped_callback,
            10,
        )

        publish_rate = float(self.get_parameter('publish_rate').value)
        self.publish_state(self.get_clock().now())
        self.create_timer(1.0 / publish_rate, self.update)

    def cmd_callback(self, msg):
        self.set_command(msg)

    def cmd_stamped_callback(self, msg):
        self.set_command(msg.twist)

    def set_command(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0:
            return

        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout:
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        world_vx = cos_yaw * self.vx - sin_yaw * self.vy
        world_vy = sin_yaw * self.vx + cos_yaw * self.vy

        self.x += world_vx * dt
        self.y += world_vy * dt
        self.yaw = normalize_angle(self.yaw + self.wz * dt)

        self.publish_state(now)
        self.set_gazebo_state()

    def publish_state(self, stamp):
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)

        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

    def set_gazebo_state(self):
        if not self.set_state_client.service_is_ready():
            return
        if self.pending_state_future is not None and not self.pending_state_future.done():
            return

        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)

        state = EntityState()
        state.name = self.model_name
        state.reference_frame = self.reference_frame
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        state.twist.linear.x = self.vx
        state.twist.linear.y = self.vy
        state.twist.angular.z = self.wz

        request = SetEntityState.Request()
        request.state = state
        self.pending_state_future = self.set_state_client.call_async(request)


def main():
    rclpy.init()
    node = GazeboOmniBaseNode()
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
