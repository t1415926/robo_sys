#!/usr/bin/env python3
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker, UVCoordinate


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ImageToPlaneMarker(Node):
    def __init__(self):
        super().__init__('image_to_plane_marker')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('marker_topic', '/camera_projection_marker')
        self.declare_parameter('plane_frame', 'map')
        self.declare_parameter('plane_width', 4.0)
        self.declare_parameter('plane_height', 3.0)
        self.declare_parameter('preserve_image_aspect', True)
        self.declare_parameter('position_x', 0.0)
        self.declare_parameter('position_y', 0.0)
        self.declare_parameter('position_z', 0.02)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('alpha', 1.0)
        self.declare_parameter('publish_every_n', 1)
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('use_png_texture', True)

        self.plane_frame = self.get_parameter('plane_frame').value
        self.plane_width = float(self.get_parameter('plane_width').value)
        self.plane_height = float(self.get_parameter('plane_height').value)
        self.preserve_image_aspect = bool(self.get_parameter('preserve_image_aspect').value)
        self.alpha = float(self.get_parameter('alpha').value)
        self.publish_every_n = max(1, int(self.get_parameter('publish_every_n').value))
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.use_png_texture = bool(self.get_parameter('use_png_texture').value)
        self.image_count = 0

        self.marker_pub = self.create_publisher(
            Marker,
            self.get_parameter('marker_topic').value,
            1,
        )
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            10,
        )

        self.get_logger().info(
            'Projecting image topic '
            f'{self.get_parameter("image_topic").value} onto plane marker '
            f'{self.get_parameter("marker_topic").value} in frame {self.plane_frame}'
        )

    def image_callback(self, msg):
        self.image_count += 1
        if self.image_count % self.publish_every_n != 0:
            return

        try:
            bgr = self.image_to_bgr(msg)
        except RuntimeError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        texture = self.encode_texture(bgr, msg.header)
        marker = self.make_marker(texture, msg.header.stamp, bgr.shape[1], bgr.shape[0])
        self.marker_pub.publish(marker)

    def image_to_bgr(self, msg):
        encoding = msg.encoding.lower()
        if msg.height == 0 or msg.width == 0:
            raise RuntimeError('Received empty image.')

        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding in ('rgb8', 'bgr8'):
            image = data.reshape((msg.height, msg.step // 3, 3))[:, :msg.width, :]
            if encoding == 'rgb8':
                return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image.copy()

        if encoding in ('rgba8', 'bgra8'):
            image = data.reshape((msg.height, msg.step // 4, 4))[:, :msg.width, :]
            if encoding == 'rgba8':
                return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        if encoding in ('mono8', '8uc1'):
            image = data.reshape((msg.height, msg.step))[:, :msg.width]
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        raise RuntimeError(
            f'Unsupported image encoding: {msg.encoding}. '
            'Supported encodings: rgb8, bgr8, rgba8, bgra8, mono8, 8UC1.'
        )

    def encode_texture(self, bgr, header):
        compressed = CompressedImage()
        compressed.header = header

        if self.use_png_texture:
            ok, encoded = cv2.imencode('.png', bgr)
            compressed.format = 'png'
        else:
            quality = min(100, max(1, self.jpeg_quality))
            ok, encoded = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
            compressed.format = 'jpeg'

        if not ok:
            raise RuntimeError('Failed to encode camera image as marker texture.')
        compressed.data = encoded.tobytes()
        return compressed

    def make_marker(self, texture, stamp, image_width, image_height):
        width, height = self.plane_size(image_width, image_height)
        half_w = width * 0.5
        half_h = height * 0.5

        top_left = Point(x=-half_w, y=half_h, z=0.0)
        bottom_left = Point(x=-half_w, y=-half_h, z=0.0)
        bottom_right = Point(x=half_w, y=-half_h, z=0.0)
        top_right = Point(x=half_w, y=half_h, z=0.0)

        uv_top_left = UVCoordinate(u=0.0, v=0.0)
        uv_bottom_left = UVCoordinate(u=0.0, v=1.0)
        uv_bottom_right = UVCoordinate(u=1.0, v=1.0)
        uv_top_right = UVCoordinate(u=1.0, v=0.0)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.plane_frame
        marker.ns = 'camera_projection'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.get_parameter('position_x').value)
        marker.pose.position.y = float(self.get_parameter('position_y').value)
        marker.pose.position.z = float(self.get_parameter('position_z').value)
        qx, qy, qz, qw = quaternion_from_euler(
            float(self.get_parameter('roll').value),
            float(self.get_parameter('pitch').value),
            float(self.get_parameter('yaw').value),
        )
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = min(1.0, max(0.0, self.alpha))
        marker.frame_locked = False
        marker.texture_resource = 'embedded://camera_projection'
        marker.texture = texture

        marker.points = [
            top_left,
            bottom_left,
            bottom_right,
            top_left,
            bottom_right,
            top_right,
        ]
        marker.uv_coordinates = [
            uv_top_left,
            uv_bottom_left,
            uv_bottom_right,
            uv_top_left,
            uv_bottom_right,
            uv_top_right,
        ]
        return marker

    def plane_size(self, image_width, image_height):
        width = max(0.001, self.plane_width)
        height = max(0.001, self.plane_height)
        if not self.preserve_image_aspect:
            return width, height

        image_aspect = image_width / max(1.0, float(image_height))
        plane_aspect = width / height
        if plane_aspect > image_aspect:
            width = height * image_aspect
        else:
            height = width / image_aspect
        return width, height


def main():
    rclpy.init()
    node = ImageToPlaneMarker()
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
