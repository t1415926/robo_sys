#!/usr/bin/env python3
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudToOccupancyGrid(Node):
    def __init__(self):
        super().__init__('pointcloud_to_occupancy_grid')
        self.declare_parameter('cloud_topic', '/Laser_map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('width_m', 40.0)
        self.declare_parameter('height_m', 40.0)
        self.declare_parameter('origin_x', -20.0)
        self.declare_parameter('origin_y', -20.0)
        self.declare_parameter('min_z', 0.05)
        self.declare_parameter('max_z', 1.50)
        self.declare_parameter('occupied_value', 100)
        self.declare_parameter('free_value', 0)
        self.declare_parameter('publish_empty_free_space', True)
        self.declare_parameter('inflate_radius_m', 0.20)
        self.declare_parameter('max_points_per_update', 250000)

        self.resolution = float(self.get_parameter('resolution').value)
        self.width_m = float(self.get_parameter('width_m').value)
        self.height_m = float(self.get_parameter('height_m').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.occupied_value = int(self.get_parameter('occupied_value').value)
        self.free_value = int(self.get_parameter('free_value').value)
        self.publish_empty_free_space = bool(self.get_parameter('publish_empty_free_space').value)
        self.inflate_radius_m = float(self.get_parameter('inflate_radius_m').value)
        self.max_points_per_update = int(self.get_parameter('max_points_per_update').value)

        self.width_cells = max(1, int(math.ceil(self.width_m / self.resolution)))
        self.height_cells = max(1, int(math.ceil(self.height_m / self.resolution)))
        self.inflate_cells = max(0, int(math.ceil(self.inflate_radius_m / self.resolution)))

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            qos,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('cloud_topic').value,
            self.cloud_callback,
            10,
        )
        self.get_logger().info(
            f'Projecting {self.get_parameter("cloud_topic").value} to '
            f'{self.get_parameter("map_topic").value} '
            f'({self.width_cells}x{self.height_cells}, {self.resolution:.3f} m/cell)'
        )

    def cloud_callback(self, msg):
        default_value = self.free_value if self.publish_empty_free_space else -1
        data = [default_value] * (self.width_cells * self.height_cells)
        occupied_cells = set()
        processed = 0

        for point in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if z < self.min_z or z > self.max_z:
                continue
            mx = int((x - self.origin_x) / self.resolution)
            my = int((y - self.origin_y) / self.resolution)
            if 0 <= mx < self.width_cells and 0 <= my < self.height_cells:
                occupied_cells.add((mx, my))
            processed += 1
            if 0 < self.max_points_per_update <= processed:
                break

        for mx, my in occupied_cells:
            self.mark_occupied(data, mx, my)

        grid = OccupancyGrid()
        grid.header.stamp = msg.header.stamp
        grid.header.frame_id = self.get_parameter('frame_id').value
        grid.info.map_load_time = msg.header.stamp
        grid.info.resolution = self.resolution
        grid.info.width = self.width_cells
        grid.info.height = self.height_cells
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        self.map_pub.publish(grid)

    def mark_occupied(self, data, mx, my):
        for dy in range(-self.inflate_cells, self.inflate_cells + 1):
            for dx in range(-self.inflate_cells, self.inflate_cells + 1):
                if dx * dx + dy * dy > self.inflate_cells * self.inflate_cells:
                    continue
                x = mx + dx
                y = my + dy
                if 0 <= x < self.width_cells and 0 <= y < self.height_cells:
                    data[y * self.width_cells + x] = self.occupied_value


def main():
    rclpy.init()
    node = PointCloudToOccupancyGrid()
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
