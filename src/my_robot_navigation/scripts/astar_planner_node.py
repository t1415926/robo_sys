#!/usr/bin/env python3
import heapq
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('extra_inflation_radius', 0.05)
        self.declare_parameter('simplify_path', True)

        self.map_msg = None
        self.blocked = None
        self.last_initial_pose = None

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.extra_inflation_radius = float(self.get_parameter('extra_inflation_radius').value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.map_callback,
            map_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('goal_topic').value,
            self.goal_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            10,
        )
        self.plan_pub = self.create_publisher(Path, self.get_parameter('plan_topic').value, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'A* planner ready: map={self.get_parameter("map_topic").value}, '
            f'goal={self.get_parameter("goal_topic").value}, plan={self.get_parameter("plan_topic").value}'
        )

    def map_callback(self, msg):
        self.map_msg = msg
        self.blocked = self.build_blocked_grid(msg)
        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height}, '
            f'{msg.info.resolution:.3f} m/cell'
        )

    def initial_pose_callback(self, msg):
        self.last_initial_pose = msg.pose.pose

    def goal_callback(self, msg):
        if self.map_msg is None or self.blocked is None:
            self.get_logger().warn('No map received yet, cannot plan.')
            return

        start_pose = self.get_start_pose()
        if start_pose is None:
            self.get_logger().warn(
                f'Cannot get start pose from TF {self.map_frame}->{self.base_frame}. '
                'Use RViz 2D Pose Estimate or make sure TF is available.'
            )
            return

        start = self.world_to_cell(start_pose.position.x, start_pose.position.y)
        goal = self.world_to_cell(msg.pose.position.x, msg.pose.position.y)

        if start is None:
            self.get_logger().warn('Start pose is outside the map.')
            return
        if goal is None:
            self.get_logger().warn('Goal pose is outside the map.')
            return
        if self.is_blocked(*start):
            self.get_logger().warn(f'Start cell is occupied or inflated: {start}')
            return
        if self.is_blocked(*goal):
            self.get_logger().warn(f'Goal cell is occupied or inflated: {goal}')
            return

        cells = self.astar(start, goal)
        if not cells:
            self.get_logger().warn(f'A* failed: no path from {start} to {goal}.')
            return

        if bool(self.get_parameter('simplify_path').value):
            cells = self.simplify_cells(cells)

        path = self.cells_to_path(cells)
        self.plan_pub.publish(path)
        self.get_logger().info(
            f'Published A* path with {len(path.poses)} poses '
            f'from ({start[0]}, {start[1]}) to ({goal[0]}, {goal[1]}).'
        )

    def get_start_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
            pose = PoseStamped().pose
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except TransformException:
            return self.last_initial_pose

    def build_blocked_grid(self, msg):
        width = msg.info.width
        height = msg.info.height
        raw = list(msg.data)
        occupied = [[False for _ in range(width)] for _ in range(height)]

        for y in range(height):
            row_offset = y * width
            for x in range(width):
                value = raw[row_offset + x]
                if value < 0:
                    occupied[y][x] = self.unknown_is_obstacle
                else:
                    occupied[y][x] = value >= self.occupied_threshold

        inflation_m = self.robot_radius + self.extra_inflation_radius
        inflate_cells = int(math.ceil(inflation_m / max(msg.info.resolution, 1e-6)))
        if inflate_cells <= 0:
            return occupied

        inflated = [row[:] for row in occupied]
        offsets = []
        for dy in range(-inflate_cells, inflate_cells + 1):
            for dx in range(-inflate_cells, inflate_cells + 1):
                if dx * dx + dy * dy <= inflate_cells * inflate_cells:
                    offsets.append((dx, dy))

        for y in range(height):
            for x in range(width):
                if not occupied[y][x]:
                    continue
                for dx, dy in offsets:
                    nx = x + dx
                    ny = y + dy
                    if 0 <= nx < width and 0 <= ny < height:
                        inflated[ny][nx] = True
        return inflated

    def world_to_cell(self, wx, wy):
        info = self.map_msg.info
        mx = int(math.floor((wx - info.origin.position.x) / info.resolution))
        my = int(math.floor((wy - info.origin.position.y) / info.resolution))
        if 0 <= mx < info.width and 0 <= my < info.height:
            return mx, my
        return None

    def cell_to_world(self, mx, my):
        info = self.map_msg.info
        return (
            info.origin.position.x + (mx + 0.5) * info.resolution,
            info.origin.position.y + (my + 0.5) * info.resolution,
        )

    def is_blocked(self, mx, my):
        return self.blocked[my][mx]

    def astar(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (0.0, 0.0, start))
        came_from = {}
        best_cost = {start: 0.0}
        closed = set()

        while open_heap:
            _priority, cost, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal:
                return self.reconstruct_path(came_from, current)

            closed.add(current)
            for neighbor, step_cost in self.neighbors(current):
                if neighbor in closed:
                    continue
                new_cost = cost + step_cost
                if new_cost >= best_cost.get(neighbor, float('inf')):
                    continue
                best_cost[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + self.heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, new_cost, neighbor))
        return []

    def neighbors(self, cell):
        x, y = cell
        motions = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
        ]
        if self.allow_diagonal:
            diagonal = math.sqrt(2.0)
            motions.extend([
                (1, 1, diagonal),
                (1, -1, diagonal),
                (-1, 1, diagonal),
                (-1, -1, diagonal),
            ])

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        for dx, dy, cost in motions:
            nx = x + dx
            ny = y + dy
            if 0 <= nx < width and 0 <= ny < height and not self.is_blocked(nx, ny):
                if dx != 0 and dy != 0 and (self.is_blocked(x + dx, y) or self.is_blocked(x, y + dy)):
                    continue
                yield (nx, ny), cost

    @staticmethod
    def heuristic(cell, goal):
        return math.hypot(goal[0] - cell[0], goal[1] - cell[1])

    @staticmethod
    def reconstruct_path(came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    @staticmethod
    def simplify_cells(cells):
        if len(cells) <= 2:
            return cells
        simplified = [cells[0]]
        previous_direction = (
            cells[1][0] - cells[0][0],
            cells[1][1] - cells[0][1],
        )
        for index in range(1, len(cells) - 1):
            current_direction = (
                cells[index + 1][0] - cells[index][0],
                cells[index + 1][1] - cells[index][1],
            )
            if current_direction != previous_direction:
                simplified.append(cells[index])
            previous_direction = current_direction
        simplified.append(cells[-1])
        return simplified

    def cells_to_path(self, cells):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame

        for index, cell in enumerate(cells):
            x, y = self.cell_to_world(*cell)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            if index + 1 < len(cells):
                nx, ny = self.cell_to_world(*cells[index + 1])
                yaw = math.atan2(ny - y, nx - x)
            elif index > 0:
                px, py = self.cell_to_world(*cells[index - 1])
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = 0.0
            qz, qw = quaternion_from_yaw(yaw)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)
        return path


def main():
    rclpy.init()
    node = AStarPlannerNode()
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
