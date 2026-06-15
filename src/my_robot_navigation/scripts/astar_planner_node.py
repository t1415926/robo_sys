#!/usr/bin/env python3
"""Standalone A* global planner used by the A* demo launch.

This node deliberately does not call Nav2's planner_server. It reads an
OccupancyGrid, accepts RViz /goal_pose goals, runs grid-based A*, and publishes
nav_msgs/Path on /plan. A separate bridge node may then hand that /plan to Nav2
controller_server for FollowPath tracking.
"""

import heapq
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


def quaternion_from_yaw(yaw):
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        # Topic and frame parameters keep the node reusable for both simulation
        # maps and future SLAM-generated maps.
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # OccupancyGrid values are interpreted as:
        #   -1: unknown
        #    0: free
        #  100: occupied
        # Values >= occupied_threshold are treated as obstacles.
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('allow_diagonal', True)

        # We inflate obstacles in the A* grid itself so the global path keeps a
        # basic clearance from walls even before Nav2's local costmap sees it.
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('extra_inflation_radius', 0.05)
        self.declare_parameter('simplify_path', True)

        # Initial implementation of "rotation is only allowed in specific
        # areas": the first valid robot pose becomes the only rotation zone.
        # Later this can be replaced with a semantic mask or polygon list.
        self.declare_parameter('rotation_constraint_enabled', True)
        self.declare_parameter('rotation_yaw_threshold', 0.35)
        self.declare_parameter('rotation_zone_tolerance', 0.20)
        self.declare_parameter('rotation_zone_marker_topic', '/rotation_zones')

        self.map_msg = None
        self.blocked = None
        self.last_initial_pose = None
        self.rotation_zone_pose = None

        # When a goal requires rotation while the robot is outside the rotation
        # zone, we first publish a path back to the zone and store the real goal
        # here. check_pending_goal() publishes the final path after arrival.
        self.pending_goal = None

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.extra_inflation_radius = float(self.get_parameter('extra_inflation_radius').value)
        self.rotation_constraint_enabled = bool(
            self.get_parameter('rotation_constraint_enabled').value
        )
        self.rotation_yaw_threshold = float(self.get_parameter('rotation_yaw_threshold').value)
        self.rotation_zone_tolerance = float(self.get_parameter('rotation_zone_tolerance').value)

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
        self.rotation_marker_pub = self.create_publisher(
            Marker,
            self.get_parameter('rotation_zone_marker_topic').value,
            1,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.2, self.check_pending_goal)

        self.get_logger().info(
            f'A* planner ready: map={self.get_parameter("map_topic").value}, '
            f'goal={self.get_parameter("goal_topic").value}, plan={self.get_parameter("plan_topic").value}'
        )

    def map_callback(self, msg):
        # Rebuild the inflated occupancy grid whenever /map changes. This keeps
        # the A* planner compatible with both static and dynamically projected
        # maps.
        self.map_msg = msg
        self.blocked = self.build_blocked_grid(msg)
        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height}, '
            f'{msg.info.resolution:.3f} m/cell'
        )

    def initial_pose_callback(self, msg):
        # Fallback start pose for cases where TF is not available yet. In the
        # normal sim path, map->odom and odom->base_footprint provide TF.
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

        if self.rotation_zone_pose is None:
            # The user asked to use the initial point as a rotation-capable
            # area. We define that lazily on the first valid planning request so
            # the pose comes from the same source A* will use for planning.
            self.rotation_zone_pose = start_pose
            self.publish_rotation_zone_marker()
            self.get_logger().info(
                'Set initial pose as rotation zone: '
                f'x={start_pose.position.x:.2f}, y={start_pose.position.y:.2f}'
            )

        self.plan_to_goal(start_pose, msg)

    def plan_to_goal(self, start_pose, goal_msg, ignore_rotation_constraint=False):
        # A* operates on integer grid cells. Convert both start and goal from
        # map-frame meters into OccupancyGrid indices before searching.
        start = self.world_to_cell(start_pose.position.x, start_pose.position.y)
        goal = self.world_to_cell(goal_msg.pose.position.x, goal_msg.pose.position.y)

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

        goal_yaw = yaw_from_quaternion(goal_msg.pose.orientation)
        start_yaw = yaw_from_quaternion(start_pose.orientation)

        # Rotation-area rule:
        # If the target requires a meaningful yaw change and the robot is not
        # already in a rotation zone, first route to the initial rotation zone.
        # Once the robot arrives there, check_pending_goal() sends the real goal.
        if (
            self.rotation_constraint_enabled
            and not ignore_rotation_constraint
            and self.rotation_zone_pose is not None
            and abs(normalize_angle(goal_yaw - start_yaw)) > self.rotation_yaw_threshold
            and not self.is_near_rotation_zone(start_pose)
        ):
            if self.plan_to_rotation_zone_then_wait(start_pose, goal_msg):
                return

        cells = self.astar(start, goal)
        if not cells:
            self.get_logger().warn(f'A* failed: no path from {start} to {goal}.')
            return

        if bool(self.get_parameter('simplify_path').value):
            cells = self.simplify_cells(cells)

        # Outside rotation zones, do not ask the controller to achieve the final
        # target yaw. The last pose will inherit the path heading instead. This
        # keeps rotate-to-goal behavior out of restricted areas.
        final_yaw = goal_yaw
        if self.rotation_constraint_enabled and not self.is_near_rotation_zone(goal_msg.pose):
            final_yaw = None
        path = self.cells_to_path(cells, final_yaw=final_yaw)
        self.plan_pub.publish(path)
        self.get_logger().info(
            f'Published A* path with {len(path.poses)} poses '
            f'from ({start[0]}, {start[1]}) to ({goal[0]}, {goal[1]}).'
        )

    def plan_to_rotation_zone_then_wait(self, start_pose, goal_msg):
        # Publish only the first leg: current pose -> rotation zone. The final
        # goal is intentionally delayed so Nav2 controller finishes this leg
        # before receiving the next FollowPath request.
        rotation_cell = self.world_to_cell(
            self.rotation_zone_pose.position.x,
            self.rotation_zone_pose.position.y,
        )
        start_cell = self.world_to_cell(start_pose.position.x, start_pose.position.y)
        if rotation_cell is None or start_cell is None:
            self.get_logger().warn('Rotation zone or start pose is outside the map.')
            return False
        if self.is_blocked(*rotation_cell):
            self.get_logger().warn('Rotation zone is occupied or inflated; falling back to direct plan.')
            return False

        cells = self.astar(start_cell, rotation_cell)
        if not cells:
            self.get_logger().warn('Cannot plan to rotation zone; falling back to direct plan.')
            return False
        if bool(self.get_parameter('simplify_path').value):
            cells = self.simplify_cells(cells)

        self.pending_goal = goal_msg
        path = self.cells_to_path(cells, final_yaw=self.heading_from_rotation_zone_to_goal(goal_msg))
        self.plan_pub.publish(path)
        self.get_logger().info(
            'Goal requires rotation. First published path to initial rotation zone; '
            'final goal will be planned after reaching the zone.'
        )
        return True

    def check_pending_goal(self):
        # Poll robot pose while a delayed goal exists. When the robot reaches the
        # rotation zone, publish the second leg and ignore the rotation check to
        # avoid looping back into the same staging behavior.
        if self.pending_goal is None:
            return
        start_pose = self.get_start_pose()
        if start_pose is None:
            return
        if not self.is_near_rotation_zone(start_pose):
            return

        goal = self.pending_goal
        self.pending_goal = None
        self.get_logger().info('Reached rotation zone; publishing final path to goal.')
        self.plan_to_goal(start_pose, goal, ignore_rotation_constraint=True)

    def is_near_rotation_zone(self, pose):
        # The current demo has one circular rotation zone. A future mask-based
        # implementation can replace this method without touching A* itself.
        if self.rotation_zone_pose is None:
            return False
        dx = pose.position.x - self.rotation_zone_pose.position.x
        dy = pose.position.y - self.rotation_zone_pose.position.y
        return math.hypot(dx, dy) <= self.rotation_zone_tolerance

    def heading_from_rotation_zone_to_goal(self, goal_msg):
        # When driving back to the rotation zone, face roughly toward the final
        # goal. This lets the controller rotate in the allowed area before the
        # second path segment is sent.
        dx = goal_msg.pose.position.x - self.rotation_zone_pose.position.x
        dy = goal_msg.pose.position.y - self.rotation_zone_pose.position.y
        if math.hypot(dx, dy) < 1e-6:
            return yaw_from_quaternion(goal_msg.pose.orientation)
        return math.atan2(dy, dx)

    def get_start_pose(self):
        # Prefer TF because it reflects the live robot pose. Fall back to the
        # last RViz initial pose so planning can still be tested in partial
        # bringups.
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
        # Convert the OccupancyGrid's flat row-major data into a 2D boolean map
        # and inflate occupied cells by robot radius. True means "A* may not use
        # this cell".
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

        # Precompute circular inflation offsets in cells. This is simple and
        # explicit; for larger maps we can optimize with distance transforms.
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
        # OccupancyGrid origin is the lower-left map pose. We plan in cell
        # centers but use floor() here to find the containing cell.
        info = self.map_msg.info
        mx = int(math.floor((wx - info.origin.position.x) / info.resolution))
        my = int(math.floor((wy - info.origin.position.y) / info.resolution))
        if 0 <= mx < info.width and 0 <= my < info.height:
            return mx, my
        return None

    def cell_to_world(self, mx, my):
        # Convert a grid index back to the center point of that cell in meters.
        info = self.map_msg.info
        return (
            info.origin.position.x + (mx + 0.5) * info.resolution,
            info.origin.position.y + (my + 0.5) * info.resolution,
        )

    def is_blocked(self, mx, my):
        return self.blocked[my][mx]

    def astar(self, start, goal):
        # Standard A*: open_heap stores (f_score, g_score, cell). best_cost
        # prevents revisiting cells through more expensive routes.
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
        # Generate 4- or 8-connected neighbors. Diagonal corner-cutting is
        # blocked so the path cannot slip between two inflated obstacle cells.
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
        # Compress straight runs into just their turning points. This keeps RViz
        # and FollowPath goals readable while preserving the same polyline.
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

    def cells_to_path(self, cells, final_yaw=None):
        # Publish a normal nav_msgs/Path so RViz can display it and the bridge
        # can send it to Nav2 controller_server's FollowPath action.
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

            if index == len(cells) - 1 and final_yaw is not None:
                yaw = final_yaw
            elif index + 1 < len(cells):
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

    def publish_rotation_zone_marker(self):
        # Visual hint in RViz: a blue translucent disk marks where rotation is
        # allowed in this first implementation.
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = 'rotation_zone'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.rotation_zone_pose.position.x
        marker.pose.position.y = self.rotation_zone_pose.position.y
        marker.pose.position.z = 0.03
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.rotation_zone_tolerance * 2.0
        marker.scale.y = self.rotation_zone_tolerance * 2.0
        marker.scale.z = 0.04
        marker.color.r = 0.0
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.45
        self.rotation_marker_pub.publish(marker)


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
