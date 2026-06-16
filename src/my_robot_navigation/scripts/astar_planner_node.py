#!/usr/bin/env python3
"""A* 示例使用的独立全局规划节点。

这个节点刻意不调用 Nav2 的 planner_server。它读取 OccupancyGrid，接收
RViz 发布的 /goal_pose，在栅格地图上运行 A*，并把 nav_msgs/Path 发布
到 /plan。后续可以由桥接节点把 /plan 交给 Nav2 controller_server 的
FollowPath action 做轨迹跟踪。
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


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner_node')
        # 话题和坐标系参数保持可配置，方便同时用于仿真地图和后续 SLAM 生成地图。
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # OccupancyGrid 数值约定：
        #   -1: 未知
        #    0: 空闲
        #  100: 占据
        # 大于等于 occupied_threshold 的格子会被当作障碍。
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('allow_diagonal', True)

        # 在 A* 栅格内部提前膨胀障碍物，使全局路径在进入 Nav2 局部控制前
        # 就和墙体、障碍保持基本安全距离。
        self.declare_parameter('robot_radius', 0.22)
        self.declare_parameter('extra_inflation_radius', 0.05)
        self.declare_parameter('simplify_path', True)
        self.declare_parameter('obstacle_cost_weight', 0.8)
        self.declare_parameter('preferred_clearance', 0.35)
        self.declare_parameter('turn_cost_weight', 0.20)

        # 掉头区简化模型：第一次有效机器人位姿会成为唯一掉头点。窄路区域
        # 仍然可以正常行驶，但不要求机器人在终点原地调整目标朝向。
        self.declare_parameter('turnaround_constraint_enabled', True)
        self.declare_parameter('turnaround_zone_tolerance', 0.35)
        self.declare_parameter('turnaround_zone_marker_topic', '/turnaround_zones')

        self.map_msg = None
        self.blocked = None
        self.obstacle_distance = None
        self.last_initial_pose = None
        self.turnaround_zone_pose = None

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.unknown_is_obstacle = bool(self.get_parameter('unknown_is_obstacle').value)
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.extra_inflation_radius = float(self.get_parameter('extra_inflation_radius').value)
        self.obstacle_cost_weight = float(self.get_parameter('obstacle_cost_weight').value)
        self.preferred_clearance = float(self.get_parameter('preferred_clearance').value)
        self.turn_cost_weight = float(self.get_parameter('turn_cost_weight').value)
        self.turnaround_constraint_enabled = bool(
            self.get_parameter('turnaround_constraint_enabled').value
        )
        self.turnaround_zone_tolerance = float(self.get_parameter('turnaround_zone_tolerance').value)

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
        self.turnaround_marker_pub = self.create_publisher(
            Marker,
            self.get_parameter('turnaround_zone_marker_topic').value,
            1,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'A* planner ready: map={self.get_parameter("map_topic").value}, '
            f'goal={self.get_parameter("goal_topic").value}, plan={self.get_parameter("plan_topic").value}'
        )

    def map_callback(self, msg):
        # 每次 /map 更新都重建膨胀后的障碍栅格。这样既支持静态地图，也支持
        # 点云实时投影生成的动态地图。
        self.map_msg = msg
        occupied = self.build_occupied_grid(msg)
        self.blocked = self.inflate_obstacles(occupied, msg)
        self.obstacle_distance = self.build_obstacle_distance_grid(occupied, msg)
        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height}, '
            f'{msg.info.resolution:.3f} m/cell'
        )

    def initial_pose_callback(self, msg):
        # TF 暂时不可用时的备用起点。在正常仿真链路中，起点来自
        # map->odom 和 odom->base_footprint 组成的 TF。
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

        if self.turnaround_zone_pose is None:
            # 根据需求：初始点就是掉头区域。这里在第一次有效规划请求时再
            # 记录它，保证这个位姿和 A* 后续使用的起点来源一致。
            self.turnaround_zone_pose = start_pose
            self.publish_turnaround_zone_marker()
            self.get_logger().info(
                'Set initial pose as turnaround zone: '
                f'x={start_pose.position.x:.2f}, y={start_pose.position.y:.2f}'
            )

        self.plan_to_goal(start_pose, msg)

    def plan_to_goal(self, start_pose, goal_msg):
        # A* 在整数栅格坐标上搜索。开始搜索前，先把 map 坐标系下的米制坐标
        # 转成 OccupancyGrid 的栅格索引。
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

        cells = self.astar(start, goal)
        if not cells:
            self.get_logger().warn(f'A* failed: no path from {start} to {goal}.')
            return

        if bool(self.get_parameter('simplify_path').value):
            cells = self.simplify_cells(cells)

        # 如果最终目标不在掉头区域内，就不要要求控制器在终点完成目标朝向。
        # 此时最后一个路径点继承道路方向，适合窄单行道路上的“只前进不掉头”。
        final_yaw = goal_yaw
        if self.turnaround_constraint_enabled and not self.is_near_turnaround_zone(goal_msg.pose):
            final_yaw = None
        path = self.cells_to_path(cells, final_yaw=final_yaw)
        self.plan_pub.publish(path)
        self.get_logger().info(
            f'Published A* path with {len(path.poses)} poses '
            f'from ({start[0]}, {start[1]}) to ({goal[0]}, {goal[1]}).'
        )

    def is_near_turnaround_zone(self, pose):
        # 当前示例只有一个圆形掉头区。后续接入 mask 或多边形区域时，
        # 可以优先替换这个函数，而不用改 A* 搜索主体。
        if self.turnaround_zone_pose is None:
            return False
        dx = pose.position.x - self.turnaround_zone_pose.position.x
        dy = pose.position.y - self.turnaround_zone_pose.position.y
        return math.hypot(dx, dy) <= self.turnaround_zone_tolerance

    def get_start_pose(self):
        # 优先使用 TF，因为它反映当前真实/仿真的机器人位姿。如果 TF 不可用，
        # 则退回到 RViz 2D Pose Estimate 最近发布的初始位姿，方便局部 bringup 测试。
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

    def build_occupied_grid(self, msg):
        # 把 OccupancyGrid 的一维行优先数据转成二维布尔栅格。True 表示原始
        # 地图中该格子是障碍或未知区域。
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
        return occupied

    def inflate_obstacles(self, occupied, msg):
        # 按机器人半径膨胀障碍物，生成 A* 的硬约束栅格。True 表示不能通行。
        width = msg.info.width
        height = msg.info.height
        inflation_m = self.robot_radius + self.extra_inflation_radius
        inflate_cells = int(math.ceil(inflation_m / max(msg.info.resolution, 1e-6)))
        if inflate_cells <= 0:
            return occupied

        inflated = [row[:] for row in occupied]

        # 预先计算圆形膨胀范围内的栅格偏移。当前写法直观清楚；如果地图变大，
        # 后续可以用距离变换等方法优化。
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

    def build_obstacle_distance_grid(self, occupied, msg):
        # 多源 Dijkstra 距离场：每个空闲格子记录到最近原始障碍的距离。A*
        # 用它作为软代价，让路径主动靠近窄路中心，而不是贴着墙走。
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        distances = [[float('inf') for _ in range(width)] for _ in range(height)]
        queue = []

        for y in range(height):
            for x in range(width):
                if occupied[y][x]:
                    distances[y][x] = 0.0
                    heapq.heappush(queue, (0.0, x, y))

        motions = [
            (1, 0, resolution),
            (-1, 0, resolution),
            (0, 1, resolution),
            (0, -1, resolution),
            (1, 1, resolution * math.sqrt(2.0)),
            (1, -1, resolution * math.sqrt(2.0)),
            (-1, 1, resolution * math.sqrt(2.0)),
            (-1, -1, resolution * math.sqrt(2.0)),
        ]

        while queue:
            distance, x, y = heapq.heappop(queue)
            if distance > distances[y][x]:
                continue
            for dx, dy, step in motions:
                nx = x + dx
                ny = y + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                new_distance = distance + step
                if new_distance < distances[ny][nx]:
                    distances[ny][nx] = new_distance
                    heapq.heappush(queue, (new_distance, nx, ny))
        return distances

    def world_to_cell(self, wx, wy):
        # OccupancyGrid 的 origin 是地图左下角位姿。这里用 floor() 找到世界坐标
        # 所在的格子，后续发布路径时再使用格子中心点。
        info = self.map_msg.info
        mx = int(math.floor((wx - info.origin.position.x) / info.resolution))
        my = int(math.floor((wy - info.origin.position.y) / info.resolution))
        if 0 <= mx < info.width and 0 <= my < info.height:
            return mx, my
        return None

    def cell_to_world(self, mx, my):
        # 把栅格索引转回该格子中心点的世界坐标，单位是米。
        info = self.map_msg.info
        return (
            info.origin.position.x + (mx + 0.5) * info.resolution,
            info.origin.position.y + (my + 0.5) * info.resolution,
        )

    def is_blocked(self, mx, my):
        return self.blocked[my][mx]

    def astar(self, start, goal):
        # 标准 A*：open_heap 中保存 (f_score, g_score, cell)。best_cost 用来
        # 避免用更差的代价重复访问同一个格子。
        open_heap = []
        start_state = (start, None)
        heapq.heappush(open_heap, (0.0, 0.0, start, None))
        came_from = {}
        best_cost = {start_state: 0.0}
        closed = set()
        best_goal_state = None

        while open_heap:
            _priority, cost, current, previous_direction = heapq.heappop(open_heap)
            state = (current, previous_direction)
            if state in closed:
                continue
            if current == goal:
                best_goal_state = state
                break

            closed.add(state)
            for neighbor, step_cost, direction in self.neighbors(current):
                neighbor_state = (neighbor, direction)
                if neighbor_state in closed:
                    continue
                new_cost = (
                    cost
                    + step_cost
                    + self.clearance_cost(neighbor, step_cost)
                    + self.turn_cost(previous_direction, direction)
                )
                if new_cost >= best_cost.get(neighbor_state, float('inf')):
                    continue
                best_cost[neighbor_state] = new_cost
                came_from[neighbor_state] = state
                priority = new_cost + self.heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, new_cost, neighbor, direction))
        if best_goal_state is None:
            return []
        return self.reconstruct_path(came_from, best_goal_state)

    def neighbors(self, cell):
        # 生成 4 邻接或 8 邻接候选格子。对角移动时禁止“切角”，避免路径从两个
        # 膨胀障碍格子之间挤过去。
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
                yield (nx, ny), cost, (dx, dy)

    def clearance_cost(self, cell, step_cost):
        # 离障碍越近代价越高。它不是硬限制，所以窄路仍然可以通过；但只要
        # 有空间，路径会自然回到道路中线。
        if self.obstacle_distance is None or self.obstacle_cost_weight <= 0.0:
            return 0.0
        distance = self.obstacle_distance[cell[1]][cell[0]]
        if not math.isfinite(distance) or distance >= self.preferred_clearance:
            return 0.0
        clearance_ratio = (self.preferred_clearance - distance) / max(self.preferred_clearance, 1e-6)
        return self.obstacle_cost_weight * clearance_ratio * clearance_ratio * step_cost

    def turn_cost(self, previous_direction, direction):
        # 对急转弯加软代价，减少 A* 在格子边缘抖动，拐角处更倾向选择平顺路线。
        if previous_direction is None or self.turn_cost_weight <= 0.0:
            return 0.0
        if previous_direction == direction:
            return 0.0
        prev_angle = math.atan2(previous_direction[1], previous_direction[0])
        next_angle = math.atan2(direction[1], direction[0])
        angle_delta = abs(math.atan2(math.sin(next_angle - prev_angle), math.cos(next_angle - prev_angle)))
        return self.turn_cost_weight * (angle_delta / math.pi)

    @staticmethod
    def heuristic(cell, goal):
        return math.hypot(goal[0] - cell[0], goal[1] - cell[1])

    @staticmethod
    def reconstruct_path(came_from, current_state):
        path = [current_state[0]]
        while current_state in came_from:
            current_state = came_from[current_state]
            path.append(current_state[0])
        path.reverse()
        return path

    @staticmethod
    def simplify_cells(cells):
        # 把连续直线段压缩成拐点序列。这样 RViz 和 FollowPath 看到的路径更简洁，
        # 同时保持原来的折线路径形状。
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
        # 转成标准 nav_msgs/Path。RViz 可以直接显示，桥接节点也可以把它发给
        # Nav2 controller_server 的 FollowPath action。
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

    def publish_turnaround_zone_marker(self):
        # RViz 可视化提示：用一个蓝色半透明圆盘标出当前允许掉头的区域。
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = 'turnaround_zone'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.turnaround_zone_pose.position.x
        marker.pose.position.y = self.turnaround_zone_pose.position.y
        marker.pose.position.z = 0.03
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.turnaround_zone_tolerance * 2.0
        marker.scale.y = self.turnaround_zone_tolerance * 2.0
        marker.scale.z = 0.04
        marker.color.r = 0.0
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.45
        self.turnaround_marker_pub.publish(marker)


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
