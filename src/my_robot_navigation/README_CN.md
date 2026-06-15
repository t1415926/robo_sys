# my_robot_navigation 中文说明

## 功能定位

`my_robot_navigation` 放置导航相关配置、地图、RViz 配置和 Python 节点。当前同时支持：

```text
Nav2 静态地图导航
A* 全局规划示例
PointLIO 点云转 2D 栅格地图
语义 mask 标注工具
相机图像平面投影可视化
```

## 主要目录

```text
config/      # Nav2 参数
launch/      # 导航相关 launch
maps/        # 示例 2D 栅格地图
rviz/        # RViz 配置
scripts/     # 自定义导航、规划、工具节点
```

## 主要脚本

```text
simple_omni_base_node.py          # 简单全向底盘仿真，订阅 /cmd_vel，发布 /odom 和 TF
astar_planner_node.py             # 自写 A* 全局规划，订阅 /map 和 /goal_pose，发布 /plan
astar_follow_path_bridge.py       # 把 A* 的 /plan 发送给 Nav2 controller_server 的 FollowPath action
send_pose_goal_node.py            # A* 示例用的 /goal_pose 自动发布节点
send_goal_node.py                 # Nav2 NavigateToPose 自动目标点发送节点
pointcloud_to_occupancy_grid.py   # /Laser_map 点云投影成 /map 栅格
semantic_mask_editor.py           # 红/绿语义 mask 可视化标注工具
image_to_plane_marker.py          # 将相机图像作为纹理投影到 RViz 平面 Marker
```

## 常用入口

启动 Nav2 仿真导航：

```bash
./start_sim_nav.sh --build
```

启动 A* 全局规划 + Nav2 控制器跟踪：

```bash
./start_astar_planning.sh --build
```

只显示 A* 路径，不跟踪：

```bash
./start_astar_planning.sh --build --no-tracking
```

## 注意事项

- A* 示例不调用 Nav2 `planner_server`。
- A* 路径通过 `astar_follow_path_bridge.py` 交给 Nav2 `controller_server` 跟踪。
- 当前旋转区约束只把第一次规划时的机器人起点作为可旋转区域。
- 后续可以扩展为读取语义 mask 或多边形区域来定义多个可旋转区域。
