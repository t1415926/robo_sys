# 当前项目进展总结

## 1. 项目管理

- 已初始化 Git 仓库，并推送到 GitHub。
- 当前主要开发分支为 `cleanup-docs-cn`。
- 已补充 Git 协作说明：`docs/git_workflow.md`。

## 2. 基础仿真链路

- 已实现轻量 2D 仿真底盘，订阅 `/cmd_vel`，发布 `/odom` 和 TF。
- 已提供一键启动脚本：
  - `start_sim_nav.sh`
  - `start_astar_planning.sh`
  - `start_gazebo_omni_nav.sh`
  - `start_pointlio_nav_bag.sh`
- RViz 可显示地图、机器人模型、TF、规划路径和 costmap。

## 3. 地图生成与修整

- 已实现离线 PCD 点云转 2D 栅格地图工具：
  - `pcd_to_grid_map.py`
- 支持按高度切片、障碍投影、障碍膨胀，生成：
  - `map.pgm`
  - `map.yaml`
- 已实现语义 mask 标注工具：
  - `semantic_mask_editor.py`
- 支持在 2D 投影图上绘制：
  - 红色：禁止通行区域
  - 绿色：允许通行区域
- 已实现相机图像投影到平面并在 RViz 中可视化的初步节点：
  - `image_to_plane_marker.py`

## 4. PointLIO 接入

- 已接入 `src/pointlio` 功能包。
- 已支持 bag 测试链路：
  - `pointlio_nav_bag.launch.py`
- `/Laser_map` 用于生成全局 `/map`。
- `/cloud_registered_body` 已接入 Nav2 local costmap，作为实时局部障碍点云来源。
- 已打开 PointLIO 机体系当前帧点云发布：
  - `scan_bodyframe_pub_en: true`

## 5. 先验地图定位

- PointLIO 代码已具备先验 PCD 地图初始化能力。
- 当前导航配置文件为：
  - `src/my_robot_bringup/config/pointlio_mid360_nav.yaml`
- 需要开启：

```yaml
common:
  use_prior_map: true
  prior_map_path: "/path/to/prior_map.pcd"
```

- 初始化主要搜索：
  - x
  - y
  - yaw
- pitch/roll 主要依赖 IMU 重力对齐和固定外参，不作为主要搜索量。

## 6. 全局规划

- 已实现自写 A* 全局规划节点：
  - `astar_planner_node.py`
- 输入：
  - `/map`
  - `/goal_pose`
- 输出：
  - `/plan`
- A* 已加入：
  - 障碍硬膨胀
  - 离障碍距离软代价
  - 转弯代价
  - 窄路中线偏好
- 当前示例不依赖 Nav2 `planner_server` 做全局规划。

## 7. 轨迹跟踪与底盘输出

- 已实现 A* 路径到 Nav2 FollowPath action 的桥接：
  - `astar_follow_path_bridge.py`
- 当前轨迹跟踪复用 Nav2 `controller_server`。
- Nav2 controller 输出：
  - `/cmd_vel`
- 已按差速底盘约束调整：
  - 不采样横向速度
  - `linear.y` 不用于底盘运动
  - 主要使用 `linear.x` 和 `angular.z`

## 8. 局部避障

- 当前 local costmap 支持订阅实时点云：
  - `/cloud_registered_body`
- 当前已能做实时障碍标记。
- 后续可进一步开启射线清除：

```yaml
marking: true
clearing: true
raytrace_max_range: 4.5
```

- 全局静态栅格地图不建议实时频繁修改，动态障碍优先交给 local costmap。

## 9. 当前可运行目标

当前项目已经具备从地图到运动控制的基本闭环：

```text
先验 PCD 点云地图
  -> PointLIO 定位
  -> map 中机器人位姿
  -> 2D 栅格地图规划
  -> A* / Nav2 路径
  -> Nav2 controller 跟踪
  -> /cmd_vel
  -> 底盘执行
```

## 10. 后续重点

- 配置真实先验 PCD 地图路径。
- 确认 2D 栅格地图与 PCD 地图共用同一 `map` 坐标。
- 标定或确认 `base_footprint -> body` 静态 TF。
- 低速验证 `/cmd_vel` 到底盘方向。
- 根据真实窄路调节：
  - 地图膨胀
  - costmap inflation
  - A* 软代价
  - controller 速度和角速度限制
