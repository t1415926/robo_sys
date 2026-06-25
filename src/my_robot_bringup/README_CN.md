# my_robot_bringup 中文说明

## 功能定位

`my_robot_bringup` 负责组合启动各个功能包，是项目的一键启动入口集合。

## 主要 launch

```text
launch/sim_bringup.launch.py           # 轻量 2D 全向底盘 + Nav2 导航
launch/astar_sim_bringup.launch.py     # A* 全局规划 + Nav2 controller 跟踪
launch/gazebo_omni_bringup.launch.py   # Gazebo 全向底盘仿真 + Nav2
launch/pointlio_nav_bag.launch.py      # PointLIO bag 建图/投影地图 + Nav2 规划
```

`pointlio_nav_bag.launch.py` 默认使用 `my_robot_navigation/config/nav2_pointcloud_params.yaml`，local costmap 会订阅 `/cloud_registered` 作为实时点云障碍层；`/Laser_map` 仍通过投影节点生成全局 `/map`。

## 常用脚本

项目根目录提供了一键脚本：

```text
start_sim_nav.sh             # 启动轻量 Nav2 仿真
start_astar_planning.sh      # 启动 A* 规划示例
start_gazebo_omni_nav.sh     # 启动 Gazebo 全向底盘仿真
start_pointlio_nav_bag.sh    # 启动 PointLIO bag 测试链路
```

## 使用示例

```bash
cd /home/dtc/robo_sys
./start_astar_planning.sh --build --goal 2.2 0.0 0.0
```

A* 示例默认使用窄单行道路形态地图 `one_way_road_map.yaml`。当前启动参数已经针对差速底盘、窄路过弯做了 A* 中线偏好和 DWB 低速跟踪调参；DWB 不采样横向速度，底盘节点会忽略 `cmd_vel.linear.y`。如需切换地图：

```bash
./start_astar_planning.sh --map src/my_robot_navigation/maps/simple_map.yaml
```

测试过弯目标：

```bash
./start_astar_planning.sh --goal 3.0 1.5 1.57
```

## 设计说明

本包只负责启动编排，不放复杂业务逻辑。具体算法和节点实现放在：

```text
my_robot_navigation
my_robot_gazebo
point_lio
livox_ros_driver2
```
