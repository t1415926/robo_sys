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
./start_astar_planning.sh --build --goal 1.0 0.8 0.0
```

## 设计说明

本包只负责启动编排，不放复杂业务逻辑。具体算法和节点实现放在：

```text
my_robot_navigation
my_robot_gazebo
point_lio
livox_ros_driver2
```
