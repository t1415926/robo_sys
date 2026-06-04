# ROS2 Humble Nav2 初版仿真导航项目

本项目是 `ros2_humble_nav2_implementation_order.md` 的初步实现版本。当前版本不做 SLAM，不启动 AMCL，而是使用：

```text
简单 2D 全向底盘模拟 simple_omni_base_node
静态简单地图 map_server
静态 map -> odom TF
Nav2 全局规划和局部避障
```

当前版本不使用雷达模拟，不订阅 `/scan`。路径规划和避障依据已保存的 2D 栅格地图：

```text
src/my_robot_navigation/maps/simple_map.yaml
src/my_robot_navigation/maps/simple_map.pgm
```

## 当前实现内容

```text
src/
├── my_robot_description   # 机器人 URDF / Xacro、RViz 模型显示
├── my_robot_gazebo        # Gazebo world、Gazebo 全向底盘模拟节点
├── my_robot_navigation    # Nav2 参数、简单地图、全向底盘模拟节点
└── my_robot_bringup       # 一键启动仿真导航
```

协作提交和推送说明见：

```text
docs/git_workflow.md
```

后续模块替代方向说明见：

```text
docs/nav2_replacement_roadmap.md
```

初版公共接口：

| 话题 / TF | 说明 |
|---|---|
| `/cmd_vel` | Nav2 输出速度指令 |
| `/odom` | 全向底盘模拟节点发布的里程计 |
| `/map` | 静态模拟地图 |
| `/plan` | Nav2 全局规划路径，RViz 中显示为 Global Path |
| `map -> odom` | 静态 TF |
| `odom -> base_footprint` | simple_omni_base_node 发布 |

## 运行前依赖

如果当前终端进入了 conda 环境，先退出 conda，避免 ROS2 Python ABI 不匹配：

```bash
conda deactivate
```

如果 `which python3` 仍然指向 Anaconda，可以用后面的“系统 Python 构建命令”。

安装运行依赖：

```bash
sudo apt update
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-tf2-ros \
  ros-humble-teleop-twist-keyboard
```

如果需要启动 Gazebo 全向底盘仿真，还需要安装：

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs
```

## 编译

在项目根目录执行：

```bash
cd /home/dtc/robo_sys
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果系统里 Anaconda 抢占了 Python，使用这个更稳的构建命令：

```bash
cd /home/dtc/robo_sys
env -u PYTHONPATH -u PYTHONHOME \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash -c 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'
source install/setup.bash
```

## 启动轻量 2D 仿真导航

推荐使用一键启动脚本：

```bash
cd /home/dtc/robo_sys
./start_sim_nav.sh
```

不启动 RViz2：

```bash
./start_sim_nav.sh --no-rviz
```

强制重新编译后启动：

```bash
./start_sim_nav.sh --build
```

启动后自动发送目标点：

```bash
./start_sim_nav.sh --goal 2.2 1.8 0.0
```

参数含义：

```text
2.2 = 目标点 x，单位 m，map 坐标系
1.8 = 目标点 y，单位 m，map 坐标系
0.0 = 目标朝向 yaw，单位 rad，可省略
```

也可以手动启动：

```bash
ros2 launch my_robot_bringup sim_bringup.launch.py
```

启动后会打开：

```text
1. simple_omni_base_node 简单全向底盘模拟
2. robot_state_publisher 发布机器人模型
3. 静态 map -> odom TF
4. /map 简单 2D 栅格地图
5. Nav2 路径规划和控制
6. RViz2
```

## 启动 Gazebo 全向底盘导航

Gazebo 版本使用同一套 Nav2、同一张 2D 地图和同一个 RViz 配置，但底盘由 Gazebo 中的全向模型显示和运动。

```bash
cd /home/dtc/robo_sys
./start_gazebo_omni_nav.sh --build
```

不启动 RViz2：

```bash
./start_gazebo_omni_nav.sh --no-rviz
```

启动后自动发送目标点：

```bash
./start_gazebo_omni_nav.sh --goal 1.0 -1.0 0.0
```

Gazebo 全向版链路：

```text
Nav2 /cmd_vel
        ↓
gazebo_omni_base_node
        ↓
Gazebo simple_omni_robot 模型位姿
        ↓
/odom + odom -> base_footprint
```

这个版本不使用 `diff_drive_controller`，也不使用 `ros2_control`。它直接支持：

```text
linear.x
linear.y
angular.z
```

## 在 RViz2 中使用

RViz2 的 Fixed Frame 已设置为 `map`。

由于当前使用静态 `map -> odom` 加全向底盘里程计，不需要点击 `2D Pose Estimate`。直接点击 `Nav2 Goal`，在地图空白区域设置目标点即可。

Nav2 会根据 `simple_map.pgm` 中的障碍物占据栅格自动规划路径，并控制机器人到达目标点。

使用 `Nav2 Goal` 时建议在白色可通行区域点击并拖出朝向箭头；不要点到黑色障碍物、地图边界或地图外。

当前 RViz 默认只显示：

```text
1. 2D 栅格地图 /map
2. 机器人模型
3. TF
4. 全局路径 /plan
```

`Global Costmap` 和 `Local Costmap` 显示层默认不打开。当前版本主要依据保存好的 2D 地图规划，先不把 costmap 图层作为观察重点。

推荐先设置短距离目标，例如：

```text
x: 1.0
y: -1.0
```

再测试绕障目标，例如：

```text
x: 2.2
y: 1.8
```

如果 RViz 点目标后没有运动，先用脚本自动目标确认导航链路：

```bash
./start_sim_nav.sh --build --goal 1.0 -1.0 0.0
```

如果自动目标可以运动，说明 Nav2 和底盘控制正常，问题多半是 RViz 中目标点没有发出、目标点落在障碍物/地图外，或没有拖出目标朝向。

## 单独测试底盘控制

启动仿真后，另开终端：

```bash
cd /home/dtc/robo_sys
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘控制会发布 `/cmd_vel`，simple_omni_base_node 会直接积分 x、y 和 yaw 速度并更新 `/odom`。

## 常用检查命令

```bash
ros2 topic list
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /map
ros2 run tf2_tools view_frames
```

重点确认：

```text
1. /odom 有数据
2. /map 有数据
3. TF 中存在 map -> odom -> base_footprint -> base_link
4. Nav2 lifecycle 节点处于 active
5. RViz2 中能看到 2D 栅格地图和规划路径
```

如果 RViz 显示 `No map received`，优先检查：

```bash
ros2 topic echo --once --qos-durability transient_local /map
ros2 lifecycle get /map_server
ros2 node list | grep -E 'map_server|planner_server|controller_server|bt_navigator'
```

正常情况下：

```text
/map 应该能 echo 到一帧 OccupancyGrid
/map_server 应该是 active
Nav2 相关节点应该存在
```

如果 `/map_server` 不是 active，查看启动终端里 `map_server` 的报错，通常是地图文件路径或地图图片格式问题。

注意：

```text
1. /map 是静态地图，通常只发布一次，不要用 ros2 topic hz /map 判断是否正常。
2. lifecycle 节点名是 /map_server，不是 /map_serverer。
3. 如果 controller_server 配置失败，lifecycle_manager 会中断后续激活，/map_server 也可能停在 inactive。
```

如果看到类似下面的错误：

```text
Couldn't load critics! Caught exception: No critics defined for FollowPath
```

说明 Nav2 参数没有按预期加载。先执行：

```bash
./start_sim_nav.sh --build
```

新版脚本会在检测到 `src/` 源码比 `install/setup.bash` 更新时自动重新编译，避免继续使用旧的 install 配置。

## 当前版本边界

当前版本刻意不实现：

```text
1. slam_toolbox 建图
2. AMCL 定位
3. 真实底盘驱动
4. 真实雷达驱动
```

后续推荐顺序：

```text
1. 先把当前仿真导航跑稳
2. 后续需要更真实定位时，再加入 AMCL 或仿真真值定位
3. 再加入 slam_toolbox 建图
4. 最后接入真实底盘 base_driver_node 和真实雷达
```
