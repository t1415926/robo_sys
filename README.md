# ROS2 Humble Nav2 初版仿真导航项目

本项目是 `ros2_humble_nav2_implementation_order.md` 的初步实现版本。当前版本不做 SLAM，不启动 AMCL，而是使用：

```text
Gazebo Classic + ros2_control + diff_drive_controller
静态简单地图 map_server
仿真真值定位 sim_localization_node
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
├── my_robot_gazebo        # Gazebo world、ros2_control 控制器配置
├── my_robot_navigation    # Nav2 参数、简单地图、仿真定位节点
└── my_robot_bringup       # 一键启动仿真导航
```

协作提交和推送说明见：

```text
docs/git_workflow.md
```

初版公共接口：

| 话题 / TF | 说明 |
|---|---|
| `/cmd_vel` | Nav2 输出速度指令 |
| `/odom` | 桥接后的标准里程计 |
| `/map` | 静态模拟地图 |
| `map -> odom` | 仿真真值定位节点发布 |
| `odom -> base_footprint` | diff_drive_controller 发布 |

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
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-teleop-twist-keyboard
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

## 启动完整仿真导航

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
1. Gazebo 简单障碍物世界
2. 自定义差速机器人
3. ros2_control 差速控制器
4. /map 简单 2D 栅格地图
5. sim_localization_node 仿真真值定位
6. Nav2 路径规划和控制
7. RViz2
```

## 在 RViz2 中使用

RViz2 的 Fixed Frame 已设置为 `map`。

由于当前使用仿真真值定位，不需要点击 `2D Pose Estimate`。直接点击 `Nav2 Goal`，在地图空白区域设置目标点即可。

Nav2 会根据 `simple_map.pgm` 中的障碍物占据栅格自动规划路径，并控制机器人到达目标点。

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

## 单独测试底盘控制

启动仿真后，另开终端：

```bash
cd /home/dtc/robo_sys
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘控制会发布 `/cmd_vel`，桥接节点会转发到 diff_drive_controller。

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
2. 用 AMCL 替换 sim_localization_node
3. 再加入 slam_toolbox 建图
4. 最后接入真实底盘 base_driver_node 和真实雷达
```
