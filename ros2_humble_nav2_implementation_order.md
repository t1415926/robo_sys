# ROS2 Humble Nav2 仿真到实车实现顺序文档

## 1. 实现原则

本实现顺序基于 `ros2_humble_nav2_sim_to_real.md` 中的总体规划，采用“先验证公共接口，再逐步替换实现层”的方式推进。

核心原则：

```text
1. 先跑通官方案例，再开发自己的机器人
2. 先仿真验证，再迁移真实底盘
3. 从第一天开始统一 ROS2 接口
4. Nav2 主体尽量不随硬件变化而变化
5. 每个阶段都要有明确验收条件
6. 初期不实现 SLAM，先使用简单模拟地图和仿真真值定位
```

必须保持稳定的接口：

| 接口 | 作用 |
|---|---|
| `/cmd_vel` | Nav2 输出速度控制指令 |
| `/odom` | 底盘里程计 |
| `/scan` | 2D 激光雷达数据 |
| `/tf` | 动态 TF |
| `/tf_static` | 静态 TF |
| `/map` | 栅格地图 |

推荐 TF 主线：

```text
map -> odom -> base_footprint -> base_link -> lidar_link
```

---

## 2. 总体实现顺序

推荐按以下顺序实现：

```text
1. 环境准备与官方案例验证
2. TurtleBot3 + Nav2 全流程跑通
3. 创建 ROS2 工作空间与功能包结构
4. 编写自有机器人 URDF / Xacro
5. 在 RViz2 中验证机器人模型和 TF
6. 接入 Gazebo Classic 仿真
7. 接入 ros2_control 和 diff_drive_controller
8. 仿真激光雷达与关键话题验证
9. 创建简单模拟地图和离散障碍物世界
10. 接入仿真真值定位
11. 接入 map_server 加载模拟地图
12. 接入完整 Nav2 完成目标点导航
13. 固化仿真 bringup 启动流程
14. 后续再加入 AMCL 或 slam_toolbox
15. 编写真实底盘 base_driver_node
16. 接入真实雷达和真实里程计
17. 固化真实 bringup 启动流程
18. 真车低速联调与 Nav2 参数调优
19. 后续工程化升级
```

---

## 3. 阶段一：环境准备与官方案例验证

### 3.1 目标

确认 Ubuntu 22.04、ROS2 Humble、Gazebo Classic、Nav2、TurtleBot3 等基础环境可用。

### 3.2 实现内容

安装基础包：

```bash
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3*
```

配置 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
```

### 3.3 验收标准

```text
1. ros2 命令可用
2. gazebo 可启动
3. rviz2 可启动
4. nav2_bringup 包存在
5. turtlebot3_gazebo 包存在
```

### 3.4 风险点

```text
1. ROS 环境变量未 source
2. Gazebo 和 ROS2 插件版本不匹配
3. TurtleBot3 模型环境变量未设置
```

---

## 4. 阶段二：TurtleBot3 + Nav2 全流程跑通

### 4.1 目标

在不写自定义代码的情况下，先理解 Nav2、Gazebo、定位、TF 和关键话题之间的关系。

### 4.2 实现内容

启动 TurtleBot3 仿真：

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

启动 Nav2：

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

检查关键话题：

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /cmd_vel
ros2 run tf2_tools view_frames
```

### 4.3 验收标准

```text
1. Gazebo 中 TurtleBot3 正常显示
2. RViz2 中能看到机器人、雷达、地图或代价地图
3. /cmd_vel、/odom、/scan、/tf、/tf_static 正常存在
4. TF 树无断裂
5. RViz2 中设置目标点后，Nav2 可以规划路径
6. 机器人可以向目标点移动并绕开障碍物
```

### 4.4 本阶段输出

```text
1. 官方案例运行记录
2. 关键话题和 TF 截图或记录
3. 对 /cmd_vel、/odom、/scan、/tf 关系的理解
```

---

## 5. 阶段三：创建自有 ROS2 工作空间和包结构

### 5.1 目标

建立后续可维护的工程目录，不把模型、仿真、导航和启动文件混在一个包中。

### 5.2 推荐结构

```text
my_robot_ws/
└── src/
    ├── my_robot_description/
    ├── my_robot_gazebo/
    ├── my_robot_navigation/
    └── my_robot_bringup/
```

### 5.3 各包职责

| 包名 | 职责 |
|---|---|
| `my_robot_description` | URDF、Xacro、mesh、RViz 配置 |
| `my_robot_gazebo` | Gazebo world、Gazebo 插件、仿真启动 |
| `my_robot_navigation` | Nav2、仿真定位、地图、后续 AMCL / SLAM 配置 |
| `my_robot_bringup` | 仿真和真车统一启动入口 |

### 5.4 验收标准

```text
1. colcon build 可以成功
2. source install/setup.bash 后能找到所有包
3. 包职责清晰，没有循环依赖
```

---

## 6. 阶段四：编写自有机器人 URDF / Xacro

### 6.1 目标

建立符合真实底盘尺寸和传感器安装位置的机器人模型。

### 6.2 实现内容

优先完成：

```text
1. base_footprint
2. base_link
3. left_wheel_link
4. right_wheel_link
5. lidar_link
6. imu_link，可选
```

需要配置：

```text
1. link 几何尺寸
2. joint 连接关系
3. visual
4. collision
5. inertial
6. 传感器安装位姿
```

### 6.3 验收标准

```text
1. robot_state_publisher 能正常发布 robot_description
2. RViz2 中机器人模型显示正确
3. TF 树包含 base_footprint、base_link、lidar_link
4. lidar_link 相对 base_link 的位置与真实安装一致
5. base_link 坐标方向正确，x 向前，y 向左，z 向上
```

### 6.4 暂不做内容

```text
1. 不急着调 Nav2
2. 不急着写真实底盘驱动
3. 不急着加复杂外观 mesh
```

---

## 7. 阶段五：接入 Gazebo Classic 仿真

### 7.1 目标

让自有机器人模型可以在 Gazebo 中生成，并具备基础物理属性。

### 7.2 实现内容

```text
1. 编写 Gazebo world
2. 编写 spawn_robot.launch.py
3. 将 URDF / Xacro 加载进 Gazebo
4. 为轮子配置合理 collision 和 inertial
5. 添加 Gazebo 传感器插件
```

### 7.3 验收标准

```text
1. Gazebo 中能看到自有机器人
2. 机器人不会一启动就翻倒或漂浮
3. 轮子和底盘碰撞体合理
4. /clock 正常发布
5. use_sim_time=true 生效
```

---

## 8. 阶段六：接入 ros2_control 和 diff_drive_controller

### 8.1 目标

使用标准控制接口让 Nav2 的 `/cmd_vel` 可以控制仿真底盘运动。

### 8.2 实现内容

```text
1. 在 Xacro 中加入 ros2_control 配置
2. 配置 diff_drive_controller
3. 配置 joint_state_broadcaster
4. 启动 controller_manager
5. 确认 /cmd_vel 输入和 /odom 输出
```

### 8.3 验收标准

```text
1. controller_manager 正常启动
2. diff_drive_controller 处于 active 状态
3. 发布 /cmd_vel 后机器人能在 Gazebo 中运动
4. /odom 正常发布
5. odom -> base_link 或 odom -> base_footprint TF 正常
6. 机器人前进、后退、左转、右转方向正确
```

### 8.4 重点检查

```text
1. 左右轮 joint 名称是否和 controller 配置一致
2. 轮半径是否正确
3. 轮距是否正确
4. 轮子速度方向是否正确
5. cmd_vel 的 linear.x 和 angular.z 是否符合 ROS 标准
```

---

## 9. 阶段七：接入仿真激光雷达

### 9.1 目标

让自有机器人在仿真中发布 Nav2 可用的 `/scan`。

### 9.2 实现内容

```text
1. 在 Gazebo 中添加 2D lidar 插件
2. 设置 scan topic 为 /scan
3. 设置 LaserScan frame_id 为 lidar_link
4. 确认 base_link -> lidar_link TF 正确
5. 在 RViz2 中显示 LaserScan
```

### 9.3 验收标准

```text
1. /scan 正常发布
2. /scan 的 frame_id 正确
3. RViz2 中雷达点云方向正确
4. 雷达数据与 Gazebo 障碍物位置一致
5. TF 可以从 map 或 odom 转换到 lidar_link
```

---

## 10. 阶段八：创建简单模拟地图和离散障碍物世界

### 10.1 目标

初期不做 SLAM 建图，先使用一张简单的已知模拟地图，让 Nav2 可以直接进行全局规划和局部避障。

### 10.2 实现内容

```text
1. 在 Gazebo world 中创建简单场景
2. 场景内只放少量离散障碍物
3. 障碍物可以先用 box、cylinder 等基础几何体
4. 在 my_robot_navigation/maps 中放置对应的 map.yaml 和 map.pgm
5. 地图坐标系采用 map，和 Gazebo world 原点保持一致
6. 先保证地图中的障碍物位置和 Gazebo 中的障碍物大致一致
```

### 10.3 验收标准

```text
1. Gazebo 中有一个简单可导航场景
2. 场景中存在若干离散障碍物
3. map.yaml 和 map.pgm 可以被 map_server 加载
4. RViz2 中能看到静态地图
5. 静态地图障碍物和 Gazebo 障碍物位置基本对应
```

### 10.4 风险点

```text
1. 地图原点和 Gazebo world 原点不一致
2. 地图分辨率设置不合理
3. 障碍物在地图中太窄，导致全局规划穿过障碍物
4. 机器人 footprint 或 robot_radius 和地图障碍物间距不匹配
```

---

## 11. 阶段九：接入仿真真值定位

### 11.1 目标

初期不使用 AMCL。先从仿真环境读取机器人真实位姿，发布 Nav2 需要的定位 TF，让导航系统优先跑通。

### 11.2 实现内容

```text
1. 编写 sim_localization_node 或使用 Gazebo ground truth 插件
2. 从 Gazebo 读取机器人在 world 中的真实位姿
3. 将 Gazebo world 视为 Nav2 的 map 坐标系
4. 结合 odom -> base_link，计算并发布 map -> odom
5. 保持 /odom 仍由 diff_drive_controller 发布
6. localization.launch.py 初期只启动仿真定位，不启动 AMCL
```

### 11.3 验收标准

```text
1. map -> odom TF 正常发布
2. odom -> base_link TF 正常发布
3. RViz2 Fixed Frame 设置为 map 后机器人位置正确
4. 机器人在 Gazebo 中移动时，RViz2 中位姿同步变化
5. 不需要 2D Pose Estimate 即可获得初始定位
```

### 11.4 暂不做内容

```text
1. 暂不追求高速导航
2. 暂不大范围调 controller 参数
3. 暂不启动 AMCL
4. 暂不启动 slam_toolbox
5. 先保证仿真真值定位稳定
```

---

## 12. 阶段十：接入 map_server 和完整 Nav2 路径规划

### 12.1 目标

加载简单模拟地图，使用仿真真值定位，实现自有机器人在仿真环境中自动规划路径、局部避障并到达目标点。

### 12.2 初期推荐模块

```text
1. NavFn Planner
2. DWB Controller
3. map_server
4. sim_localization_node
5. costmap_2d
6. bt_navigator
```

### 12.3 实现内容

```text
1. 编写 nav2_params.yaml
2. 配置 global_costmap
3. 配置 local_costmap
4. 配置 planner_server
5. 配置 controller_server
6. 配置 behavior_server
7. 配置 bt_navigator
8. 启动 map_server 加载模拟 map.yaml
9. 启动仿真定位节点发布 map -> odom
10. 编写 navigation.launch.py
```

### 12.4 验收标准

```text
1. Nav2 lifecycle 节点全部 active
2. RViz2 中可设置 Nav2 Goal
3. global path 正常生成
4. local path 正常生成
5. costmap 能正确显示障碍物
6. 机器人能低速到达目标点
7. 机器人可以绕开模拟地图中的离散障碍物
8. 机器人不会频繁原地转圈或反复恢复行为
```

### 12.5 初始参数建议

```yaml
max_vel_x: 0.4
min_vel_x: 0.0
max_vel_theta: 0.8
acc_lim_x: 0.5
acc_lim_theta: 1.0
xy_goal_tolerance: 0.15
yaw_goal_tolerance: 0.25
```

---

## 13. 阶段十一：固化仿真启动流程

### 13.1 目标

把分散启动命令整理成统一入口，降低后续调试复杂度。

### 13.2 实现内容

建议提供以下 launch：

```text
1. display.launch.py
2. gazebo.launch.py
3. sim_localization.launch.py
4. navigation.launch.py
5. sim_bringup.launch.py
```

### 13.3 sim_bringup 推荐包含

```text
1. robot_state_publisher
2. Gazebo
3. spawn_entity
4. controller_manager
5. joint_state_broadcaster
6. diff_drive_controller
7. RViz2，可选
```

### 13.4 验收标准

```text
1. 一条命令可以启动完整仿真底盘
2. 一条命令可以启动仿真真值定位
3. 一条命令可以加载模拟地图并启动导航
4. 所有 launch 都支持 use_sim_time 参数
```

---

## 14. 阶段十二：后续加入 AMCL 或 slam_toolbox

### 14.1 目标

在仿真导航闭环稳定后，再加入真实定位链路。优先加入 AMCL，最后再加入 slam_toolbox 建图。

### 14.2 推荐顺序

```text
1. 保留当前简单模拟地图
2. 用 AMCL 替换 sim_localization_node
3. 验证 AMCL 在模拟地图中收敛
4. AMCL 稳定后，再考虑加入 slam_toolbox
5. 使用 slam_toolbox 重新生成地图
6. 用新地图替换手工模拟地图
```

### 14.3 验收标准

```text
1. AMCL 可以发布 map -> odom
2. 设置初始位姿后粒子云可以收敛
3. 替换仿真真值定位后 Nav2 仍能正常导航
4. 后续 slam_toolbox 建图不会影响已经稳定的 Nav2 主配置
```

---

## 15. 阶段十三：编写真实底盘 base_driver_node

### 15.1 目标

让真实底盘能够使用和仿真一致的 `/cmd_vel` 输入，并输出 Nav2 需要的 `/odom` 和 TF。

### 15.2 实现内容

base_driver_node 需要完成：

```text
1. 订阅 /cmd_vel
2. 解析 linear.x 和 angular.z
3. 转换为左右轮速度或底盘协议指令
4. 通过串口、CAN、USB、Modbus 或网络发送控制命令
5. 读取编码器、轮速或底盘反馈
6. 计算里程计
7. 发布 /odom
8. 发布 odom -> base_link TF
9. 增加急停和速度限幅保护
```

### 15.3 差速换算

```text
v = linear.x
w = angular.z
L = 左右轮间距
R = 轮子半径
```

```text
v_left  = v - w * L / 2
v_right = v + w * L / 2
w_left  = v_left / R
w_right = v_right / R
```

### 15.4 验收标准

```text
1. ros2 topic pub /cmd_vel 可以控制真实底盘前进
2. 左转、右转方向正确
3. 停止发布 /cmd_vel 后底盘能及时停止
4. /odom 频率稳定，建议 30 Hz
5. odom -> base_link TF 频率稳定，建议 30 Hz
6. 里程计方向和距离基本正确
7. 时间戳使用当前 ROS 时间
```

### 15.5 初期保护策略

```text
1. 限制 max_vel_x <= 0.2 m/s
2. 限制 max_vel_theta <= 0.4 rad/s
3. 控制命令超时后自动刹停
4. 通信异常后自动停止底盘
5. 真车测试时先架空轮子，再落地低速测试
```

---

## 16. 阶段十四：接入真实雷达

### 16.1 目标

让真实雷达发布与仿真一致的 `/scan`，并保证 TF 与实际安装位置一致。

### 16.2 实现内容

```text
1. 安装真实雷达 ROS2 驱动
2. 设置输出话题为 /scan
3. 设置 frame_id 为 lidar_link
4. 测量雷达相对 base_link 的安装位置
5. 在 URDF 或 static_transform_publisher 中配置静态 TF
6. 在 RViz2 中检查雷达数据方向
```

### 16.3 验收标准

```text
1. /scan 正常发布
2. /scan 频率稳定，建议 10 Hz
3. frame_id 为 lidar_link
4. base_link -> lidar_link TF 正确
5. RViz2 中障碍物方向与真实环境一致
```

---

## 17. 阶段十五：固化真实 bringup 流程

### 17.1 目标

用真实底盘驱动和真实雷达替换 Gazebo，但保持 Nav2 配置尽量不变。

### 17.2 real_bringup 推荐包含

```text
1. robot_state_publisher
2. base_driver_node
3. lidar driver
4. imu driver，可选
5. AMCL 或后续 slam_toolbox
6. Nav2
7. RViz2，可选
```

### 17.3 关键参数变化

| 仿真 | 真车 |
|---|---|
| `use_sim_time=true` | `use_sim_time=false` |
| Gazebo `/scan` | 真实雷达 `/scan` |
| Gazebo `/odom` | 底盘驱动 `/odom` |
| Gazebo 控制底盘 | base_driver_node 控制底盘 |

### 17.4 验收标准

```text
1. real_bringup 可以正常启动
2. use_sim_time=false
3. /cmd_vel、/odom、/scan、/tf、/tf_static 全部正常
4. RViz2 中雷达和机器人模型对齐
5. AMCL 可以在真实地图中收敛
```

---

## 18. 阶段十六：真车低速导航与参数调优

### 18.1 目标

在真实底盘上低速完成目标点导航，并逐步优化稳定性。

### 18.2 初期真实参数建议

```yaml
max_vel_x: 0.2
max_vel_theta: 0.4
acc_lim_x: 0.3
acc_lim_theta: 0.6
xy_goal_tolerance: 0.25
yaw_goal_tolerance: 0.35
```

### 18.3 调试顺序

```text
1. 先测试 /cmd_vel 控制
2. 再测试 /odom 方向和距离
3. 再测试 /scan 和 TF 对齐
4. 再测试 AMCL 定位
5. 最后测试 Nav2 目标点导航
6. SLAM 建图后续单独加入
```

### 18.4 验收标准

```text
1. 机器人低速导航不失控
2. AMCL 定位稳定
3. 局部代价地图障碍物位置正确
4. Nav2 能规划路径
5. 机器人能到达短距离目标点
6. 机器人能避开静态障碍物
7. 急停和通信超时保护有效
```

---

## 19. 阶段十七：后续工程化升级

### 19.1 可选升级方向

```text
1. 将 base_driver_node 升级为 ros2_control hardware_interface
2. 引入 robot_localization 融合轮速和 IMU
3. 将 NavFn Planner 升级为 SmacPlanner2D
4. 将 DWB Controller 升级为 Regulated Pure Pursuit
5. 增加速度平滑器 velocity_smoother
6. 增加行为树任务逻辑
7. 增加自动巡航、多点导航、任务调度
8. 增加 rosbag2 数据记录和回放流程
9. 加入 slam_toolbox 建图流程
```

### 19.2 升级前提

```text
1. 仿真导航已经稳定
2. 真车 /cmd_vel、/odom、/scan、/tf 已稳定
3. 真车低速导航已经可重复成功
4. 已有基础安全保护
```

---

## 20. 每次进入下一阶段前的统一检查

进入下一阶段前都建议检查：

```text
[ ] 当前阶段目标已经完成
[ ] 当前阶段有可重复启动命令
[ ] 当前阶段关键话题稳定
[ ] 当前阶段 TF 无断裂
[ ] 当前阶段参数已保存到配置文件
[ ] 当前阶段问题已记录
[ ] 当前阶段可以回退到上一个稳定状态
```

---

## 21. 最小可交付版本定义

### 21.1 仿真最小可交付版本

```text
1. 自有机器人模型可在 Gazebo 中运行
2. /cmd_vel 可控制机器人运动
3. /odom、/scan、/tf 正常
4. map_server 可加载简单模拟地图
5. 仿真真值定位可发布 map -> odom
6. Nav2 可在离散障碍物地图中完成目标点导航
```

### 21.2 真车最小可交付版本

```text
1. base_driver_node 可控制真实底盘
2. 真实底盘发布 /odom 和 odom -> base_link TF
3. 真实雷达发布 /scan
4. AMCL 可在真实地图中定位
5. Nav2 可低速完成短距离目标点导航
6. 急停和通信超时保护可用
```

---

## 22. 推荐当前第一批实施任务

如果从零开始，第一批任务只做以下内容：

```text
1. 跑通 TurtleBot3 + Gazebo + Nav2
2. 记录 /cmd_vel、/odom、/scan、/tf 的实际表现
3. 创建 my_robot_ws 和四个基础功能包
4. 编写最简 URDF / Xacro
5. 在 RViz2 中验证自有机器人模型和 TF
```

完成这 5 项后，再进入 Gazebo、ros2_control 和 Nav2 参数配置。
