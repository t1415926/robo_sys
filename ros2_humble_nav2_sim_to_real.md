# ROS2 Humble 仿真路径规划方案：从仿真到真实底盘迁移

## 1. 项目目标

本项目目标是搭建一套基于 **ROS2 Humble** 的移动机器人路径规划与导航系统，先在仿真环境中完成建图、定位、路径规划、避障和速度控制，再逐步迁移到真实移动底盘上。

核心思路：

> 仿真阶段和真实底盘阶段尽量保持相同的 ROS2 接口，包括 `/cmd_vel`、`/odom`、`/scan`、`/tf` 等，使后续迁移时主要替换底盘驱动和传感器驱动，而不需要大幅修改导航系统。

---

## 2. 推荐技术栈

| 模块 | 推荐方案 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS 版本 | ROS2 Humble |
| 仿真平台 | Gazebo Classic |
| 导航框架 | Nav2 |
| 建图 | slam_toolbox |
| 定位 | AMCL |
| 控制接口 | ros2_control |
| 差速底盘控制 | diff_drive_controller |
| 可视化 | RViz2 |
| 机器人模型 | URDF / Xacro |
| 地图服务 | nav2_map_server |
| 真车接口 | 自定义 base_driver_node 或 ros2_control hardware_interface |

---

## 3. 系统总体架构

### 3.1 仿真阶段架构

```text
Gazebo Classic
   │
   ├── 机器人模型 URDF / Xacro
   │      ├── base_link
   │      ├── wheel_left
   │      ├── wheel_right
   │      ├── lidar_link
   │      └── camera_link 可选
   │
   ├── 传感器仿真
   │      ├── /scan
   │      └── /imu 可选
   │
   ├── ros2_control
   │      └── diff_drive_controller
   │             ├── 订阅 /cmd_vel
   │             └── 发布 /odom
   │
   └── TF
          map → odom → base_link → lidar_link

Nav2
   │
   ├── map_server
   ├── amcl / slam_toolbox
   ├── planner_server
   ├── controller_server
   ├── behavior_server
   ├── bt_navigator
   └── costmap_2d
```

### 3.2 真实底盘阶段架构

```text
Nav2
   │
   └── 发布 /cmd_vel
          │
          ▼
真实底盘驱动节点
   │
   ├── 串口 / CAN / USB / Modbus
   │
   ├── 控制真实底盘运动
   │
   ├── 读取编码器 / 轮速 / IMU
   │
   └── 发布 /odom 和 TF

真实雷达
   │
   └── 发布 /scan

Nav2 继续使用：
   ├── /cmd_vel
   ├── /odom
   ├── /scan
   ├── /tf
   └── /map
```

---

## 4. 推荐开发路线

## 阶段一：先跑通 TurtleBot3 + Nav2

这一阶段不建议直接写自己的机器人模型，先使用 TurtleBot3 验证 ROS2 Humble、Gazebo、Nav2 是否正常。

### 4.1 安装基础包

```bash
sudo apt update

sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3*
```

### 4.2 启动 TurtleBot3 仿真

```bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 4.3 启动导航

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

或者使用 TurtleBot3 自带导航 launch 文件。

### 4.4 检查关键话题

```bash
ros2 topic list
```

重点关注：

```text
/cmd_vel
/odom
/scan
/tf
/tf_static
/map
```

查看雷达数据：

```bash
ros2 topic echo /scan
```

查看里程计：

```bash
ros2 topic echo /odom
```

查看速度指令：

```bash
ros2 topic echo /cmd_vel
```

查看 TF 树：

```bash
ros2 run tf2_tools view_frames
```

### 4.5 阶段一目标

完成以下内容：

```text
1. Gazebo 中机器人可以正常运动
2. RViz2 中可以看到激光雷达数据
3. TF 树正常
4. 可以通过 RViz2 设置目标点
5. Nav2 能够规划路径
6. 机器人能自动导航并避障
```

---

## 阶段二：建立自己的机器人仿真模型

当 TurtleBot3 流程跑通后，再搭建自己的机器人模型。

建议先建立一个差速底盘模型。即使真实底盘是四轮差速、履带式底盘，也可以先等效为差速模型。

---

## 5. 推荐 ROS2 工作空间结构

```text
my_robot_ws/
└── src/
    ├── my_robot_description/
    │   ├── urdf/
    │   │   └── robot.urdf.xacro
    │   ├── meshes/
    │   ├── launch/
    │   │   └── display.launch.py
    │   └── rviz/
    │
    ├── my_robot_gazebo/
    │   ├── worlds/
    │   │   └── warehouse.world
    │   ├── launch/
    │   │   └── gazebo.launch.py
    │   └── config/
    │
    ├── my_robot_navigation/
    │   ├── config/
    │   │   ├── nav2_params.yaml
    │   │   ├── mapper_params_online_async.yaml
    │   │   └── amcl_params.yaml
    │   ├── maps/
    │   └── launch/
    │       ├── slam.launch.py
    │       ├── localization.launch.py
    │       └── navigation.launch.py
    │
    └── my_robot_bringup/
        └── launch/
            ├── sim_bringup.launch.py
            └── real_bringup.launch.py
```

### 各功能包作用

| 包名 | 作用 |
|---|---|
| `my_robot_description` | 存放机器人 URDF、Xacro、mesh、RViz 配置 |
| `my_robot_gazebo` | 存放 Gazebo 世界、仿真启动文件 |
| `my_robot_navigation` | 存放 Nav2、SLAM、AMCL、地图等配置 |
| `my_robot_bringup` | 统一启动仿真或真实机器人系统 |

---

## 6. 坐标系设计

路径规划系统中，TF 坐标系非常重要。建议从仿真开始就按照真实机器人标准设计。

### 6.1 推荐 TF 树

```text
map
 └── odom
      └── base_footprint
           └── base_link
                ├── lidar_link
                ├── imu_link
                └── camera_link
```

### 6.2 坐标系含义

| 坐标系 | 含义 |
|---|---|
| `map` | 全局地图坐标系，由 SLAM 或 AMCL 维护 |
| `odom` | 里程计坐标系，短时间连续，但长期会漂移 |
| `base_footprint` | 机器人在地面上的投影中心 |
| `base_link` | 机器人本体坐标系 |
| `lidar_link` | 激光雷达坐标系 |
| `imu_link` | IMU 坐标系 |
| `camera_link` | 相机坐标系 |

### 6.3 Nav2 重点依赖的 TF

```text
map → odom
odom → base_link / base_footprint
base_link → lidar_link
```

仿真阶段：

```text
Gazebo / ros2_control 发布 odom → base_link
slam_toolbox 或 AMCL 发布 map → odom
robot_state_publisher 发布 base_link → lidar_link
```

真实底盘阶段：

```text
真实底盘驱动发布 odom → base_link
AMCL 或 SLAM 发布 map → odom
robot_state_publisher 发布 base_link → lidar_link
```

---

## 7. 关键话题设计

从仿真开始，建议统一使用以下话题名：

| 话题 | 作用 |
|---|---|
| `/cmd_vel` | Nav2 输出的速度控制指令 |
| `/odom` | 底盘里程计 |
| `/scan` | 2D 激光雷达数据 |
| `/tf` | 动态坐标变换 |
| `/tf_static` | 静态坐标变换 |
| `/map` | 栅格地图 |
| `/goal_pose` | 目标点 |
| `/imu` | IMU 数据，可选 |

后续迁移到真实底盘时，只要保证真实设备仍然发布这些话题，Nav2 配置可以基本不变。

---

## 8. 建图方案

建图阶段推荐使用：

```text
slam_toolbox
```

### 8.1 仿真建图流程

启动仿真：

```bash
ros2 launch my_robot_bringup sim_bringup.launch.py
```

启动 slam_toolbox：

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

启动键盘遥控：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

保存后会得到：

```text
map.yaml
map.pgm
```

### 8.2 真实底盘建图流程

真实底盘建图流程基本相同，只是需要替换数据来源：

```text
仿真 /scan  →  真实雷达 /scan
仿真 /odom  →  真实底盘 /odom
use_sim_time:=true  →  use_sim_time:=false
```

真实底盘建图启动逻辑：

```bash
ros2 launch my_robot_bringup real_bringup.launch.py
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false
```

---

## 9. 定位与导航方案

建图完成后，导航阶段推荐使用：

```text
map_server + AMCL + Nav2
```

### 9.1 启动定位

```bash
ros2 launch my_robot_navigation localization.launch.py map:=/home/xxx/map.yaml
```

### 9.2 启动导航

```bash
ros2 launch my_robot_navigation navigation.launch.py
```

### 9.3 RViz2 操作流程

```text
1. Fixed Frame 设置为 map
2. 使用 2D Pose Estimate 设置机器人初始位姿
3. 使用 Nav2 Goal 设置导航目标点
4. 观察全局路径、局部路径和代价地图
```

---

## 10. Nav2 模块选择

| 功能 | 推荐模块 |
|---|---|
| 全局规划 | NavFn Planner |
| 局部控制 | DWB Controller |
| 定位 | AMCL |
| 建图 | slam_toolbox |
| 地图服务 | map_server |
| 行为控制 | bt_navigator |
| 避障地图 | costmap_2d |

### 10.1 初期推荐组合

```text
NavFn Planner + DWB Controller
```

优点：

```text
1. 配置简单
2. 资料多
3. 适合差速底盘
4. 适合初期验证
```

### 10.2 后期可升级组合

```text
SmacPlanner2D + Regulated Pure Pursuit
```

适合：

```text
1. 希望轨迹更平滑
2. 机器人速度更高
3. 需要更自然的转弯路径
4. 后续做实际巡检、仓储导航等任务
```

---

## 11. ros2_control 与底盘控制

建议从仿真阶段就使用：

```text
ros2_control + diff_drive_controller
```

这样后续迁移到真实底盘时，结构更清晰。

### 11.1 仿真阶段

```text
Nav2 发布 /cmd_vel
        ↓
diff_drive_controller
        ↓
Gazebo 中的轮子运动
        ↓
发布 /odom
```

### 11.2 真实底盘阶段

真实底盘有两种迁移方案。

### 方案 A：自定义 base_driver_node

结构简单，适合初期实车调试。

```text
Nav2
  │
  └── /cmd_vel
          ↓
base_driver_node
          ↓
串口 / CAN / USB / Modbus
          ↓
真实底盘控制器
          ↓
读取编码器 / 轮速
          ↓
发布 /odom 和 TF
```

base_driver_node 需要完成：

```text
1. 订阅 /cmd_vel
2. 将 linear.x 和 angular.z 转换成底盘控制协议
3. 通过串口、CAN 或 USB 发送给底盘
4. 读取底盘反馈
5. 计算并发布 /odom
6. 发布 odom → base_link 的 TF
```

### 方案 B：ros2_control hardware_interface

结构更标准，适合长期项目和工程化。

```text
Nav2
  │
  └── /cmd_vel
          ↓
diff_drive_controller
          ↓
hardware_interface
          ↓
真实底盘控制器
```

优点：

```text
1. 仿真和真车接口统一
2. 更符合 ROS2 控制框架
3. 后续可扩展机械臂、云台、升降机构等
```

缺点：

```text
1. 初期开发复杂度更高
2. 需要理解 ros2_control 的硬件接口机制
```

### 11.3 建议选择

初期建议：

```text
先写自定义 base_driver_node
```

等真车跑通后，再考虑封装为：

```text
ros2_control hardware_interface
```

---

## 12. 真实底盘迁移重点

真实底盘迁移时，不建议修改 Nav2 主体。主要替换以下部分：

| 仿真阶段 | 真实阶段 |
|---|---|
| Gazebo 发布 `/scan` | 真实雷达驱动发布 `/scan` |
| Gazebo 发布 `/odom` | 底盘驱动发布 `/odom` |
| Gazebo 中机器人运动 | 真实底盘运动 |
| use_sim_time=true | use_sim_time=false |

核心接口保持：

```text
/cmd_vel
/odom
/scan
/tf
/tf_static
/map
```

---

## 13. 真实底盘驱动节点设计

### 13.1 输入

```text
订阅：
/cmd_vel
```

消息类型：

```text
geometry_msgs/msg/Twist
```

主要使用：

```text
linear.x   # 前进速度，单位 m/s
angular.z  # 角速度，单位 rad/s
```

### 13.2 输出

```text
发布：
/odom
/tf
```

`/odom` 消息类型：

```text
nav_msgs/msg/Odometry
```

TF：

```text
odom → base_link
```

### 13.3 底盘通信方式

根据真实底盘支持的协议选择：

```text
1. 串口
2. CAN
3. USB 虚拟串口
4. Modbus RTU
5. Ethernet TCP / UDP
```

### 13.4 差速底盘速度换算

假设：

```text
v = linear.x
w = angular.z
L = 左右轮间距
```

左右轮线速度：

```text
v_left  = v - w * L / 2
v_right = v + w * L / 2
```

如果需要转换为轮子角速度：

```text
w_left  = v_left / R
w_right = v_right / R
```

其中：

```text
R = 轮子半径
L = 轮距
```

---

## 14. 传感器配置

### 14.1 激光雷达

导航初期最推荐使用 2D 激光雷达。

要求发布：

```text
/scan
```

消息类型：

```text
sensor_msgs/msg/LaserScan
```

需要配置静态 TF：

```text
base_link → lidar_link
```

示例：

```bash
ros2 run tf2_ros static_transform_publisher \
0.25 0 0.35 0 0 0 base_link lidar_link
```

含义：

```text
x = 0.25 m
y = 0
z = 0.35 m
roll = 0
pitch = 0
yaw = 0
```

实际参数需要根据雷达安装位置测量。

### 14.2 IMU

IMU 不是 2D Nav2 必需，但真实底盘上建议保留。

发布话题：

```text
/imu
```

消息类型：

```text
sensor_msgs/msg/Imu
```

作用：

```text
1. 辅助里程计
2. 提高转向估计稳定性
3. 后续可融合 robot_localization
```

### 14.3 里程计

里程计是 Nav2 稳定运行的关键。

要求：

```text
1. 连续
2. 不突跳
3. 时间戳正确
4. 坐标方向正确
5. 发布频率稳定
```

建议频率：

```text
/odom：30 Hz
/tf：30 Hz
/scan：10 Hz
/cmd_vel 控制频率：20~50 Hz
```

---

## 15. 参数初始建议

### 15.1 机器人尺寸参数

根据真实机器人尺寸设置：

```yaml
robot_radius: 0.35
```

如果机器人不是圆形，也可以使用 footprint：

```yaml
footprint: "[[0.4, 0.3], [0.4, -0.3], [-0.4, -0.3], [-0.4, 0.3]]"
```

### 15.2 代价地图参数

```yaml
inflation_radius: 0.45
cost_scaling_factor: 3.0
```

含义：

```text
inflation_radius：障碍物膨胀半径
cost_scaling_factor：障碍物代价衰减速度
```

### 15.3 速度参数

仿真初期：

```yaml
max_vel_x: 0.4
min_vel_x: 0.0
max_vel_theta: 0.8
acc_lim_x: 0.5
acc_lim_theta: 1.0
```

真实底盘初期建议更保守：

```yaml
max_vel_x: 0.2
max_vel_theta: 0.4
acc_lim_x: 0.3
acc_lim_theta: 0.6
```

### 15.4 目标容差

```yaml
xy_goal_tolerance: 0.15
yaw_goal_tolerance: 0.25
```

如果真实底盘控制精度较差，可以放宽：

```yaml
xy_goal_tolerance: 0.25
yaw_goal_tolerance: 0.35
```

---

## 16. 推荐学习与实现顺序

### 第一步：跑通官方案例

```text
TurtleBot3 + Gazebo + Nav2
```

目标：

```text
理解 /cmd_vel、/odom、/scan、/tf 的关系
```

### 第二步：搭建自己的 URDF 模型

完成：

```text
1. base_link
2. 左右轮
3. lidar_link
4. imu_link 可选
5. robot_state_publisher
```

目标：

```text
RViz2 中能正确显示机器人模型和 TF
```

### 第三步：接入 Gazebo

完成：

```text
1. 在 Gazebo 中生成机器人
2. 添加 2D 激光雷达插件
3. 添加轮子物理属性
4. 添加 ros2_control
```

目标：

```text
Gazebo 中机器人能被 /cmd_vel 控制运动
```

### 第四步：接入 slam_toolbox 建图

完成：

```text
1. 启动 slam_toolbox
2. 遥控机器人运动
3. 生成地图
4. 保存地图
```

目标：

```text
得到 map.yaml 和 map.pgm
```

### 第五步：接入 AMCL 定位

完成：

```text
1. 加载已有地图
2. 启动 AMCL
3. 设置初始位姿
4. 观察粒子云收敛
```

目标：

```text
机器人能在地图中稳定定位
```

### 第六步：接入 Nav2 路径规划

完成：

```text
1. 启动 planner_server
2. 启动 controller_server
3. 启动 bt_navigator
4. 设置导航目标点
```

目标：

```text
机器人能自动规划路径并移动到目标点
```

### 第七步：迁移真实底盘

完成：

```text
1. 写 base_driver_node
2. 订阅 /cmd_vel
3. 控制真实底盘运动
4. 发布 /odom
5. 发布 odom → base_link TF
6. 接入真实雷达 /scan
```

目标：

```text
真实底盘能使用同一套 Nav2 配置进行导航
```

---

## 17. 常见问题与排查方法

### 17.1 RViz 中看不到机器人模型

检查：

```bash
ros2 topic echo /robot_description
ros2 topic echo /tf_static
```

可能原因：

```text
1. robot_state_publisher 没启动
2. URDF 加载失败
3. joint_state_publisher 没发布关节状态
4. fixed frame 设置错误
```

### 17.2 Nav2 不规划路径

检查：

```text
1. 是否有 map
2. 是否有 map → odom
3. 是否有 odom → base_link
4. 机器人是否在地图内
5. 初始位姿是否设置正确
6. costmap 是否正常
```

常用命令：

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /map
ros2 topic echo /scan
ros2 topic echo /odom
```

### 17.3 机器人原地转圈

可能原因：

```text
1. 轮子方向反了
2. 左右轮速度符号反了
3. angular.z 转换错误
4. base_link 坐标方向错误
5. 雷达方向和 TF 不一致
```

### 17.4 地图和雷达对不上

可能原因：

```text
1. lidar_link 静态 TF 错误
2. 雷达安装角度不对
3. 雷达数据 frame_id 不对
4. base_link 和 lidar_link 关系错误
5. use_sim_time 设置错误
```

### 17.5 真车定位不稳定

可能原因：

```text
1. 里程计漂移严重
2. 轮子打滑
3. 雷达数据噪声大
4. AMCL 参数不合适
5. 初始位姿设置不准确
6. 环境特征太少
```

---

## 18. 真车迁移检查清单

在真实底盘上运行 Nav2 前，先检查：

```text
[ ] /cmd_vel 可以控制底盘运动
[ ] /odom 正常发布
[ ] /scan 正常发布
[ ] /tf 正常
[ ] /tf_static 正常
[ ] base_link 坐标方向正确
[ ] lidar_link 安装位置正确
[ ] 轮距参数正确
[ ] 轮径参数正确
[ ] 速度单位是 m/s 和 rad/s
[ ] 时间戳正常
[ ] use_sim_time=false
[ ] RViz2 中雷达点云和地图对齐
[ ] AMCL 可以收敛
[ ] 低速导航正常
```

---

## 19. 项目包装名称

可以命名为：

```text
基于 ROS2 Humble 与 Nav2 的移动机器人仿真到实车路径规划系统
```

或者：

```text
面向真实底盘迁移的 ROS2 Nav2 仿真导航与路径规划平台
```

---

## 20. 简历描述示例

```text
基于 ROS2 Humble、Gazebo Classic、Nav2 和 slam_toolbox 搭建移动机器人仿真导航系统，完成 URDF/Xacro 机器人建模、激光雷达仿真、ros2_control 差速底盘控制、SLAM 建图、AMCL 定位与目标点导航。系统保持 /cmd_vel、/odom、/scan、TF 等接口与真实底盘一致，支持后续替换真实雷达和底盘驱动，实现从仿真到实车的平滑迁移。
```

---

## 21. 最终推荐路线总结

```text
1. TurtleBot3 跑通 Nav2
2. 理解 /cmd_vel、/odom、/scan、/tf
3. 建立自己的 URDF / Xacro 模型
4. 接入 Gazebo 仿真
5. 使用 ros2_control 控制底盘
6. 使用 slam_toolbox 建图
7. 使用 AMCL 定位
8. 使用 Nav2 规划路径
9. 编写真实底盘驱动
10. 替换真实雷达和真实底盘
11. 保持 ROS2 接口不变
12. 真车低速测试与参数调优
```

核心原则：

> 仿真阶段就按照真实底盘的接口设计系统，后续迁移时只替换硬件驱动层，不重写导航逻辑。
