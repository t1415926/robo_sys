# my_robot_gazebo 中文说明

## 功能定位

`my_robot_gazebo` 提供 Gazebo 仿真环境和简单全向底盘仿真节点，用于在 3D 仿真中验证导航链路。

## 主要文件

```text
worlds/simple_obstacles.world       # 简单障碍物世界
launch/gazebo.launch.py             # 启动 Gazebo 基础环境
launch/gazebo_omni.launch.py        # 启动 Gazebo 全向底盘仿真
scripts/gazebo_omni_base_node.py    # 接收 /cmd_vel，更新 Gazebo 模型位姿并发布 /odom
config/controllers.yaml             # 早期 ros2_control 控制器配置，当前主流程不依赖
```

## 使用方法

推荐使用项目根目录脚本：

```bash
cd /home/dtc/robo_sys
./start_gazebo_omni_nav.sh --build
```

## 当前底盘接口

当前 Gazebo 全向节点直接支持：

```text
linear.x
linear.y
angular.z
```

也就是说，它适合验证全向底盘路径跟踪和 `/cmd_vel` 输出，不依赖差速底盘控制器。

## 注意事项

- 当前版本不使用复杂 `ros2_control` 差速链路。
- 如果后续接真实底盘，应优先保持 `/cmd_vel`、`/odom` 和 TF 接口一致。
