# my_robot_description 中文说明

## 功能定位

`my_robot_description` 负责机器人模型描述和 RViz 模型显示。当前项目使用简单全向底盘模型，主要用于仿真、A* 规划示例和 RViz 可视化。

## 主要文件

```text
urdf/simple_omni_robot.urdf.xacro   # 当前主要使用的简单全向底盘模型
urdf/my_robot.urdf.xacro            # 早期基础机器人模型
launch/display.launch.py            # 单独查看机器人模型
rviz/display.rviz                   # 模型显示 RViz 配置
```

## 使用方法

单独查看模型：

```bash
cd /home/dtc/robo_sys
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_description display.launch.py
```

在完整系统中，一般不需要单独启动本包，`my_robot_bringup` 的 launch 会自动调用 `robot_state_publisher` 加载模型。

## 坐标系约定

```text
base_footprint  # 机器人底盘在地面的投影坐标系
base_link       # 机器人主体坐标系
```

后续接入真实机器人时，需要根据实际车体尺寸和传感器安装位置更新 URDF。
