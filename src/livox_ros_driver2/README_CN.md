# livox_ros_driver2 中文说明

## 功能定位

`livox_ros_driver2` 是 Livox 雷达 ROS2 驱动，用于接入 MID360、HAP 等 Livox 设备，并发布点云和 IMU 话题。

## 当前项目中的作用

真实雷达链路：

```text
Livox MID360
        ↓
livox_ros_driver2
        ↓
/livox/lidar
/livox/imu
        ↓
point_lio
```

## 主要文件

```text
config/MID360_config.json              # MID360 网络配置
launch_ROS2/msg_MID360_launch.py       # MID360 ROS2 启动入口
msg/CustomMsg.msg                      # Livox 自定义点云消息
msg/CustomPoint.msg                    # Livox 自定义点格式
```

## 网络配置

需要重点检查：

```text
config/MID360_config.json
```

其中：

```text
host_net_info.*_ip     # 电脑有线网卡 IP
lidar_configs[].ip     # 雷达 IP
```

启动前建议先检查：

```bash
ip addr
ping 192.168.1.12
```

## 启动示例

```bash
cd /home/dtc/robo_sys
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

检查话题：

```bash
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```

## 注意事项

- `msg_MID360_launch.py` 中 `xfer_format=1` 表示发布 Livox CustomMsg，适合当前 PointLIO 接入。
- 如果实际话题名不是 `/livox/lidar` 和 `/livox/imu`，建议在 launch 中 remap，或者同步修改 PointLIO 配置。
- 真实机器人运行时建议使用固定有线网卡 IP。
