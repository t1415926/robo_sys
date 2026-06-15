# point_lio 中文说明

## 功能定位

`point_lio` 是 3D 激光惯导 SLAM 功能包，用于读取激光雷达和 IMU 数据，输出里程计、路径和点云地图。

## 当前项目中的作用

```text
/livox/lidar + /livox/imu
        ↓
point_lio / pointlio_mapping
        ↓
/Laser_map
/odom
map -> base_footprint
```

`/Laser_map` 会进一步由 `my_robot_navigation/scripts/pointcloud_to_occupancy_grid.py` 投影成 2D `/map`，用于 Nav2 或 A* 规划。

## 主要配置

```text
config/avia.yaml
config/mid360.yaml
config/mid360_mapping_quality.yaml
config/with_map.yaml
```

当前项目集成时主要使用：

```text
src/my_robot_bringup/config/pointlio_mid360_nav.yaml
```

该文件把输入话题统一为：

```text
/livox/lidar
/livox/imu
```

## 运行示例

```bash
cd /home/dtc/robo_sys
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run point_lio pointlio_mapping \
  --ros-args \
  --params-file src/my_robot_bringup/config/pointlio_mid360_nav.yaml
```

## 注意事项

- 建图前需要确认雷达和 IMU 话题频率稳定。
- 外参 `extrinsic_T`、`extrinsic_R` 会直接影响地图质量。
- 保存 PCD 时需要确认 `src/pointlio/PCD` 目录存在。
- 真实机器人导航建议先离线建图和修图，再进行闭环导航测试。
