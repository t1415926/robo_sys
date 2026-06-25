# 真实雷达接入、建图与地图修整流程

本文档说明如何把真实 3D 激光雷达接入当前项目，使用 PointLIO 建图，再生成 Nav2 可用于规划的 2D 栅格地图。当前项目已经包含：

```text
src/livox_ros_driver2              # Livox 雷达 ROS2 驱动
src/pointlio                       # 3D 激光惯导 SLAM
src/my_robot_navigation/scripts/pointcloud_to_occupancy_grid.py
src/my_robot_bringup/config/pointlio_mid360_nav.yaml
```

当前推荐先完成“离线建图 + 人工修整 + 固化 2D 地图”，再接入实车闭环导航。这样调试风险低，地图质量也更可控。

## 总体链路

真实雷达建图链路：

```text
Livox MID360 / Avia
        ↓
livox_ros_driver2
        ↓
/livox/lidar + /livox/imu
        ↓
PointLIO
        ↓
/Laser_map + /odom + map -> base_footprint
        ↓
点云保存 / 点云修整 / 2D grid 投影
        ↓
/map nav_msgs/OccupancyGrid 或保存后的 map.yaml + map.pgm
        ↓
Nav2 全局规划 /plan
```

实时导航链路：

```text
真实底盘里程计 / IMU / 雷达
        ↓
定位或 SLAM 输出 map -> odom / odom -> base_footprint
        ↓
Nav2 读取静态或动态 /map
        ↓
Nav2 输出 /cmd_vel
        ↓
底盘控制器
```

## 需要先测量和确认的参数

### 1. 车体几何参数

这些参数影响 2D grid 投影、Nav2 footprint、障碍物膨胀和路径可通行性。

| 参数 | 含义 | 建议记录方式 | 用途 |
|---|---|---|---|
| `base_footprint` 到地面高度 | 车体参考点离地高度，通常可取 0 | 用尺量或按 URDF 定义 | TF、机器人模型 |
| 雷达到地面高度 `lidar_z_ground` | 雷达坐标系原点到地面距离 | 垂直测量，单位 m | 估算投影过滤高度 |
| 雷达到 `base_footprint` 的平移 `x,y,z` | 雷达相对车体中心的位置 | 前为 +x，左为 +y，上为 +z | PointLIO 外参、URDF |
| 雷达到 `base_footprint` 的姿态 `roll,pitch,yaw` | 雷达安装角 | 水平仪、标定或机械图纸 | PointLIO 外参、TF |
| 车体外形尺寸 | 长、宽、高 | 实测最大外包络 | Nav2 footprint |
| 安全外扩距离 | 车体外轮廓到障碍物的安全余量 | 0.05 到 0.20 m 起步 | costmap inflation |
| 最小通过宽度 | 机器人实际能通过的通道宽度 | 实测或场地约束 | 判断地图是否需要修整 |

示例记录：

```text
base_footprint 位于车体几何中心地面投影
车体长 0.60 m，宽 0.45 m
雷达相对 base_footprint: x=0.10 m, y=0.00 m, z=0.35 m
雷达安装姿态: roll=0, pitch=0, yaw=0
安全外扩: 0.10 m
```

### 2. PointLIO 外参

当前配置文件：

```text
src/my_robot_bringup/config/pointlio_mid360_nav.yaml
```

重点参数：

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"

mapping:
  extrinsic_T: [-0.011, -0.02329, 0.04412]
  extrinsic_R: [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
```

`extrinsic_T` 和 `extrinsic_R` 是 LiDAR 与 IMU 之间的外参，不一定等于 LiDAR 到车体中心的外参。使用 MID360 内置 IMU 时，如果雷达和 IMU 已在设备内部标定，通常先保持现有值或使用设备推荐值；如果换安装方式、外置 IMU 或 SLAM 明显漂移，再做专门标定。

调试顺序：

1. 先保持当前外参，低速采集一段直线和转弯数据。
2. 查看 `/path` 是否明显扭曲，`/Laser_map` 墙面是否重影。
3. 如果转弯时地图分层、墙体发散，优先检查时间同步、IMU 方向和外参。

### 3. Livox 网络和驱动参数

配置文件：

```text
src/livox_ros_driver2/config/MID360_config.json
src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
```

需要确认：

| 参数 | 文件 | 说明 |
|---|---|---|
| `host_net_info.*_ip` | `MID360_config.json` | 电脑有线网卡 IP |
| `lidar_configs[].ip` | `MID360_config.json` | 雷达 IP |
| `xfer_format` | `msg_MID360_launch.py` | `1` 表示 Livox CustomMsg，PointLIO 当前推荐 |
| `publish_freq` | `msg_MID360_launch.py` | 一般先用 10 Hz |
| `frame_id` | `msg_MID360_launch.py` | 建议统一为 `livox_frame` |

网络检查：

```bash
ip addr
ping 192.168.1.12
```

启动驱动：

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
ros2 topic echo /livox/imu --once
```

如果驱动实际输出的话题不是 `/livox/lidar`、`/livox/imu`，优先在 launch 中 remap 到这两个标准话题，或者修改 `pointlio_mid360_nav.yaml`。

## 建图流程

### 阶段 1：静态检查

机器人上电后先不要运动：

1. 启动雷达驱动。
2. 确认 `/livox/lidar` 和 `/livox/imu` 有稳定频率。
3. 打开 RViz2，查看点云方向是否符合预期。
4. 静止 10 到 20 秒，观察 IMU 是否稳定，PointLIO 初始化是否正常。

建议记录一段原始 bag，便于反复调试：

```bash
mkdir -p /home/dtc/robo_sys/bags
ros2 bag record \
  /livox/lidar \
  /livox/imu \
  /tf \
  /tf_static \
  -o /home/dtc/robo_sys/bags/real_lidar_mapping_test
```

### 阶段 2：低速建图

建图时尽量满足：

- 起步前静止 5 到 10 秒。
- 低速移动，避免急加速和原地高速旋转。
- 每条走廊或区域至少走一次闭合路径。
- 经过门口、窄通道、转角时降低速度。
- 不要让大量动态物体长期停在雷达视野内。

运行 PointLIO：

```bash
cd /home/dtc/robo_sys
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run point_lio pointlio_mapping \
  --ros-args \
  --params-file src/my_robot_bringup/config/pointlio_mid360_nav.yaml
```

常用检查：

```bash
ros2 topic hz /Laser_map
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_footprint
```

如果需要保存 PCD 地图，把配置里的：

```yaml
pcd_save:
  pcd_save_en: true
  interval: -1
```

建图结束后用 `Ctrl+C` 正常退出，PointLIO 会在包内 `PCD/` 目录保存点云。保存前确认目录存在：

```bash
mkdir -p /home/dtc/robo_sys/src/pointlio/PCD
```

### 阶段 3：生成 2D 栅格地图

当前项目已有两种生成 2D 栅格地图的方式。

离线 PCD 转 Nav2 地图：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_navigation pcd_to_grid_map.py \
  /home/dtc/robo_sys/src/pointlio/PCD/scans.pcd \
  --output-prefix /home/dtc/robo_sys/src/my_robot_navigation/maps/real_site_map \
  --resolution 0.05 \
  --min-z 0.05 \
  --max-z 1.50 \
  --inflate-radius 0.15
```

实时点云投影节点：

```text
src/my_robot_navigation/scripts/pointcloud_to_occupancy_grid.py
```

关键参数：

| 参数 | 含义 | 初始建议 |
|---|---|---|
| `resolution` | 栅格分辨率 | `0.05` 或 `0.10` m |
| `width_m` / `height_m` | 地图范围 | 覆盖场地并留边界 |
| `origin_x` / `origin_y` | 地图左下角坐标 | 通常为 `-width_m/2`、`-height_m/2` |
| `min_z` | 低于该高度的点不作为障碍 | 约 `0.05` 到 `0.15` m |
| `max_z` | 高于该高度的点不作为障碍 | 约机器人可碰撞高度，如 `1.2` 到 `1.8` m |
| `inflate_radius_m` | 投影时障碍物预膨胀 | 约 `0.10` 到 `0.30` m |

`min_z` 和 `max_z` 的选择和车体、雷达安装高度有关：

```text
min_z: 去掉地面噪声和低矮毛刺
max_z: 保留机器人会撞到的障碍，忽略天花板、横梁上方点
```

如果地面点很多导致地图被涂黑，先增大 `min_z`。如果桌面、柜子等障碍没有进入地图，降低 `max_z` 或检查点云坐标系 z 方向。

实时 `/Laser_map -> /map` 时，也可以再用 map_saver 保存当前 2D 地图：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/dtc/robo_sys/src/my_robot_navigation/maps/real_site_map
```

会生成：

```text
src/my_robot_navigation/maps/real_site_map.yaml
src/my_robot_navigation/maps/real_site_map.pgm
```

## 实时点云避障

PointLIO 会发布两类点云：

```text
/Laser_map          累计地图点云，适合投影成全局 /map
/cloud_registered   当前帧配准到 map 坐标系后的点云，适合 local costmap 实时标记障碍
```

项目中新增了 `nav2_pointcloud_params.yaml`。`pointlio_nav_bag.launch.py` 默认使用该参数文件：

```text
local_costmap:
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  obstacle_layer:
    observation_sources: registered_cloud
    registered_cloud:
      topic: /cloud_registered
      data_type: PointCloud2
      marking: true
      clearing: false
      min_obstacle_height: 0.05
      max_obstacle_height: 1.50
      obstacle_max_range: 4.0
      observation_persistence: 0.3
```

说明：

- `/cloud_registered` 用于局部实时障碍标记，减少动态障碍对全局地图的污染。
- 当前先只做 marking，不做 clearing；移动障碍主要依赖较短 `observation_persistence` 自然过期。
- `/Laser_map` 仍用于构建或更新全局 `/map`。
- 上车前要根据雷达高度、车体高度和地面噪声调 `min_obstacle_height`、`max_obstacle_height`、`obstacle_max_range`。

## 地图人工修整

真实建图很容易出现地面噪声、玻璃漏检、动态人影、货架边缘毛刺。建议允许人工修整，尤其是最终用于 Nav2 的 2D 栅格地图。

### 推荐工具

| 工具 | 用途 | 适用阶段 |
|---|---|---|
| RViz2 | 查看 TF、点云、2D map、路径 | 全流程 |
| CloudCompare | 裁剪、旋转、清理 3D PCD 点云 | 3D 地图修整 |
| PCL 工具 | 体素降采样、离群点过滤 | 3D 地图批处理 |
| Open3D Python | 自动化点云裁剪、滤波、投影 | 3D 到 2D 处理 |
| GIMP / Krita | 直接编辑 `.pgm` 栅格图 | 2D 地图修整 |
| ImageMagick | 批量二值化、膨胀、腐蚀 | 2D 地图批处理 |
| `semantic_mask_editor.py` | 在真实 2D 投影图上绘制红/绿语义 mask | 禁行区、通行区标注 |

### 语义 mask 标注工具

项目中提供了一个初版可视化标注工具：

```text
src/my_robot_navigation/scripts/semantic_mask_editor.py
```

用途：

```text
真实 2D 投影图
        ↓
红色半透明 mask: 禁止通过区域
绿色半透明 mask: 可以通行区域
        ↓
导出 overlay、语义 mask、Nav2 风格 PGM/YAML
```

安装依赖：

```bash
sudo apt install python3-opencv python3-yaml
```

直接运行：

```bash
cd /home/dtc/robo_sys
python3 src/my_robot_navigation/scripts/semantic_mask_editor.py \
  --image /path/to/real_projection.png \
  --output-prefix /home/dtc/robo_sys/maps/site_mask \
  --resolution 0.05 \
  --origin -10.0,-10.0,0.0
```

如果已有 Nav2 地图 YAML，可以复用地图元数据，保证输出的栅格分布一致：

```bash
python3 src/my_robot_navigation/scripts/semantic_mask_editor.py \
  --image /path/to/real_projection.png \
  --map-yaml /home/dtc/robo_sys/src/my_robot_navigation/maps/real_site_map.yaml \
  --output-prefix /home/dtc/robo_sys/src/my_robot_navigation/maps/real_site_semantic
```

编译安装后也可以这样运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_robot_navigation semantic_mask_editor.py \
  --image /path/to/real_projection.png \
  --map-yaml src/my_robot_navigation/maps/real_site_map.yaml
```

交互方式：

```text
左键拖动: 使用当前模式绘制
右键拖动: 擦除
鼠标滚轮: 调整笔刷大小
r / 1: 红色禁行区
g / 2: 绿色通行区
e / 0: 擦除模式
[ / ]: 缩小 / 放大笔刷
- / =: 降低 / 提高透明度
u: 撤销
c: 清空 mask
s: 保存导出
h: 显示帮助
q / Esc: 退出
```

导出文件：

```text
*_overlay.png       # 底图 + 半透明红绿 mask，便于人工检查
*_mask_color.png    # 红绿彩色 mask
*_mask_index.png    # 机器可读 mask，0=未标注，1=禁行，2=通行
*_nav2.pgm          # Nav2 风格栅格，红=障碍，绿=可通行，未标注=未知
*_nav2.yaml         # 与 *_nav2.pgm 配套的地图元数据
```

默认导出中，未标注区域是未知灰色。如果希望未标注区域作为可通行区域：

```bash
python3 src/my_robot_navigation/scripts/semantic_mask_editor.py \
  --image /path/to/real_projection.png \
  --unlabeled-as-free
```

如果希望未标注区域全部作为禁止区域：

```bash
python3 src/my_robot_navigation/scripts/semantic_mask_editor.py \
  --image /path/to/real_projection.png \
  --unlabeled-as-occupied
```

### 3D 点云修整建议

先处理 PCD，再投影到 2D：

1. 删除地面以下离群点。
2. 删除天花板、灯架、雷达不会撞到的高处结构。
3. 删除建图过程中的人、临时箱子、车等动态物体。
4. 对墙面和固定障碍保守保留，不要过度删减。
5. 对玻璃、黑色吸光物体等漏检位置，可以后续在 2D 地图中手工补障碍。

### 2D 栅格修整规则

`.pgm` 地图常见含义：

```text
黑色: 障碍物
白色: 可通行区域
灰色: 未知区域
```

修整时建议：

- 把不可通行区域涂成障碍。
- 把机器人不能进入的房间、楼梯、危险区涂成障碍。
- 窄门和窄通道不要画得过窄，至少保留机器人宽度 + 两侧安全余量。
- 玻璃墙、镜面、低矮障碍如果点云漏掉，要人工补黑。
- 动态物体造成的小黑点可以擦除。
- 地图边界外建议保持未知或障碍，不要全部涂成可通行。

修完后重新加载：

```bash
ros2 run nav2_map_server map_server \
  --ros-args \
  -p yaml_filename:=/home/dtc/robo_sys/src/my_robot_navigation/maps/real_site_map.yaml
```

## 规划验证

地图可用于规划前，至少验证这些内容：

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic echo /plan --once
```

RViz 中检查：

1. Fixed Frame 设置为 `map`。
2. Map 能正常显示。
3. RobotModel 与地图对齐。
4. 点 Nav2 Goal 后，`Global Path` 不穿墙、不穿障碍。
5. 目标点靠近障碍物时，路径会绕行。
6. 窄通道路径居中，不贴边。

如果路径穿墙：

- 检查 2D 地图中墙体是否连续。
- 增大 `inflate_radius_m` 或 Nav2 costmap 的 inflation 半径。
- 检查地图阈值，确认障碍没有被当成 free。

如果路径绕不过去：

- 检查通道是否被地图噪声堵住。
- 擦除动态障碍残影。
- 降低地图投影膨胀半径。
- 检查机器人 footprint 是否过大。

## 上车闭环前检查清单

- 雷达固定牢靠，外参记录完成。
- 电脑与雷达网络固定 IP 已配置。
- `/livox/lidar` 和 `/livox/imu` 频率稳定。
- PointLIO 静止初始化正常。
- `/Laser_map` 无明显重影。
- `map -> base_footprint` TF 连续。
- 2D 地图已人工修整并保存。
- Nav2 能在 RViz 中生成不穿障碍的 `/plan`。
- `/cmd_vel` 到底盘控制器的方向和单位已验证。
- 急停或手动接管可用。

## 建议的实施顺序

1. 调通真实 Livox 驱动，只看 `/livox/lidar` 和 `/livox/imu`。
2. 录制 1 到 2 分钟静态和低速移动 bag。
3. 用 bag 离线跑 PointLIO，调 `pointlio_mid360_nav.yaml`。
4. 保存 PCD，使用 CloudCompare/Open3D 清理 3D 地图。
5. 投影生成 2D grid，保存为 `real_site_map.yaml` 和 `real_site_map.pgm`。
6. 用 GIMP/Krita 修整 2D 栅格地图。
7. 用 Nav2 在 RViz 中只验证规划，不接底盘运动。
8. 接入底盘 `/cmd_vel`，低速小范围闭环测试。
9. 固化参数和地图，提交到仓库。

## 后续可补充的工具脚本

后续建议在项目中增加这些脚本：

```text
scripts/filter_pcd_for_navigation.py      # PCD 裁剪、降采样、去离群点
scripts/pcd_to_grid_map.py                # 已实现：离线 PCD 转 map.yaml + map.pgm
scripts/edit_grid_map.py                  # 批量膨胀/腐蚀/清理小噪声
launch/real_lidar_mapping.launch.py       # 真实雷达驱动 + PointLIO + RViz
launch/real_lidar_nav.launch.py           # 静态修整地图 + 定位 + Nav2
```

第一阶段可以先不写复杂工具，使用 CloudCompare 和 GIMP 手动修整；等地图流程稳定后，再把重复操作脚本化。

## 相机图像投影到空间平面

项目中已有初版 RViz 可视化节点：

```text
src/my_robot_navigation/scripts/image_to_plane_marker.py
src/my_robot_navigation/launch/image_projection.launch.py
```

它订阅普通相机图像，把图像作为纹理贴到 RViz 中的一个平面 Marker 上：

```text
/camera/image_raw
        ↓
image_to_plane_marker.py
        ↓
/camera_projection_marker visualization_msgs/Marker
        ↓
RViz Marker 显示
```

启动示例：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_navigation image_projection.launch.py \
  image_topic:=/camera/image_raw \
  plane_frame:=map \
  plane_width:=4.0 \
  plane_height:=3.0 \
  position_x:=0.0 \
  position_y:=0.0 \
  position_z:=0.02 \
  yaw:=0.0
```

关键参数：

| 参数 | 含义 |
|---|---|
| `image_topic` | 输入图像话题 |
| `marker_topic` | 输出 RViz Marker 话题，默认 `/camera_projection_marker` |
| `plane_frame` | 投影平面所在坐标系，常用 `map` |
| `plane_width` / `plane_height` | 平面实际尺寸，单位 m |
| `position_x/y/z` | 平面中心在 `plane_frame` 下的位置 |
| `roll/pitch/yaw` | 平面姿态，单位 rad |
| `alpha` | 图像平面透明度 |

当前版本做的是“图像到矩形平面”的初步投影，不依赖点云，也不把点云改成彩色点云。后续标定相机外参后，可以把相机到车体、车体到地图、平面位置和尺度串起来，替换现在的手工平面参数。
