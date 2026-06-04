# Nav2 后续替代方向说明

本文档说明当前仿真导航系统后续可以逐步替换的模块，以及建议的实现顺序。目标是先保持系统可运行，再逐个替换路径搜索、轨迹跟踪、定位和地图生成模块。

## 1. 当前系统状态

当前版本采用标准 Nav2 组件和一个简单全向底盘模拟节点：

```text
静态 2D 栅格地图 /map
        ↓
NavFnPlanner 全局路径规划
        ↓
/plan
        ↓
DWBLocalPlanner 局部轨迹跟踪
        ↓
/cmd_vel
        ↓
simple_omni_base_node 全向底盘模拟
        ↓
/odom + odom -> base_footprint
```

当前定位链路是简化版：

```text
map -> odom              静态 TF
odom -> base_footprint   simple_omni_base_node 发布
```

当前阶段不接 SLAM、不接 AMCL、不接雷达模拟。地图使用保存好的简单 2D 栅格地图。

## 2. 替代原则

后续不要一次性重写全部导航系统。推荐遵循以下原则：

```text
1. 每次只替换一个模块
2. 替换模块前先明确输入和输出
3. 新模块先独立发布调试话题
4. 调试稳定后再接入 Nav2 plugin 或正式接口
5. 保留标准 Nav2 组件作为对照组
```

这样可以快速判断问题来自算法、地图、TF、参数还是底盘接口。

## 3. 可替换模块总览

| 模块 | 当前实现 | 后续替代方向 | 建议优先级 |
|---|---|---|---|
| 全局路径规划 | `NavFnPlanner` | A*、Dijkstra、Theta*、Hybrid A* | 中 |
| 局部轨迹跟踪 | `DWBLocalPlanner` | Pure Pursuit、Stanley、MPC | 中 |
| 定位 | 静态 `map -> odom` | AMCL、slam_toolbox localization、自研 SLAM 定位 | 高 |
| 建图 | 手工 2D 栅格地图 | 2D SLAM、3D SLAM 转 2D 栅格 | 高 |
| 底盘接口 | `simple_omni_base_node` | 真实底盘驱动、真实里程计 | 高 |
| 避障感知 | 静态地图 costmap | 雷达、深度相机、3D 点云投影 | 中 |

## 4. 全局路径规划替代

当前全局规划器：

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
```

当前输入：

```text
1. /map 或 global_costmap
2. 当前机器人位姿
3. 目标点
```

当前输出：

```text
/plan，类型 nav_msgs/Path
```

后续自研路径搜索可以按以下顺序做：

```text
第一步：写独立 A* 节点
输入 /map、起点、终点
输出 /plan_debug

第二步：在 RViz 中显示 /plan_debug
确认路径能绕开障碍物

第三步：加入路径平滑
减少折线和贴边路径

第四步：封装为 Nav2 planner plugin
替换 NavFnPlanner
```

推荐先实现 A*，再考虑 Theta* 或 Hybrid A*。当前全向底盘对非完整约束要求不高，所以一开始不需要 Hybrid A*。

## 5. 局部轨迹跟踪替代

当前轨迹跟踪器：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
```

当前输入：

```text
1. /plan
2. 当前机器人位姿 TF
3. /odom
4. local_costmap
```

当前输出：

```text
/cmd_vel，类型 geometry_msgs/Twist
```

后续自研轨迹跟踪可以按以下顺序做：

```text
第一步：写独立 Pure Pursuit 节点
订阅 /plan 和 TF
发布 /cmd_vel_debug 或 /cmd_vel

第二步：用 simple_omni_base_node 验证跟踪效果
观察机器人是否能沿路径移动

第三步：加入速度限制、目标点减速、角速度控制

第四步：加入障碍物距离约束

第五步：封装为 Nav2 controller plugin
替换 DWBLocalPlanner
```

全向底盘的简单跟踪器可以输出：

```text
linear.x
linear.y
angular.z
```

如果后续换成差速底盘，则需要限制 `linear.y = 0`，并重新设计转向控制。

## 6. 定位替代

当前定位是简化实现：

```text
map -> odom 静态不变
odom -> base_footprint 来自简单底盘积分
```

这只适合早期仿真验证。后续接入定位或 SLAM 后，需要由定位算法实时发布：

```text
map -> odom
```

Nav2 对定位模块的核心要求：

```text
1. TF 中存在 map -> odom -> base_footprint
2. /odom 连续稳定
3. map -> odom 能修正累计里程计漂移
4. 时间戳一致
```

推荐替代顺序：

```text
第一步：保持当前 /odom 不变
第二步：接入 AMCL 或 slam_toolbox localization
第三步：关闭静态 map -> odom
第四步：由定位算法发布 map -> odom
第五步：确认 RViz 中机器人位姿稳定
第六步：再接入自研 SLAM 定位模块
```

如果已有 SLAM 算法不是 ROS2 节点，需要写 wrapper，将算法输出转换为：

```text
1. TF: map -> odom
2. 可选调试位姿: geometry_msgs/PoseStamped
3. 可选地图: nav_msgs/OccupancyGrid
```

## 7. 建图和 3D 地图转 2D 栅格

标准 2D Nav2 最终需要 2D 地图：

```text
nav_msgs/OccupancyGrid
```

如果已有算法生成 3D 地图，例如点云、体素地图、OctoMap、TSDF 或 ESDF，需要转换成 2D 栅格地图供 Nav2 使用。

推荐转换流程：

```text
3D 点云或体素地图
        ↓
坐标变换到 map 坐标系
        ↓
高度过滤
        ↓
地面分割
        ↓
投影到 XY 平面
        ↓
膨胀处理
        ↓
生成 nav_msgs/OccupancyGrid
        ↓
发布 /map 或保存为 .pgm + .yaml
```

高度过滤示例：

```text
z < 0.05 m       认为接近地面，通常不作为障碍物
0.05 m ~ 1.20 m 认为可能影响移动机器人通行
z > 1.20 m       可根据机器人高度忽略
```

具体阈值需要根据机器人底盘高度、传感器高度和环境调整。

如果 3D SLAM 只用于建图，可以离线保存 2D 地图：

```text
3D 建图完成
        ↓
投影生成 2D OccupancyGrid
        ↓
保存 simple_map.pgm / simple_map.yaml
        ↓
Nav2 map_server 加载静态地图
```

如果 3D SLAM 在线运行，则可以直接发布：

```text
/map
map -> odom
```

Nav2 继续使用 `/map` 和 TF 工作。

## 8. 底盘接口替代

当前底盘模拟节点：

```text
simple_omni_base_node
```

订阅：

```text
/cmd_vel
```

发布：

```text
/odom
odom -> base_footprint
```

后续接真实底盘时，真实底盘驱动也应尽量保持相同接口：

```text
输入 /cmd_vel
输出 /odom
输出 odom -> base_footprint
```

这样 Nav2 上层不需要大改。

替代顺序：

```text
第一步：保留 Nav2 和地图不变
第二步：停用 simple_omni_base_node
第三步：启动真实底盘驱动
第四步：确认 /cmd_vel 能驱动底盘
第五步：确认 /odom 和 TF 正常
第六步：再调 Nav2 速度和加速度参数
```

## 9. 推荐阶段计划

### 阶段一：当前 2D 全向仿真跑稳

目标：

```text
1. RViz 显示 /map
2. RViz 显示 /plan
3. Nav2 Goal 后机器人能到达目标
4. /odom 和 TF 稳定
```

### 阶段二：接入真实或仿真定位

目标：

```text
1. 用 AMCL 或 SLAM localization 替换静态 map -> odom
2. 保持 Nav2 能正常规划和控制
3. 定位丢失时能在 RViz 中快速发现
```

### 阶段三：接入建图

目标：

```text
1. 2D SLAM 或 3D SLAM 能生成地图
2. 地图能转换为 nav_msgs/OccupancyGrid
3. 地图可保存为 .pgm + .yaml
4. Nav2 能加载生成的地图导航
```

### 阶段四：自研全局路径搜索

目标：

```text
1. A* 节点能基于 /map 输出 /plan_debug
2. RViz 能显示自研路径
3. 路径质量与 NavFnPlanner 对比
4. 封装成 Nav2 planner plugin
```

### 阶段五：自研轨迹跟踪

目标：

```text
1. Pure Pursuit 或 Stanley 能跟踪 /plan
2. 输出 /cmd_vel 控制当前全向底盘
3. 能处理目标减速和终点朝向
4. 封装成 Nav2 controller plugin
```

### 阶段六：真实底盘和真实传感器

目标：

```text
1. 真实底盘驱动接入 /cmd_vel 和 /odom
2. 真实传感器接入 SLAM 或 costmap
3. 根据实车速度、加速度、半径重新调参
4. 保持上层 Nav2 接口不变
```

## 10. 建议优先做的三件事

短期建议优先推进：

```text
1. 把当前 2D 全向仿真跑稳定
2. 加入定位替代方案，先用 AMCL 或 slam_toolbox localization 验证 map -> odom
3. 实现一个独立 A* 节点，先只发布 /plan_debug，不急着替换 Nav2 planner
```

这样既能保持项目持续可运行，也能逐步积累自研算法模块。
