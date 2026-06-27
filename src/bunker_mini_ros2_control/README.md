# BUNKER MINI ROS2 底盘控制示例

这个包用于 ROS2 Humble 下接收规划节点发布的 `/cmd_vel`，并控制 BUNKER MINI。支持两种方式：

- `can_driver`：真实底盘驱动，直接把 `/cmd_vel` 转成 SocketCAN 报文发到 `can0`。
- `cmd_vel_bridge`：仅做 ROS 话题桥接，把 `/cmd_vel` 转发到 `/smoother_cmd_vel`。

它不直接发送 CAN 报文。真实底盘仍需要先完成 CAN 接线、`can0` 配置和底盘驱动节点启动。根据指南，常见底盘速度话题是 `/smoother_cmd_vel`，如果你的 ROS2 驱动订阅 `/cmd_vel`，启动时改参数即可。

## 安全检查

1. 第一次测试请把车架起来，或放在空旷区域。
2. 确认急停 `Q1` 可随时按下。
3. 确认 `SWB` 位于 CAN/指令控制模式。
4. 先用 `candump can0` 确认能收到 `0x211` 等底盘反馈。
5. 低速测试，默认线速度只有 `0.05 m/s`。

## CAN 口配置

```bash
sudo ip link set can0 up type can bitrate 500000
ip -details -statistics link show can0
candump can0
```

如果是串口 CAN：

```bash
sudo slcand -o -c -s6 /dev/ttyACM0 can0
sudo ip link set can0 up
```

## 编译

把 `bunker_mini_ros2_control` 放进 ROS2 工作空间：

```bash
mkdir -p ~/ros2_ws/src
cp -r bunker_mini_ros2_control ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select bunker_mini_ros2_control
source install/setup.bash
```

## 真实 BUNKER MINI CAN 控制

手册协议：

- `0x421`：控制模式设定帧，`byte0 = 0x01` 表示 CAN 指令模式。
- `0x111`：运动控制帧，20 ms 发送一次。
- `byte0..1`：线速度，`signed int16`，单位 `mm/s`，Motorola/大端。
- `byte2..3`：角速度，`signed int16`，单位 `0.001 rad/s`，Motorola/大端。
- `byte4..7`：保留 `0x00`。

先配置 CAN：

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
candump can0
```

能看到 `0x211`、`0x221` 等反馈后，启动真实驱动：

```bash
ros2 launch bunker_mini_ros2_control can_driver.launch.py \
  can_interface:=can0 \
  cmd_vel_topic:=/cmd_vel \
  max_linear_x:=0.10 \
  max_angular_z:=0.20
```

首次测试务必架空或放在空旷区域，急停放手边：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.03, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

停车：

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## 手动 CAN 帧测试

切 CAN 指令模式：

```bash
cansend can0 421#01
```

停止：

```bash
cansend can0 111#0000000000000000
```

低速前进 `0.03 m/s`，也就是 `30 mm/s = 0x001E`：

```bash
cansend can0 111#001E000000000000
```

低速左转 `0.10 rad/s`，也就是 `100 mrad/s = 0x0064`：

```bash
cansend can0 111#0000006400000000
```

## ROS 话题桥接

你的截图里 `/cmd_vel` 已经有规划节点发布的速度，例如：

```text
linear.x: 0.063
angular.z: -0.126
```

启动桥接节点后，它会订阅 `/cmd_vel`，做限速和超时停车保护，再转发到底盘驱动话题：

```bash
ros2 launch bunker_mini_ros2_control cmd_vel_bridge.launch.py
```

默认输出到 `/smoother_cmd_vel`。如果你的 ROS2 底盘驱动直接订阅 `/cmd_vel`，不要让桥接节点也输出 `/cmd_vel`，否则会形成同名回环。更推荐让规划节点输出 `/cmd_vel`，底盘驱动订阅 `/smoother_cmd_vel`：

```bash
ros2 launch bunker_mini_ros2_control cmd_vel_bridge.launch.py \
  input_topic:=/cmd_vel \
  output_topic:=/smoother_cmd_vel
```

如果底盘方向相反，可以用参数反向：

```bash
ros2 run bunker_mini_ros2_control cmd_vel_bridge --ros-args \
  -p input_topic:=/cmd_vel \
  -p output_topic:=/smoother_cmd_vel \
  -p invert_linear_x:=true
```

现场初调建议把限速调低：

```bash
ros2 launch bunker_mini_ros2_control cmd_vel_bridge.launch.py \
  max_linear_x:=0.10 \
  max_angular_z:=0.20
```

检查连接：

```bash
ros2 topic info /cmd_vel
ros2 topic echo /smoother_cmd_vel
```

`/cmd_vel` 应该有规划节点作为 Publisher，桥接节点作为 Subscriber；`/smoother_cmd_vel` 应该有桥接节点作为 Publisher，底盘驱动作为 Subscriber。

## 启动测试动作

默认发布到 `/smoother_cmd_vel`：

```bash
ros2 launch bunker_mini_ros2_control bunker_mini_cmd.launch.py
```

如果你的底盘驱动订阅 `/cmd_vel`：

```bash
ros2 launch bunker_mini_ros2_control bunker_mini_cmd.launch.py cmd_vel_topic:=/cmd_vel
```

循环演示：

```bash
ros2 launch bunker_mini_ros2_control bunker_mini_cmd.launch.py repeat_demo:=true
```

更慢速度：

```bash
ros2 launch bunker_mini_ros2_control bunker_mini_cmd.launch.py linear_speed:=0.03 angular_speed:=0.10
```

## 节点行为

节点会按以下顺序发布速度：

1. 低速前进 2 秒。
2. 停车 1 秒。
3. 原地左转 2 秒。
4. 停车并持续发布零速度。

退出节点时也会连续发布几帧零速度，避免底盘保持上一条运动指令。
