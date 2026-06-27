# MID360 连接失败排查记录

## 问题现象

现场现象是:

- 已经能查到雷达 IP: `192.168.1.173`
- 主机网卡 IP 为: `192.168.1.5`
- 但启动 `livox_ros_driver2` 后，表现为“连接不上雷达”或没有正常出点云

## 排查环境

- 工作区: `/home/nvidia/rslidar_ws`
- 驱动包: `src/livox_ros_driver2`
- 主机网卡: `eno1`
- 主机 IP: `192.168.1.5/24`
- 雷达 IP: `192.168.1.173`

确认网卡状态时，`eno1` 已经在正确网段:

```bash
ip -brief addr
```

结果中可见:

```text
eno1             UP             192.168.1.5/24
```

这说明主机与雷达处于同一网段，网络配置本身不是首要问题。

## 初始错误

直接启动:

```bash
source /home/nvidia/rslidar_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

节点报错:

```text
liblivox_lidar_sdk_shared.so: cannot open shared object file: No such file or directory
```

这说明问题并不在雷达连通性本身，而是驱动进程在启动阶段就因为动态库找不到而退出了，尚未真正进入和雷达通信的步骤。

## 根因分析

根因有两部分。

### 1. 运行时动态库路径未配置

驱动运行依赖:

```text
/usr/local/lib/liblivox_lidar_sdk_shared.so
```

虽然该库文件已经安装在 `/usr/local/lib` 下，但当前终端环境的 `LD_LIBRARY_PATH` 未包含该目录，导致 `ros2 launch` 时 `dlopen` 失败。

### 2. MID360 多设备配置文件格式不够规范

原始 `MID360s_config.json` 中存在两个可疑点:

- 顶层键名使用了 `"Mid360s"`
- `host_net_info` 中未显式指定 `lidar_ip`

结合仓库 `README.md` 中的 MID360 配置示例，较稳妥的格式应当:

- 使用 `"MID360"` 作为设备配置键名
- 在 `host_net_info` 中加入与该网卡绑定的 `lidar_ip` 列表

这不是最先导致程序退出的原因，但属于后续可能导致“能看到 IP、但驱动配置未被 SDK 正确识别”的隐患，因此一并修正。

## 修复内容

### 1. 为当前终端补充动态库路径

启动前执行:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
source /home/nvidia/rslidar_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

如果希望永久生效，可加入 `~/.bashrc`:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

然后执行:

```bash
source ~/.bashrc
```

### 2. 修正配置文件

修正后的配置文件为 [MID360s_config.json](/home/nvidia/rslidar_ws/src/livox_ros_driver2/config/MID360s_config.json)。

关键修正如下:

```json
{
  "lidar_summary_info" : {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info" : {
      "cmd_data_port"  : 56100,
      "push_msg_port"  : 56200,
      "point_data_port": 56300,
      "imu_data_port"  : 56400,
      "log_data_port"  : 56500
    },
    "host_net_info" : [
      {
        "lidar_ip"       : ["192.168.1.173"],
        "host_ip"        : "192.168.1.5",
        "cmd_data_port"  : 56101,
        "push_msg_port"  : 56201,
        "point_data_port": 56301,
        "imu_data_port"  : 56401,
        "log_data_port"  : 56501
      }
    ]
  }
}
```

## 修复后验证结果

重新启动后，驱动已经成功初始化，并能完成和雷达的握手配置。日志中出现了以下关键信息:

```text
Init lds lidar success!
successfully set lidar attitude, ip: 192.168.1.173
successfully change work mode
successfully enable Livox Lidar imu, ip: 192.168.1.173
```

同时 ROS 话题已经正常出现:

```bash
ros2 topic list
```

结果包含:

```text
/livox/imu
/livox/lidar
```

此外，UDP 监听端口也已成功建立:

```text
192.168.1.5:56101
192.168.1.5:56201
192.168.1.5:56301
192.168.1.5:56401
```

这说明驱动进程已经正常运行，且已经进入数据通信阶段。

## 最终结论

本次“连接不上雷达”的直接原因不是雷达 IP 错误，也不是主机网段错误，而是:

1. `liblivox_lidar_sdk_shared.so` 未被运行环境正确加载
2. `MID360s_config.json` 的格式存在兼容性隐患

在补充 `LD_LIBRARY_PATH` 并修正配置文件后，`livox_ros_driver2` 已可正常启动，并成功连接到 `192.168.1.173` 这台 MID360。

## 后续建议

建议将以下内容固化到环境初始化流程中:

1. 在 `~/.bashrc` 中加入:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

2. 启动前统一执行:

```bash
source /home/nvidia/rslidar_ws/install/setup.bash
```

3. 若后续出现“驱动已启动但 RViz 无点云”，优先检查:

- RViz 的 `Fixed Frame` 是否为 `livox_frame`
- 当前显示订阅的是否是 `/livox/lidar`
- 点云消息格式是否与当前 launch 文件配置一致

