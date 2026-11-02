# robot_raw_bridge

测试用桥接包：主机把雷达/底盘串口原始字节通过 ROS 2 转发到虚拟机，本包把它还原成标准 ROS 话题，再交给 SLAM。

| 节点 | 输入 | 输出 |
| --- | --- | --- |
| `lidar_raw_bridge` | `std_msgs/msg/UInt8MultiArray` `/lidar/raw`（X2 `AA 55 ...`） | `/scan`（LaserScan）、`/lidar/points`（PointCloud2） |
| `chassis_raw_bridge` | `std_msgs/msg/UInt8MultiArray` `/stm32/raw`（STM32 协议帧 `AA 55 ...`） | `/odom`、TF `odom→base_link`、`/robot_status`、`/system_event` |

两个输入 topic 都可用参数改：

```bash
ros2 run robot_raw_bridge lidar_raw_bridge --ros-args \
  -p input_topic:=/lidar/raw
ros2 run robot_raw_bridge chassis_raw_bridge --ros-args \
  -p input_topic:=/stm32/raw -p wheel_separation_m:=0.165
```

完整建图：

```bash
ros2 launch robot_bringup mapping_external.launch.py \
  lidar_raw_topic:=/lidar/raw \
  chassis_raw_topic:=/stm32/raw \
  rviz:=true
```

本包**不打开本机串口**，只订阅主机转发的原始字节；串口所有权在主机端。

注意：

- 主机 publisher 用 RELIABLE，本包 subscriber 用 BEST_EFFORT；DDS 兼容，已验证。
- 底盘帧按 `protocol.md` v1 解析，只处理 `0xC1 ODOM`、`0xC3 STATUS`、`0xC4 EVENT`。
- 如果主机 topic 名不是 `/stm32/raw`，用 `chassis_raw_topic` 覆盖；消息类型不同则不能直接复用，需要改订阅类型。

## 远程遥控帧发布

`cmd_raw_publisher` 在虚拟机里读键盘，发布**完整 `SET_VELOCITY` 帧**到 `/stm32/cmd_raw`（`std_msgs/msg/UInt8MultiArray`）。主机订阅这个 topic，把每个 message 的 `data` 原样写入底盘串口即可。

```bash
ros2 run robot_raw_bridge cmd_raw_publisher
# 可改参数：
ros2 run robot_raw_bridge cmd_raw_publisher --ros-args \
  -p output_topic:=/stm32/cmd_raw \
  -p linear_speed_mm_s:=150 \
  -p angular_speed_mrad_s:=500
```

| 按键 | 动作 |
| --- | --- |
| `w` / `s` | 前进 / 后退，150 mm/s |
| `a` / `d` | 原地左转 / 右转，500 mrad/s |
| `x` 或空格 | 停止 |
| `q` | 退出（先发一帧零速） |

- 节点以 **50 Hz** 持续发帧，满足固件 200 ms 断流保护；松手后不会自动停，按 `x` 停。
- 左右是**原地旋转**，不是横移；`SET_VELOCITY` 只有 v/w，没有横向通道。
- 底盘必须先由主机 `SET_ENABLE(1)` 使能；本节点只发速度帧，不发使能/清故障。
- 主机侧订阅示例：收到 `UInt8MultiArray` 后直接把 `msg.data` 写串口，不要重新组帧。
