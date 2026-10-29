# robot_lidar

YDLIDAR X2 2D 激光雷达驱动（M4）。读取 X2 串口数据，按一圈累计后发布：

| 话题 | 类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | BEST_EFFORT | 一圈扫描，给 SLAM Toolbox / Nav2 |
| `/lidar/points` | `sensor_msgs/PointCloud2` | BEST_EFFORT | 同一圈的点云，RViz 直接看 |

节点不发布 TF。`frame_id` 默认 `laser_link`，URDF 再加入 `base_link → laser_link` 的静态变换。

## 协议要点

已对照 `docs/datasheet/LiDAR/` 与商家 YDLIDAR SDK（`ydlidar_ros2/sdk/src/ydlidar_driver.cpp`）。

| 项 | 值 |
| --- | --- |
| 串口 | 115200 8N1，3.3 V TTL，无流控 |
| 帧头 | `AA 55`（`PH=0x55AA` 低字节在前；手册图里的 `A5 5A` 是笔误） |
| 包结构 | `AA 55 | CT | LSN | FSA(2) | LSA(2) | CS(2) | S1(2)..Sn(2)` |
| 包长 | `10 + 2×LSN` |
| 距离 | `Si / 4` mm |
| 一级角度 | `(raw >> 1) / 64` 度；raw 的 bit0 是固定校验位 C=1 |
| CS | 16 位**字**异或：`PH ^ (CT | LSN<<8) ^ FSA ^ LSA ^ 各采样字`，不含 CS 自身 |
| CT bit0 | 0=点云包；1=一圈起始包，`F = CT[7:1]/10` Hz |
| 二级角度修正 | `atan(21.8×(155.3−d)/(155.3×d))`，d 单位 mm；示例 d=1000 → −6.7622° |
| 电机 | USB 转接板用 **DTR** 控制；上电待机，DTR 拉高才转 |
| 角度方向 | 手册定义顺时针为正；ROS REP-103 逆时针为正 |

跨圈包（`LSA < FSA` 且 `FSA > 270°`、`LSA < 90°`）按 `+360°` 计算间隔；其他 `LSA < FSA` 的异常包沿用上一包间隔（与商家 SDK 一致）。

## 参数

全部在 `robot_bringup/config/lidar.yaml`：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `port` | `/dev/robot_lidar` | udev 固定名；也可改成 `/dev/ttyUSB0` 等 |
| `baud` | `115200` | |
| `frame_id` | `laser_link` | |
| `angle_offset_deg` | `0.0` | 安装偏航角，CCW 为正 |
| `invert_angle` | `true` | 把 X2 顺时针角转成 ROS 逆时针；设 `false` 与商家 X2.yaml 一致 |
| `range_min_m` / `range_max_m` | `0.12` / `8.0` | 越界点丢弃 |
| `scan_bins` | `720` | 0.5°/bin；X2 6 Hz 时约 0.72° 分辨率 |
| `publish_scan` | `true` | 是否发 `/scan` |
| `publish_pointcloud` | `true` | 是否发 `/lidar/points` |
| `motor_dtr` | `true` | USB 转接板必须为 true；直连 UART 设为 false |
| `scan_frequency_hz` | `6.0` | 仅用于 `scan_time` 默认值；收到起始包后用实测值覆盖 |
| `timer_period_ms` | `10` | 串口读取周期 |
| `reconnect_period_s` | `2.0` | 打开失败/掉线后的重试间隔 |

## 运行

```bash
# 1. 安装固定设备名（CP2102 转接板）
sudo cp config/udev/99-robot-lidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout "$USER"      # 重新登录后生效

# 2. 构建
cd mobile_manipulator_ws
colcon build --symlink-install
source install/setup.sh

# 3. 启动
ros2 launch robot_bringup lidar.launch.py

# 4. RViz2：Fixed Frame 选 laser_link，Add -> LaserScan /scan
#    或 Add -> PointCloud2 /lidar/points
```

如果转接板不是 CP2102（例如 CH340），udev 规则不会匹配；把 `lidar.yaml` 的 `port` 改成实际设备即可。当前 PC 测试用的就是 CH340 转接板，直接用了 `/dev/ttyUSB0`。

上车时如果雷达也是 CH340，会和 STM32 主控同 VID/PID，不能只靠 `99-robot-stm32.rules` 区分；需要按序列号或物理 USB 口各加一条规则，示例见 `config/udev/99-robot-lidar.rules`。

## 测试

```bash
colcon test --paths src/robot_lidar
colcon test-result --verbose
```

| 套件 | 用例 | 覆盖 |
| --- | --- | --- |
| `test_x2_protocol` | 15 | 16 位字校验、帧头、角度修正（手册两个示例）、跨圈、上一包间隔、C 位、坏包重同步、任意分块、最大 LSN |

**2026-10-29 PC 实机测试**：X2 经 CH340 接 `/dev/ttyUSB0`，115200，`motor_dtr=true`，电池/环境条件未记录。

| 项 | 结果 |
| --- | --- |
| 扫描频率 | 8 s 收到 49 帧 = **6.11 Hz**，与起始包间隔 0.164 s 一致 |
| LaserScan | 720 bin，312 个有限点，范围 0.164~4.077 m，`scan_time=0.1667 s` |
| 扇区覆盖 | 12 个 30° 扇区全部有回波 |
| PointCloud2 | 约 304 点，字段 `x/y/z` |
| 退出 | SIGINT 退出码 0，DTR 释放 |

同一轮还用 PTY 模拟过 6 圈数据（360°、40 点/包）：`/scan` 720 bin / 360 个有限点，`/lidar/points` 360 点，退出码 0。

**测试中发现并修复**：X2 每圈在约 338° 发一个起始包，起始包之后的第一个点包会跨到 7°；原先的角度回绕兜底把这一圈又切了一次，导致发布成 12 Hz、每帧只剩一小段。现在以起始包为分圈依据，回绕兜底只在完全没有起始包时启用。

**尚未确认**：角度方向与 `angle_offset_deg` 需要和实际摆放对照（RViz 里看左右是否与现场一致）；DTR 极性、真实角度方向已在实机上验证能转、能出数，但安装方向未标定。

## 已知限制

- 没有做强度/干扰位过滤；`Si` 的 LSB 干扰标志只保留在 `Point::interference`，默认不过滤。
- 没有做 `ignore_array`、角度裁剪、单圈补齐（缺失 bin 填 `inf`）。
- 没有控制电机转速；X2 转接板只通过 DTR 开关电机，转速由硬件默认值决定。
- 只发布 2D 数据；没有高度/z 信息。
- 实际扫描频率约 6 Hz（X2 数据手册典型值，实机 6.11 Hz），不是 `software-design.md` 里 `/scan` 的通用 10 Hz。
- 角度方向、`angle_offset_deg` 和安装位姿待现场标定；`invert_angle` 默认按 REP-103 取反，商家 X2.yaml 保持顺时针。
