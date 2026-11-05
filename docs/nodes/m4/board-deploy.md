# M4 板端部署与 PC 端建图操作记录

> 日期：2026-11-05
> 目标：RK3566 只负责底盘和雷达，发 `/odom`、`/scan`、TF；PC 端跑 SLAM/建图。
> 板卡存储 2GB RAM + 8GB eMMC，暂不在板上建图/导航；等 4+32G 版本再迁移。

| 项目 | 内容 |
| --- | --- |
| 板卡 | LubanCat 1，RK3566，aarch64 |
| 板卡系统 | Ubuntu 22.04.5 LTS，kernel 6.1.99-rk356x |
| 板卡网络 | `192.168.0.4`，wlan0 |
| PC | Ubuntu 22.04 虚拟机，`192.168.0.3` |
| ROS 2 | Humble，板卡 arm64，PC amd64 |
| 串口 | 两个 CH340：`/dev/robot_lidar`、`/dev/robot_stm32` |
| 代码 | `develop` + 本次板端部署改动 |

> SSH 账号/密码见 `config/相关数据.md`（该文件不入库，本文不记录密码）。

---

## 1. 板卡网络与 SSH

PC 能 ping 通板卡：

```bash
ping -c 1 192.168.0.4
ssh cat@192.168.0.4
```

板卡两个 CH340 都枚举为 `/dev/ttyUSB0/1`，VID/PID 相同（`1a86:7523`），不能只靠 VID/PID 区分。

---

## 2. 安装 ROS 2 Humble（板端）

板卡初始没有 `/opt/ros`。安装最小 `ros-base` 和本工程依赖，不装 RViz/桌面。

### 2.1 ROS 2 apt key 与源

PC 上已有 Open Robotics key，导出后复制到板卡：

```bash
# PC
sudo gpg --no-default-keyring --keyring /etc/apt/trusted.gpg \
  --export --armor C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 > /tmp/ros2.key
scp /tmp/ros2.key cat@192.168.0.4:/tmp/ros2.key
```

```bash
# 板卡
gpg --dearmor --yes -o /tmp/ros2.gpg /tmp/ros2.key
sudo install -m 0644 /tmp/ros2.gpg /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://mirrors.ustc.edu.cn/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
```

### 2.2 Ubuntu 源

板卡原始源是阿里云 `ubuntu-ports`，但直接全量 `apt update` 会因为 Translation 文件和第三方 embedfire 源非常慢。实际改成只保留需要的组件：

```bash
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
sudo tee /etc/apt/sources.list >/dev/null <<'EOF'
deb http://mirrors.aliyun.com/ubuntu-ports jammy main universe
deb http://mirrors.aliyun.com/ubuntu-ports jammy-updates main universe
deb http://mirrors.aliyun.com/ubuntu-ports jammy-security main universe
EOF
```

更新时关闭翻译文件：

```bash
sudo apt-get update \
  -o Dir::Etc::sourcelist=/etc/apt/sources.list \
  -o Dir::Etc::sourceparts=- \
  -o Acquire::Languages=none \
  -o APT::Get::List-Cleanup=1
```

> **注意**：Ubuntu 更新用 `List-Cleanup=1` 会把 ROS 2 的索引一起清掉。每次 Ubuntu 更新后要单独刷新 ROS 源：
>
> ```bash
> sudo apt-get update \
>   -o Dir::Etc::sourcelist=/etc/apt/sources.list.d/ros2.list \
>   -o Dir::Etc::sourceparts=- \
>   -o Acquire::Languages=none \
>   -o APT::Get::List-Cleanup=0
> ```

### 2.3 安装包

```bash
sudo apt-get install -y --no-install-recommends \
  ros-humble-ros-base \
  ros-humble-tf2 ros-humble-tf2-ros \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-diagnostic-msgs ros-humble-std-srvs \
  ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-geometry-msgs \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  ros-humble-launch ros-humble-launch-ros \
  ros-humble-ament-index-python ros-humble-ament-cmake \
  python3-colcon-common-extensions cmake build-essential
```

结果：192 个 `ros-humble-*` 包，安装后根分区剩余约 4.8G。

### 2.4 环境变量

```bash
cat >> ~/.bashrc <<'EOF'

# ROS 2 board environment
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export LANG=C.UTF-8
EOF
```

PC 端也必须保持 `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=0`。

---

## 3. 串口识别与固定设备名

### 3.1 识别结果

两个 CH340 的 VID/PID 完全一样，按 USB 物理路径区分：

| 设备 | `/dev` | USB 控制器 | 原始数据特征 | 对应 |
| --- | --- | --- | --- | --- |
| CH340 #1 | `/dev/ttyUSB0` | `fd000000.usb` / `xhci-hcd.4.auto` | `aa 55 00 28 ...` | X2 雷达 |
| CH340 #2 | `/dev/ttyUSB1` | `fd8c0000.usb`（busnum 会变） | `aa 55 10 c1 ...` | STM32 底盘 |

> 底盘 CH340 在 USB 重枚举后 busnum 会从 6 变成 3，**不能按 busnum 绑定**；用控制器 `DEVPATH`。

验证方法：用 Python `termios` 打开端口，设置 115200 raw、DTR 高，读 1~2 秒，打印十六进制：

- 雷达：`aa 55 00/50/42 ... 28 ...`（X2 数据包）
- 底盘：`aa 55 10 c1 ...`（`REPORT_ODOM`）、`aa 55 0a c3 ...`（状态）

### 3.2 udev 规则

新增 `config/udev/99-robot-board.rules`：

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
    DEVPATH=="*/fd000000.usb/*", \
    SYMLINK+="robot_lidar", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
    DEVPATH=="*/fd8c0000.usb/*", \
    SYMLINK+="robot_stm32", GROUP="dialout", MODE="0660"
```

安装：

```bash
sudo cp config/udev/99-robot-board.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty
sudo usermod -aG dialout cat
```

验证：

```bash
ls -l /dev/robot_stm32 /dev/robot_lidar
# /dev/robot_lidar -> ttyUSB0
# /dev/robot_stm32 -> ttyUSB1
```

> 换 USB 口后 `busnum/devpath` 可能变化，重新用 `udevadm info -a -n /dev/ttyUSB0` 核对。

---

## 4. 功能包部署与编译

板卡上只放运行需要的四个包，不复制 PC 的 build/install：

```bash
mkdir -p ~/mobile_manipulator_ws/src
# PC 上执行：
tar -C mobile_manipulator_ws/src -czf - \
  robot_interfaces robot_base robot_lidar robot_bringup \
  | ssh cat@192.168.0.4 'tar -C ~/mobile_manipulator_ws/src -xzf -'
```

板卡编译（关闭测试，2 线程避免 2G 内存吃紧）：

```bash
source /opt/ros/humble/setup.bash
cd ~/mobile_manipulator_ws
colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF \
  --parallel-workers 2
```

实测：4 个包约 5 分钟完成。

---

## 5. 板端启动

板端只跑：

- `robot_state_publisher`：URDF，发 `base_link → laser_link/wheel_*` 静态 TF、`/robot_description`
- `base_bridge`：`/odom`、`odom → base_link`、`/robot_status`、`/system_event`，订阅 `/cmd_vel`
- `x2_lidar`：`/scan`、`/lidar/points`

```bash
source /opt/ros/humble/setup.bash
source ~/mobile_manipulator_ws/install/setup.bash
ros2 launch robot_bringup board.launch.py
```

实测：

| 项 | 结果 |
| --- | --- |
| `/odom` | 约 49.8 Hz |
| `/scan` | 约 6.2 Hz（5 s 31 帧），720 bin，284 个有效点，0.133~3.926 m |
| TF | `odom → base_link`、`base_link → laser_link` 正常 |
| 状态 | `IDLE`，故障码 `0x0401` 电池低压（WARN），电池约 7216 mV |
| 串口 | `/dev/robot_stm32`、`/dev/robot_lidar` 均正常打开 |

> 电池低压是 WARN，不阻止使能，但正式测试前建议充电。

---

## 6. PC 端建图

PC 端只跑 SLAM + RViz，**不要**再启动 `robot_state_publisher`/`base_bridge`/`x2_lidar`，否则会出现重复 TF 和重复 `/scan`。

```bash
source /opt/ros/humble/setup.bash
source ~/mobile_manipulator_ws/install/setup.bash
ros2 launch robot_bringup remote_mapping.launch.py rviz:=true
```

PC 应能看到板卡节点和 topic：

```bash
ros2 node list
# /base_bridge /robot_state_publisher /x2_lidar
ros2 topic list
# /odom /scan /lidar/points /robot_status /robot_description
```

实测：`/map` 已发布，RViz Fixed Frame `map`，RobotModel 出现在地图中。最终地图 `room2` 为 **136×75 @0.05 m**（约 6.8×3.75 m），已保存到 `maps/room2.pgm/.yaml` 与 `room2.posegraph/.data`。

> `/scan` 是 `BEST_EFFORT`。`ros2 topic hz /scan` 默认 RELIABLE 收不到；用 RViz 或 Best Effort 订阅验证。

---

## 7. 遥控/使能

PC 端：

```bash
tmux attach -t teleop_pc
# 或新终端：
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

板端 `base_bridge` 订阅 `/cmd_vel`。先使能：

```bash
ros2 service call /set_enable std_srvs/srv/SetBool "{data: true}"
```

实测服务返回 `success=True`，失能返回 `success=True`，状态回到 IDLE。

> `teleop_twist_keyboard` 默认 linear 0.5 m/s、turn 1.0 rad/s；建图时建议先降速（`q/z` 调节）。

---

## 8. 故障与处理

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `apt update` 很慢/卡住 | 全量源 + Translation + embedfire 源 | 只保留 jammy main/universe/updates/security，`Acquire::Languages=none` |
| ROS 包突然找不到 | Ubuntu 更新用了 `List-Cleanup=1`，清掉了 ROS 索引 | 单独刷新 `ros2.list`，`List-Cleanup=0` |
| `/dev/ttyUSB*` 顺序变化 | 两颗 CH340 VID/PID 相同，底盘口 busnum 会变 | 用控制器 `DEVPATH` 的 udev 规则固定 |
| `/scan` 收不到 | 默认 `ros2 topic hz` 用 RELIABLE，雷达是 BEST_EFFORT | RViz 或 Best Effort 订阅 |
| `/odom`/TF 缺失 | 板端 `base_bridge` 没起/串口没开 | 看 `/tmp/board.log`，确认 `serial link up` |
| TF 重复/跳变 | PC 和板端同时跑 `robot_state_publisher`/`base_bridge` | 只让板端发，PC 只跑 `remote_mapping` |
| RViz 地图 GLSL 警告 | VMware/OpenGL 驱动 | 地图仍能渲染；必要时 `LIBGL_ALWAYS_SOFTWARE=1 rviz2` |

---

## 9. M4 结论

- **建图**：板端发 `/scan`、`/odom` 和 TF，PC 端 SLAM Toolbox 在线建图通过；最终 `room2` 136×75 @0.05 m，已保存栅格与位姿图。
- **定位**：SLAM Toolbox 在已建图上发布 `map→odom`，RViz 的 RobotModel 在 `map` 下与激光扫描对齐；这是 M4 的定位链路。AMCL/Nav2 定位属于 M5。
- **分工**：板卡 2+8G 只做传感器/串口桥接，PC 做建图/导航；等 4+32G 版本再考虑上板。

## 10. 后续

1. PC 端 Nav2（M5）：安装 `nav2_bringup`/`nav2_amcl`，用 `maps/room2`；
2. 板上不做建图/导航，等 4+32G 版本再迁移；
3. 加 IMU/EKF 时，`base_bridge` 的 `publish_tf` 改为 `false`，由 EKF 发 `odom→base_link`；
4. 板端加 systemd 自启、日志轮转；
5. 电池充电，复测 `0x0401` 是否消失。
