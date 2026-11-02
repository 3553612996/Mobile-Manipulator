# BtCar —— 手机蓝牙遥控

Android 原生（Kotlin + Compose）遥控端：用 HC-05/HC-06 一类的 SPP 模块接在 STM32 的 USART3 上，
替代 RK3566 做主控，手机上直接发 protocol v1 的二进制帧，实时看车回传的状态。

机械臂部分先留了占位卡片，等机械臂接入后再补关节滑条和夹爪按钮。

## 界面

| 区域 | 内容 |
|------|------|
| 顶部 | 链路状态点、设备名/地址、连接 / 断开 |
| 实时状态 | 状态机、故障码、电量、板温、最近一帧距今多久、收发帧数 |
| 最大速度 | 100–500 mm/s，步进 50 |
| 行驶控制 | 摇杆、使能 / 失能、急停、清除故障 |
| 运动反馈 | 左右轮实测速度、X/Y 位置、航向 |
| 机械臂 | 预留 |

摇杆上/下 = 前进/后退（vx），左/右 = 转向（wz，右推顺时针）。
protocol v1 没有横移自由度，所以摇杆的横向分量只映射到角速度。

## 和固件的配合

1. **连接后立刻以 20 ms 周期发 `SET_VELOCITY`**，没使能时发的是零速。
   固件在使能状态下 200 ms 收不到速度指令就判 `0x0202 指令超时`，
   所以“先保持零速流、再使能”是必须的顺序。
2. **只有使能后摇杆才会进速度帧**，避免点使能的瞬间按上一帧残留值窜出去。
3. **急停会锁存**：`ESTOP` 之后状态进 `ESTOP`，`SET_ENABLE` 会被拒绝，
   必须先 `CLEAR_FAULT(0xFFFF)` 再重新使能。
4. **断开 / 切后台先发零速 + `SET_ENABLE(0)`**，再关 socket。
   直接断流会让固件在使能状态下锁存超时故障。
5. 上限沿用固件：`vx ≤ 500 mm/s`、`wz ≤ 1500 mrad/s`；手机端把 `maxSpeed` 线性映射到这两个上限。

## 接线（测试用）

```
STM32 USART3_TX (PB10) ──► HC-05 RXD
STM32 USART3_RX (PB11) ◄── HC-05 TXD
GND ──────────────────── GND
HC-05 VCC ────────────── 5V（模块自带 LDO，别接 3.3V 供电脚）
```

- 固件 UART 配置是 **115200 8N1 无流控**（`APP_ROBOT_UART_BAUDRATE`），HC-05 必须改成同样的波特率。
- **USART3 是 RK3566 和蓝牙共用的**：两者不能同时接线，测试蓝牙时把 RK3566 那一侧的 TX/RX 拔掉，
  否则两个主机一起发会把固件收到的字节流搅乱。

## HC-05 配置

用 USB-TTL 进 AT 模式（按住模块按键上电，或 EN/KEY 拉高），串口助手 38400 8N1：

```
AT                      -> OK
AT+UART=115200,0,0      -> OK      # 9600 改 115200
AT+ROLE=0               -> OK      # 从机
AT+NAME=BT-CAR          -> OK      # 可选
AT+PSWD=1234            -> OK      # 配对码
AT+RESET                -> OK
```

## 手机侧使用

1. 系统蓝牙设置里先配对 `BT-CAR`（配对码默认 1234）。
2. 打开 App → 连接 → 选设备。首次会申请 `BLUETOOTH_SCAN` / `BLUETOOTH_CONNECT`（Android 12+）
   或定位权限（Android 11 及以下，系统扫描蓝牙需要）。
3. 点「使能」，再推摇杆。松手立刻回零。
4. 「急停」→ 车锁死在 `ESTOP`；「清除故障」后需要重新点「使能」。

## 代码结构

```
app/src/main/java/com/wuying/btcar/
├── MainActivity.kt          权限、蓝牙开关、设备选择、切后台失能
├── BtCarViewModel.kt        会话状态、20 ms 速度流、各条命令、上报分发
├── link/SppLink.kt          RFCOMM 连接（安全失败退回不安全）、收帧、发帧、等 ACK 重发
├── protocol/Protocol.kt     v1 帧格式、CRC-16/CCITT-FALSE、增量解析器
├── protocol/Messages.kt     0xC1..0xC7 上报解码
└── ui/                      Theme / ControlScreen / Joystick
app/src/test/java/.../ProtocolTest.kt   编解码 + 解析器单测
```

协议常量和 `firmware/app/05_Middlewares/protocol/protocol_def.h`、`robot_base/protocol.hpp`
是同一份协议的三处实现，改协议要三处一起改，并跑 `docs/architecture/protocol.md` 里的测试向量。

## 构建

需要 JDK 17 和 Android SDK 36：

```bash
cd tools/android
# local.properties 里写 sdk.dir=<SDK 路径>
./gradlew :app:assembleDebug
# 产物：app/build/outputs/apk/debug/app-debug.apk
./gradlew :app:testDebugUnitTest
```

`gradle/wrapper/gradle-wrapper.properties` 里的下载地址指向腾讯云镜像（官方地址会 302 到 GitHub，
国内经常拉不下来）；换网络环境时可以改回 `https://services.gradle.org/distributions/gradle-9.4.1-bin.zip`。

AGP 9 自带 Kotlin 支持（内置 KGP 2.2.10），所以 `build.gradle.kts` 里**不要**再声明
`org.jetbrains.kotlin.android`，否则会报 `Cannot add extension with name 'kotlin'`。

## 常见问题

- **测试 JVM 起不来，报 `ClassNotFoundException: Files`**：Windows 的系统 PATH 里混进了双引号
  （例如 `C:\WINDOWS\system32"`），AGP 会把它原样拼进 `-Djava.library.path`，命令行被截断。
  把 PATH 里那个引号删掉即可。
- **连上了但收不到数据**：先确认 HC-05 波特率是 115200；再看 App 里的「帧」计数，
  一直不涨说明帧格式或波特率不对，有「错」说明在收但 CRC 不过（多半还是波特率）。
- **使能被拒绝**：状态不在 `IDLE`/`MANUAL`（例如还停在 `ESTOP` 或故障里），先清故障。

## 机械臂预留

`SET_ARM_JOINTS(0x02)`、`SET_GRIPPER(0x03)`、`ARM(0xC2)` 已经在 `Protocol.kt` / `Messages.kt`
里定义好了解码，界面上只放了占位卡片。接入时在 `ControlScreen` 的 `ArmPlaceholder()` 位置
换成六路滑条 + 夹爪按钮，`BtCarViewModel` 里加两个发送函数即可，链路和解析不用动。
