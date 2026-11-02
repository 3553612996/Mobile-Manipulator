# 机器人整体状态，对应 MCU REPORT_STATUS (0xC3)。
# v0 草案：字段在 M8 抓取闭环前冻结。

std_msgs/Header header

# 控制状态
uint8 STATE_INIT=0
uint8 STATE_IDLE=1
uint8 STATE_MANUAL=2
uint8 STATE_AUTO=3
uint8 STATE_ESTOP=4
uint8 STATE_FAULT=5
uint8 state

# 当前锁存的故障码；0 表示无故障。编码见 docs/architecture/protocol.md 第 8 节
uint16 fault_code

# 电池电压，mV
uint16 battery_mv

# MCU 温度，摄氏度
int8 temperature_c

# 最大调度抖动，微秒
uint16 max_sched_jitter_us
