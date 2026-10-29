# 系统事件，对应 MCU REPORT_EVENT (0xC4)。
# v0 草案：事件类型由 fault_code 与状态变化共同判断，字段在 M8 前冻结。

std_msgs/Header header

# 故障码；0 表示无故障
uint16 fault_code

# MCU 上电后的毫秒时间戳
uint32 mcu_uptime_ms
