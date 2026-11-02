# robot_interfaces

所有节点共享的自定义消息、服务与动作。只放接口定义，不放实现代码。

## 接口

| 类型 | 文件 | 用途 |
| --- | --- | --- |
| `msg/RobotStatus.msg` | 机器人整体状态 | 对应 MCU `REPORT_STATUS (0xC3)` |
| `msg/SystemEvent.msg` | 系统事件 | 对应 MCU `REPORT_EVENT (0xC4)` |
| `srv/ClearFault.srv` | 清除故障 | 对应 MCU `CLEAR_FAULT (0x07)` |
| `srv/GetVersion.srv` | 查询版本 | app / bootloader / git hash |
| `action/Grasp.action` | 抓取动作 | 长任务，支持反馈与取消 |

## 版本状态

**v0 草案，尚未冻结。** 字段按 `docs/architecture/protocol.md` 设计，冻结节点在 M8 抓取闭环之前。冻结前修改字段时，必须同步 `docs/architecture/software-design.md` 与两端实现。

## 依赖

- `std_msgs`：`Header`
- `geometry_msgs`：`PoseStamped`

## 生成后检查

```bash
ros2 interface show robot_interfaces/msg/RobotStatus
ros2 interface show robot_interfaces/action/Grasp
```

C++ 头文件由 `rosidl_generate_interfaces` 生成，包含路径形如：

```cpp
#include "robot_interfaces/msg/robot_status.hpp"
#include "robot_interfaces/action/grasp.hpp"
```
