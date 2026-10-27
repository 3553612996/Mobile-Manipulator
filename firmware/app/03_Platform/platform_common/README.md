# platform_common 设计说明

新增的平台公共基础层，只负责建立后续所有 Platform、Service、App 都要共同使用的工程语言。

##落地文件

| 文件 | 职责 |
| --- | --- |
| `platform_type.h` | 从 `04_Impl/impl_board/board_types.h` 引出平台统一基础类型 |
| `platform_error.h` | 定义 `platform_err_t` 和统一的成功/失败判断宏 |
| `platform_def.h` | 收纳通用状态、布尔、NULL、对齐、数组长度和延时声明 |
| `README.md` | 说明本目录的边界、依赖方向和课堂验收点 |

## 依赖方向

```text
04_Impl/impl_board/board_types.h
        |
        v
03_Platform/platform_common/platform_type.h
        |
        +--> platform_error.h
        |
        +--> platform_def.h
```

`board_types.h` 是当前板级/当前编译环境的基础类型源头。`platform_type.h` 是给 Platform、Service、App 使用的类型出口。上层代码不建议直接 include `board_types.h`。


## 注意点

`platform_error.h` 不再同时使用同名 `#define` 和 enum 枚举值。错误码只保留 enum 定义，避免预处理阶段把枚举名替换成数字后导致编译错误。

`int64` / `uint64` 在 STM32/Keil 这类 32 位环境下应使用 `long long`，不能写成 `long`，否则很可能只有 32 位。
