# 工具索引

> PC 侧工具按用途分目录；每个工具只做一件事，入口都不依赖仓库外的文件。

```text
tools/
├── bench/       台架测试：真机闭环、协议联调
├── docgen/      文档生成：Markdown → Word
└── host_gui/    上位机：Qt 串口控制台
```

| 工具 | 用途 | 依赖 | 引入节点 |
| --- | --- | --- | --- |
| [`bench/proto_loop.py`](bench/proto_loop.py) | 串口协议闭环测试：下发 → ACK → 上报三段核对，支持静态/运动/模拟器 | `pyserial` | M2 |
| [`docgen/md2docx.py`](docgen/md2docx.py) | 把 `docs/**/*.md` 转成 `docs/word/**/*.docx`，目录结构镜像 | `python-docx` | M2 |
| [`host_gui/`](host_gui/) | 上位机：遥测曲线、速度滑杆、急停、机械臂与夹爪 | `PySide6`、`pyserial` | M2 |

## 常用命令

```bash
# 台架闭环（底盘架空后再加 --move）
python tools/bench/proto_loop.py --list
python tools/bench/proto_loop.py --port COM7 --auto
python tools/bench/proto_loop.py --sim --auto --move

# 上位机（--sim 不需要硬件）
python tools/host_gui/main.py
python tools/host_gui/main.py --sim

# 文档转 Word
python tools/docgen/md2docx.py
```

## 约定

- **工具不写仓库状态**：抓包、截图、日志都写到临时目录或 `build/`，不入库（`.gitignore` 已覆盖）。
- **台架脚本必须能自检**：`proto_loop.py` 的 `--sim` 用内置模拟器跑同一套用例，脚本本身改坏了能先在这里发现。
- **复用而不是复制编解码**：`proto_loop.py` 直接 import `host_gui/protocol.py`，全仓库只有一份上位机侧的解码实现，避免两套实现各自漂移。

文档见 [`docs/README.md`](../docs/README.md)；测试代码见 [`tests/README.md`](../tests/README.md)。
