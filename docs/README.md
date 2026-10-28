# 文档索引

```text
docs/
├── project-intro.md          项目介绍：背景、范围、选型、指标、里程碑
├── architecture/             架构与协议：怎么分层、接口是什么、协议怎么编解码
├── conventions/              工程规范：提交、编码、测试、AI 使用
├── nodes/                    里程碑产出：每个节点的测试报告与架构检查
└── word/                     Word 副本，由 tools/docgen/md2docx.py 生成
```

## 架构与协议

| 文档 | 内容 |
| --- | --- |
| [architecture/overview.md](architecture/overview.md) | 项目框架：双控分工、硬件拓扑、软件分层、数据流、部署 |
| [architecture/software-design.md](architecture/software-design.md) | 软件结构：包划分、接口、状态机、并发、错误处理、参数管理 |
| [architecture/protocol.md](architecture/protocol.md) | 串口协议 v1：帧格式、命令字、CRC、超时、故障码、测试向量 |

## 工程规范

| 文档 | 内容 |
| --- | --- |
| [conventions/commit-convention.md](conventions/commit-convention.md) | 提交规范：type/scope、拆分粒度、分支、版本与标签 |
| [conventions/coding-style.md](conventions/coding-style.md) | 编码规范：命名、单位、错误处理、ISR 约束、格式化 |
| [conventions/testing.md](conventions/testing.md) | 自测规范：测试分层、用例要求、故障注入、发版检查清单 |
| [conventions/ai-usage.md](conventions/ai-usage.md) | AI 使用说明：使用边界、验证要求、记录方式 |

## 节点产出

每个里程碑一个文件夹，放该节点的结论性文档。索引与约定见 [nodes/README.md](nodes/README.md)。

| 节点 | 产出 |
| --- | --- |
| M2 | [测试报告](nodes/m2/test-report.md)、[架构分层检查](nodes/m2/layering-audit.md) |

## 项目级

| 文档 | 内容 |
| --- | --- |
| [project-intro.md](project-intro.md) | 项目介绍：背景、范围、技术选型理由、指标、里程碑、已知限制 |

## 维护约定

1. **协议变更时，代码与 `architecture/protocol.md` 在同一个提交里修改。**
2. 实测数据填入 `project-intro.md` 的指标表，**未测的保持"待测"**，不填估计值。
3. 架构或接口有调整时，同步更新 `architecture/overview.md` / `architecture/software-design.md`。
4. 文档里的设计决定都写了理由。如果发现某条理由已经不成立，直接改掉并说明原因。
5. **Word 副本由脚本生成，不手改**：改完 Markdown 后运行 `python tools/docgen/md2docx.py`。目录结构会镜像到 `docs/word/`。
6. **可执行的东西不放 `docs/`**：测试在 [`tests/`](../tests/)，工具在 [`tools/`](../tools/)。
