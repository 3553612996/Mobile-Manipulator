# 文档索引

| 文档 | 内容 | 状态 |
| --- | --- | --- |
| [project-intro.md](project-intro.md) | 项目介绍：背景、范围、选型理由、指标、里程碑、已知限制 | v1 |
| [architecture.md](architecture.md) | 项目框架：双控分工、硬件拓扑、软件分层、数据流、部署、构建 | v1 |
| [software-design.md](software-design.md) | 软件结构：包划分、接口、状态机、并发、错误处理、参数管理 | v1 |
| [protocol.md](protocol.md) | 串口协议：帧格式、命令字、CRC、超时、故障码、测试向量 | v1 草稿，实现中 |
| [commit-convention.md](commit-convention.md) | 提交规范：type/scope、拆分粒度、分支、标签 | v1 |
| [coding-style.md](coding-style.md) | 编码规范：命名、单位、错误处理、ISR 约束、格式化 | v1 |
| [testing.md](testing.md) | 自测规范：测试分层、故障注入、发版检查清单 | v1 |
| [test-report-m2.md](test-report-m2.md) | M2 节点测试报告：环境、L1/L3 结果、缺陷记录、已知限制 | M2 通过 |
| [ai-usage.md](ai-usage.md) | AI 使用说明：使用边界、验证要求、记录方式 | v1 |

## 维护约定

1. **协议变更时，代码与 `protocol.md` 在同一个提交里修改。**
2. 实测数据填入 `project-intro.md` 的指标表，**未测的保持"待测"**，不填估计值。
3. 架构或接口有调整时，同步更新 `architecture.md` / `software-design.md`。
4. 文档里的设计决定都写了理由。如果发现某条理由已经不成立，直接改掉并说明原因。
