# RoboGame 2026 项目文档

RoboGame 2026 竞技机器人完整软件栈：**Mac 上无硬件跑完整模拟比赛，实机上零重写接入真实硬件**。

## 文档地图

| 文档 | 内容 |
|---|---|
| [architecture.md](architecture.md) | 系统架构：分层设计、数据流、HFSM、Planner、WorldState、看门狗 |
| [protocol.md](protocol.md) | 树莓派 ↔ STM32 二进制通信协议：帧格式、17 条消息、代码生成 |
| [development.md](development.md) | 开发指南：环境搭建、测试、模拟比赛、配置文件、代码约定 |
| [deployment.md](deployment.md) | 实机部署：树莓派 + ROS 2 + STM32 上车步骤、启动与排障 |
| [design/](design/) | 原始设计文档（[算法设计](design/RoboGame2026_algo.md)、[早期部署规划](design/deployment.md)），实现以本文档集为准 |

**阅读顺序建议**：新成员先读本文 + [architecture.md](architecture.md) 建立全局认知，
再按角色分工——调策略/导航读 architecture 的 Planner 与导航节，写驱动/调协议读
[protocol.md](protocol.md)，改代码前必读 [development.md](development.md) 的代码约定，
上装调车读 [deployment.md](deployment.md)。

## 软件栈一览

```
┌──────────────────────────────────────────────────────────┐
│ 任务层  Planner / HFSM / MatchManager / Watchdog          │  纯决策
├──────────────────────────────────────────────────────────┤
│ 技能层  Navigation / Alignment / Grab / Place             │  闭环技能
├──────────────────────────────────────────────────────────┤
│ 服务层  Localization(融合) / Perception / Manipulator     │  传感+执行
├──────────────────────────────────────────────────────────┤
│ 硬件桥  McuClient ←二进制协议→ STM32 (UART)               │  唯一硬件通道
└──────────────────────────────────────────────────────────┘
```

| 目录 | 职责 |
|---|---|
| `core/` | 核心包（硬件无关），上表四层全部实现 |
| `mock/` | Mac 模拟比赛入口（`make mock`） |
| `ros2_ws/` | ROS 2 工作空间：实机用的相机后端、任务节点、消息定义 |
| `config/` | 全部 YAML 配置（策略/导航/感知/硬件/场地） |
| `firmware/stm32/` | 协议 C 头文件（由 Python 单一真源生成，勿手改） |
| `scripts/` | 配置校验 / 协议头生成 / 实机启动脚本 |
| `tests/` | unit + protocol + integration（210 个测试） |

## 常用命令

```bash
make setup        # 创建 .venv 并安装依赖（Mac 友好）
make test         # 全部测试（无需硬件，秒级）
make mock         # 跑一场完整模拟比赛
make check-config # 校验全部配置文件
make lint         # compileall 语法检查
```

更多命令（`make ros-build` / `make run` 等）见 [development.md](development.md) 与 [deployment.md](deployment.md)。

实机部署与启动见 [deployment.md](deployment.md)。
