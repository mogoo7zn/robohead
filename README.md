# RoboGame 2026 — 机器人软件框架

RoboGame 2026 竞技机器人的完整软件栈。设计目标：

1. **Mac 上完成绝大部分开发与测试**（无硬件跑完整模拟比赛）
2. **无真实硬件也能跑完整流程**（Mock 后端）
3. **上传 GitHub 后，队友在 Raspberry Pi + STM32 + 实车上逐个接入真实硬件**
4. **替换真实硬件时零重写 Planner / HFSM / WorldState 等顶层代码**
5. **所有硬件相关模块有明确接口 + Mock + 实机 Backend**

## 快速开始（Mac，无需任何硬件）

```bash
make setup     # 创建 .venv 并安装依赖（含 opencv/numpy/pyyaml/pytest）
make test      # 全部单元 + 协议 + 集成测试（~200 个，秒级）
make mock      # 跑一场完整模拟比赛
```

`make mock` 的预期结果：机器人沿巡线导航到材料区，依次抓取 4 橙 + 2 紫，
在 A、B 两座塔各搭 3 层（紫色封顶），场上方块耗尽后立即进入 ENDGAME
安全停车，全程无看门狗触发、无 RECOVERY。

## 架构

```
┌───────────────────────────────────────────────────────────┐
│ 任务层  Planner / HFSM / MatchManager / Watchdog           │  纯决策
├───────────────────────────────────────────────────────────┤
│ 技能层  Navigation / Alignment / Grab / Place              │  闭环技能
├───────────────────────────────────────────────────────────┤
│ 服务层  Localization(融合) / Perception / Manipulator      │  传感+执行
├───────────────────────────────────────────────────────────┤
│ 硬件桥  McuClient ←二进制协议→ STM32 (UART)                │  唯一硬件通道
└───────────────────────────────────────────────────────────┘
```

**核心原则**

- **WorldState 单写者**：所有状态变更只能通过 `WorldStateStore.apply_event()`
  （事件在 `core/state/events.py`），读取用不可变 `snapshot()`。
- **所有超时/计时跑在 Clock 上**：测试用 `FakeClock`，永不真实等待。
- **硬件可注入**：见下表，换硬件只替换叶子，上层一字不改。

## 换硬件 = 换叶子

| 接口 | Mock（Mac） | 实机（Pi） |
|---|---|---|
| `Transport` | `MemoryTransport` + `FakeSTM32` | `SerialTransport`（hardware.yaml） |
| `MarkerDetector` | `MockMarkerDetector` | `CameraMarkerDetector`（AprilTag） |
| `BlockDetector` | `MockBlockDetector` | `CameraBlockDetector`（HSV） |
| `Clock` | `FakeClock` | `RealClock` |

两条入口共用同一装配（`core/mission/stack.py` 的 `MissionStack`）：

- Mac：`python -m mock.runner`
- Pi：`ros2 launch robogame_bringup mission.launch.py`

## Raspberry Pi 部署

```bash
# 1. 编译 ROS 2 工作空间（本机无 ROS 也可，走 Docker）
make ros-build

# 2. 配置串口（config/hardware.yaml）
#    mcu.transport: serial
#    mcu.serial.device: /dev/serial0

# 3. 实机场地（config/real_field.yaml，先测量再改）

# 4. 启动（含配置预检）
make run        # = bash scripts/start_robot.sh

# 或开机自启（可选）
sudo cp deploy/robogame.service /etc/systemd/system/
sudo touch /opt/robogame/ENABLE && sudo systemctl enable --now robogame
```

## 目录结构

```text
core/     核心包（硬件无关，含 mock 后端）
  model/           枚举 / Pose2D / 事件基础
  state/           WorldState + Store（单写者）
  protocol/        CRC16 / 帧定义 / 流解析
  hardware/        McuClient + Transport（memory/serial）
  mission/         HFSM 状态 / Planner / Watchdog / MissionStack
  navigation/      RouteGraph / Navigator / 底盘控制
  localization/    MarkerMap / 位姿估计 / 里程+IMU+视觉融合
  perception/      方块检测（mock+HSV） / AprilTag 检测（mock）
  skill/           Alignment / Grab / Place 技能
  manipulation/    机械臂序列器（走协议）
  mock/            SimWorld / FakeSTM32（物理仿真）
mock/     Mac 上的完整模拟比赛入口
ros2_ws/src/
  robogame_interfaces/   ROS msg 定义
  robogame_adapters/     相机后端（AprilTag / HSV）
  robogame_bringup/      实机任务节点 + launch
config/            全部 yaml 配置（策略/导航/感知/硬件/场地）
firmware/stm32/    协议头（由 Python 单一真源生成，勿手改）
scripts/           check_config / gen_protocol_header / start_robot
tests/             unit + protocol + integration
```

## Pi ↔ STM32 通信协议

帧格式（小端）：`AA 55 | VER | SEQ | MSG_ID | LEN(u16) | PAYLOAD | CRC16(u16)`
CRC16-CCITT（poly 0x1021, init 0xFFFF），覆盖 VER..PAYLOAD。

协议唯一真源是 `core/protocol/messages.py`；
C 头文件由脚本生成：`python scripts/gen_protocol_header.py`，
CI 会校验漂移（`--check`）。

## 测试

```bash
make test                  # 全部
pytest tests/unit -q       # 单元（含 ROS 适配器，无需安装 ROS）
pytest tests/protocol -q   # 协议/CRC/流解析
pytest tests/integration -q# 端到端模拟比赛
make check-config          # 配置完整性（TODO 残留/图连通性/相机参数）
```

## 文档

详细文档（中文）见 `docs/`：

- 项目导读：`docs/README.md`
- 系统架构（分层 / HFSM / Planner / WorldState / Watchdog）：`docs/architecture.md`
- Pi ↔ STM32 通信协议：`docs/protocol.md`
- 开发指南（环境 / 测试 / 模拟比赛 / 配置 / 约定）：`docs/development.md`
- 实机部署（树莓派 + ROS 2）：`docs/deployment.md`
- 原始算法设计：`docs/design/RoboGame2026_algo.md`
