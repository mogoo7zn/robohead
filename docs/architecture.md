# 系统架构

## 设计原则（五条不变量）

整个代码库围绕五条不变量展开，改任何模块前先确认不破坏它们：

1. **WorldState 单写者**——模块只能发事件，不能改状态；读用不可变快照；
2. **Clock 注入**——一切计时走 `Clock` 接口，代码里不出现 `time.sleep/time.time`；
3. **叶子注入**——硬件差异收敛到 4 个可注入接口（Clock/Transport/双相机检测器），换硬件零改顶层；
4. **协议单一真源**——消息表只在 Python 定义，C 头自动生成；
5. **参数进 YAML**——场地、阈值、PID、速度上限一律配置化，代码不含魔法数。

## 分层设计

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

- 上层只依赖接口，不依赖具体硬件；换硬件只替换最底层的"叶子"。
- 树莓派与 STM32 之间只有一条 UART 二进制通道（见 [protocol.md](protocol.md)）。

## 一次比赛循环的数据流

```
相机帧 / UART 遥测
   │
   ▼
Perception / McuClient            （感知与硬件桥，产出原始数据）
   │  事件（MarkerSeen / BlockSeen / Odom / IMU …）
   ▼
WorldStateStore.apply_event()     （单写者，唯一写入入口）
   │  snapshot()（不可变快照）
   ▼
Planner.decide()                  （现在该做什么？）
   │
   ▼
HFSM tick()                       （状态机驱动技能）
   │
   ▼
Navigator / Alignment / Grab / Place   （技能层闭环）
   │  CMD_VELOCITY / CMD_LINE_FOLLOW_* / CMD_GRIPPER …
   ▼
McuClient → UART → STM32          （执行）
```

## 核心机制

### 1. WorldState 单写者

所有模块对世界状态的认知都汇聚在一个 `WorldState`（`core/state/world_state.py`）里，
包含 10 个不可变 section：

| Section | 内容 |
|---|---|
| `robot` | x / y / yaw / 速度 / 定位置信度 |
| `route` | 当前节点、当前段、巡线状态 |
| `match` | 已用/剩余时间、阶段（NORMAL/SAFE/ENDGAME） |
| `inventory` | 携带的橙/紫方块数（规则约束：总数≤3、紫≤1） |
| `supply` | 场上剩余方块估计（None=未知，乐观；0=已耗尽） |
| `perception` | 最近看到的 marker / 方块 |
| `mission` | 当前任务、当前技能、HFSM 状态、重试次数 |
| `hardware` | 相机/MCU/IMU/底盘等健康标志 |
| `manipulator` | 轴位置、夹爪状态、是否夹住 |
| `building` | 三座塔的层数与紫顶标志 |

**规则**：任何模块不得直接修改 WorldState；只能构造事件（`core/state/events.py`）
交给 `WorldStateStore.apply_event()`。读取一律用不可变 `snapshot()`。
这保证了状态变更可追溯、测试可断言。

### 2. Clock 注入

所有超时与计时跑在 `Clock` 接口上：测试和模拟用 `FakeClock`（永不真实等待，秒级跑完整场比赛），实机用 `RealClock`。

### 3. MissionStack 装配（换硬件 = 换叶子）

`core/mission/stack.py` 的 `MissionStack` 是唯一装配层，把上面四层组装成一台"完整的大脑"，
只留 4 个可注入叶子：

| 叶子接口 | Mac 模拟（mock） | 实机（Pi） |
|---|---|---|
| `Clock` | `FakeClock` | `RealClock` |
| `Transport` | `MemoryTransport` + `FakeSTM32` | `SerialTransport`（UART） |
| `MarkerDetector` | `MockMarkerDetector` | `CameraMarkerDetector`（AprilTag，ROS） |
| `BlockDetector` | `MockBlockDetector` | `CameraBlockDetector`（HSV，ROS） |

两条入口共用同一装配：

- Mac：`python -m mock.runner`
- Pi：`ros2 launch robogame_bringup mission.launch.py`（50 Hz 控制循环）

### 4. HFSM 状态机

顶层比赛状态机（`core/mission/states.py`，括号内为超时秒数，超时一律走安全出口）：

```
BOOT(10) → SELF_CHECK(15) → WAIT_START(等按钮) → INITIAL_LOCALIZE(10) → MISSION ─┐
                                                                                │子机完成
      watchdog: RECOVER ↙        ↘ watchdog: SAFE_STOP                          ▼
      RECOVERY(20) ←──→ MISSION      （任何状态） ──────────────→  ENDGAME(10) → SAFE_STOP → DONE
            超时 ↘                                    （有货先放掉再收尾）
                  ↘ SAFE_STOP
```

- `WAIT_START` 无超时：死等物理启动按钮（`START_EVENT`，走 STM32，不依赖网络）；
- `INITIAL_LOCALIZE` 超时也放行——带里程计先跑，边跑边修；
- `RECOVERY` 停车重定位，置信度恢复 → 回 `MISSION`，20s 修不好 → `SAFE_STOP`。

`MISSION` 是复合状态，内部再跑一台任务子状态机：

```
PLAN(5) → ACQUIRE_NAV(120) → ACQUIRE_ALIGN(30) → ACQUIRE_GRAB(60) → PLAN    抓取链
PLAN → BUILD_NAV(120) → BUILD_PLACE(90，可连续放完一车) ───────────→ PLAN    搭建链
任何失败 → TASK_FAILED(2) → 重试未超限回 PLAN；超限 → 子机 DONE → 顶层 ENDGAME
```

- `PLAN` 调 `Planner.decide()` 决定下一个任务；决策 ENDGAME 时若车上还有货，
  会先走搭建链把货放完再收尾（放掉的块才计分）；
- `MISSION` 每帧先问 Watchdog：健康 → 继续任务；可恢复 → RECOVERY；危险 → SAFE_STOP；
- 每个状态都有超时与失败出口，最终兜底是 SAFE_STOP（全停 + 结束比赛）。

### 5. Planner（规则 + Utility）

`core/strategy/planner.py`，候选任务：`ACQUIRE_ORANGE` / `ACQUIRE_PURPLE` / `BUILD` / `ENDGAME`。

```
utility = expected_score − 时长 × time_weight(0.05) − 风险 × risk_weight(0.4)
```

硬规则（优先于打分）：

1. 剩余时间 < 45s（`min_remaining_for_new_task`）→ ENDGAME；
2. 满载（3 块）→ BUILD；
3. 带着紫块 → BUILD（紫块只能做塔顶，尽早放掉）。

软规则：

- **supply 语义**：某颜色场上存量已知为 0 → 跳过该颜色 ACQUIRE；None（未知）→ 乐观保留；
- 无任何候选 → ENDGAME（"场上没有方块了"）；
- 连续失败的任务按 `8 × retry_count` 递减 utility，避免死磕不存在的方块；
- SAFE 阶段对抓取候选额外加风险惩罚；
- `purple_as_roof` 策略下按"橙、橙、紫"凑满一车再建塔，保证紫块恰好封顶。

### 6. Watchdog（两级安全）

`core/mission/watchdog.py` 持续跟踪 MCU 心跳、双相机、定位、IMU、里程计的"最后出现时间"，按配置超时给出三档裁决：

- `OK`：继续比赛；
- `RECOVER`：进入 RECOVERY（局部重试 / 重新定位）；
- `SAFE_STOP`：全停。

MCU 侧还有独立看门狗：Pi 每 50ms 发 HEARTBEAT，STM32 超过 200ms 收不到就自动零速停车——即使树莓派死机，机器人也不会失控。

## 定位与导航

**定位**（`core/localization/`，参数见 `config/localization.yaml`）：
里程计 dead-reckoning + AprilTag 绝对修正。
维护一个校正变换 `T_map_odom`，`map_pose = T_map_odom ∘ odom_pose`——
marker 修正只更新变换，绝不回写 MCU 的里程计流，保证两次修正之间不漂移、不倒跳。
置信心随时间衰减，低于阈值触发重新定位。

**导航**（`core/navigation/`，参数见 `config/navigation.yaml`）：
路线图 `RouteGraph`（黑线网络拓扑，来自 routes yaml）。
`Navigator` 逐段执行：

1. **TURN**：原地转向对准线段方向；
2. **FOLLOW**：下发 `CMD_LINE_FOLLOW_START`，由 STM32 做低层巡线闭环；
3. **ARRIVE**：由 Pi 根据融合位姿判定到达（不信任线控的到达判断），下发停止。

## 目录结构

```
core/
  model/          枚举 / Pose2D / 结果类型
  state/          WorldState + Store（单写者）+ 事件定义
  protocol/       CRC16 / 帧编解码 / 消息表（协议唯一真源）
  hardware/       McuClient + Transport（memory / serial）
  mission/        HFSM / Planner 装配 / MatchManager / Watchdog / MissionStack
  strategy/       Planner（规则 + utility）
  navigation/     RouteGraph / RoutePlanner / Navigator / 底盘抽象
  localization/   MarkerMap / 位姿估计 / odom+IMU+marker 融合
  perception/     方块检测（mock + HSV） / marker 检测（mock）
  skill/          Alignment / Grab / Place 技能
  manipulation/   机械臂序列器（X/Z 轴 + 夹爪，走协议）
  mock/           SimWorld / FakeSTM32（物理与 MCU 仿真）
  utils/          Clock / config / log
mock/             模拟比赛入口 runner.py
ros2_ws/src/
  robogame_interfaces/   ROS msg 定义（Pose2D / BlockDetection / MissionStatus）
  robogame_adapters/     相机后端（AprilTag / HSV，"最新帧 + 消费式 detect()"）
  robogame_bringup/      实机任务节点 mission_node + launch
```
