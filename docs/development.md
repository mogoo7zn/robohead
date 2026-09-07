# 开发指南

## 环境搭建（Mac，无需任何硬件）

```bash
make setup     # 创建 .venv 并安装依赖（numpy / PyYAML / OpenCV / pyserial / pytest）
make test      # 210 个测试，秒级
make mock      # 一场完整模拟比赛
```

注意：系统 python3 可能过旧（macOS 自带 3.9），一律用 `.venv/bin/python`。

## 常用 make 目标

| 命令 | 用途 |
|---|---|
| `make setup` | 创建 .venv 并安装 dev 依赖 |
| `make test` | 全部 pytest |
| `make mock` | 完整模拟比赛 |
| `make check-config` | 配置完整性校验 |
| `make lint` | compileall 语法检查（core/mock/scripts） |
| `make clean` | 清理 venv / 构建产物 / 缓存 |
| `make docker-build` | 构建 ROS 2 Jazzy 开发容器（arm64/amd64） |
| `make ros-build` | 在容器内编译 ros2_ws（本机无需装 ROS） |
| `make run` | 树莓派上启动机器人（见 [deployment.md](deployment.md)） |

## 测试

```bash
make test                      # 全部（210 个）
pytest tests/unit -q           # 单元：HFSM / Planner / 定位 / 导航 / 感知 / ROS 适配器
pytest tests/protocol -q       # 协议：CRC / 帧编解码 / 流解析 / 往返一致性
pytest tests/integration -q    # 集成：端到端完整模拟比赛（真实装配 MissionStack）
pytest tests/unit/test_planner.py -q   # 单个文件
```

- 所有计时跑在 `FakeClock` 上，永不真实等待；
- ROS 适配器测试无需安装 ROS（rclpy 未安装时自动跳过真实节点部分）；
- CI（`.github/workflows/ci.yml`）：ubuntu + macos × Python 3.11/3.12，
  跑 config 校验、全部测试、完整模拟比赛、compileall 与协议头漂移检查。

## 模拟比赛（make mock）

```bash
python -m mock.runner                 # 尽快跑完（默认）
python -m mock.runner --realtime      # 按真实时间比例回放
python -m mock.runner --max-time 420  # 自定义比赛时长上限（秒）
python -m mock.runner -v              # 逐 tick 详细日志
```

它装配**真实的任务/技能/服务层代码**，只把 4 个叶子换成仿真：

- `SimWorld`（`core/mock/sim_world.py`）：机器人运动学、方块、巡线 pure-pursuit 仿真；
- `FakeSTM32`（`core/mock/fake_stm32.py`）：协议级 MCU 仿真——解析命令帧、产生遥测帧，
  和真 STM32 走完全相同的二进制协议。

预期结果（当前基线）：巡线全程稳定无逃逸 → 抓完 4 橙 + 2 紫 → A/B 双塔各 3 层紫顶 →
场上方块耗尽进入 ENDGAME 安全停车（约 t=194s），全程看门狗不触发。
终端有 1Hz trace 可用于调试。

## 配置文件（config/）

| 文件 | 用途 | 何时改 |
|---|---|---|
| `strategy.yaml` | Planner utility 权重、任务时长估计、比赛时间阈值 | 调策略 |
| `navigation.yaml` | 巡线/导航参数（pure-pursuit、到达半径、速度上限） | 调运动 |
| `localization.yaml` | 融合参数（marker 权重、置信度衰减、重定位阈值） | 调定位 |
| `perception.yaml` | HSV 阈值、AprilTag 参数、对准控制器增益 | 调视觉 |
| `manipulation.yaml` | X/Z 轴行程与速度、抓/放序列位置（mm） | 机械标定后 |
| `hardware.yaml` | 传输选择：Mac 用 memory，Pi 用 serial（`/dev/serial0`） | 换硬件时 |
| `mock_field.yaml` / `mock_routes.yaml` | 模拟场地与路线（完整可用值） | 一般不动 |
| `real_field.yaml` / `*.example.yaml` | 实机场地模板（**必须实测后填写**） | 赛前标定 |

规则：

1. 场地尺寸、塔位、marker 坐标、HSV 阈值、PID、速度上限等**一律进 YAML**，禁止写死在代码里；
2. 每次改配置后跑 `make check-config`——校验必填项、TODO 残留、路线图连通性、相机参数完整性；
3. mock 配置与实机配置严格分离，测试参数禁止混入比赛配置。

## 代码约定

1. **WorldState 单写者**：新模块要更新世界状态，只能新增一个事件
   （`core/state/events.py`）并在 `core/state/store.py` 的 `apply_event` 里处理，
   绝不直接改字段。
2. **计时用注入的 Clock**，不调用 `time.sleep` / `time.time`，保证测试零等待。
3. **技能返回 `SkillStatus`**（RUNNING / SUCCESS / FAILURE / TIMEOUT），HFSM 只消费状态。
4. **新增一个硬件后端**的步骤：
   - 实现与 mock 相同的接口（Transport / MarkerDetector / BlockDetector / Clock）；
   - 在 `MissionStack` 注入点换掉叶子，或加一个配置项选择后端；
   - 在 `tests/unit` 加对应单测（参考 `test_ros_adapters.py`）。
5. 协议改动只能改 `core/protocol/messages.py`，然后
   `python scripts/gen_protocol_header.py` 重新生成 C 头（见 [protocol.md](protocol.md)）。

## scripts/

| 脚本 | 用途 |
|---|---|
| `check_config.py` | 配置完整性校验（`make check-config`，CI 必跑） |
| `gen_protocol_header.py` | 从 messages.py 生成 STM32 协议头，`--check` 防漂移 |
| `start_robot.sh` | 实机启动入口（预检 + source ROS + launch，见 deployment.md） |
