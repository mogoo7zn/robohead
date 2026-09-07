# 树莓派 ↔ STM32 通信协议

## 帧格式

小端（little-endian）、struct-packed、无对齐填充：

```
┌──────┬──────┬─────┬─────┬────────┬─────────┬───────────┬───────────────┐
│ SOF1 │ SOF2 │ VER │ SEQ │ MSG_ID │ LEN(u16)│  PAYLOAD  │ CRC16(u16 LE) │
│ 0xAA │ 0x55 │ 0x01│ u8  │  u8    │  ≤256   │  0..N 字节 │               │
└──────┴──────┴─────┴─────┴────────┴─────────┴───────────┴───────────────┘
 头部 7 字节                                        CRC 覆盖范围：VER..PAYLOAD
```

- **CRC16-CCITT**：多项式 `0x1021`，初值 `0xFFFF`，覆盖 `VER` 到 `PAYLOAD`（不含 SOF/SEQ）。
- 解析器（`core/protocol/stream.py`）逐字节扫描 SOF，坏帧自动重同步，丢帧不影响后续。

### 帧示例

一条真实编码的 `HEARTBEAT`（uptime_ms=1234，seq=1），可用它核对串口抓包与固件解析：

```
aa 55 01 01 01 04 00 d2 04 00 00 e9 93 13
```

| 字节 | 值 | 含义 |
|---|---|---|
| `aa 55` | 0xAA 0x55 | 帧头 SOF1/SOF2 |
| `01` | 1 | VER |
| `01` | 1 | SEQ |
| `01` | 0x01 | MSG_ID = HEARTBEAT |
| `04 00` | 4 | LEN（u16 LE） |
| `d2 04 00 00` | 1234 | PAYLOAD：uptime_ms（u32 LE） |
| `e9 13` | 0x1393 | CRC16（u16 LE，覆盖 VER..PAYLOAD） |

## 传输参数

| 参数 | 值 | 配置位置 |
|---|---|---|
| 物理链路 | UART 串口（Pi 上 `/dev/serial0`） | `config/hardware.yaml` → `mcu.serial` |
| 波特率 | 115200 | `mcu.serial.baudrate` |
| Pi → MCU 心跳 | 每 50ms 一条 `HEARTBEAT` | `mcu.heartbeat_period` |
| MCU 侧看门狗 | 200ms 收不到心跳 → 零速停车 | `mcu.heartbeat_timeout` |
| Pi 侧超时 | 1s 无 MCU 遥测 → 硬件故障（Watchdog） | `watchdog.high_level_timeout` |

Mac 模拟不走串口：Transport 是内存队列（`memory`），协议字节流完全一致。

## 消息表（17 条）

### Pi → STM32（0x01–0x09）

| MSG_ID | 名称 | 字段 | 用途 |
|---|---|---|---|
| 0x01 | `HEARTBEAT` | uptime_ms | 心跳，每 50ms；STM32 超 200ms 未收到即零速停车 |
| 0x02 | `CMD_STOP` | mode(0=软停 1=刹车) | 停车 |
| 0x03 | `CMD_VELOCITY` | vx, vy, wz (float) | 底盘速度（机体系 m/s、rad/s） |
| 0x04 | `CMD_LINE_FOLLOW_START` | segment_id, target_speed | 让 STM32 开始巡某段线 |
| 0x05 | `CMD_LINE_FOLLOW_STOP` | — | 停止巡线 |
| 0x06 | `CMD_LINE_FOLLOW_CONFIG` | kp, ki, kd | 下发巡线 PID 参数 |
| 0x07 | `CMD_LINEAR_AXIS` | axis(0=X 1=Z), position_mm, speed | 机械臂轴运动 |
| 0x08 | `CMD_GRIPPER` | command(0=开 1=合) | 夹爪 |
| 0x09 | `CMD_HOME` | axis(0xFF=全部) | 轴回零 |

### STM32 → Pi（0x81–0x88）

| MSG_ID | 名称 | 字段 | 用途 |
|---|---|---|---|
| 0x81 | `MCU_HEARTBEAT` | uptime_ms, fault_flags | MCU 心跳 + 故障位图 |
| 0x82 | `ODOMETRY` | x, y, yaw, vx, vy, wz | 里程计（MCU 自身 dead-reckon 坐标系） |
| 0x83 | `IMU` | yaw, gyro_z, accel_x, accel_y | 积分航向与角速度 |
| 0x84 | `LINE_STATE` | line_detected, line_error, confidence, controller_state, intersection_detected, fault | 巡线状态（0=idle 1=running 2=lost 3=fault） |
| 0x85 | `MANIPULATOR_STATE` | x_mm, z_mm, homed, gripper_state, grip_detected, moving, fault, fault_code | 机械臂状态（夹爪 0=未知 1=开 2=合 3=夹住 4=故障） |
| 0x86 | `LIMIT_STATE` | bitmask | 限位开关位图 |
| 0x87 | `FAULT` | fault_code, detail | 故障上报 |
| 0x88 | `START_EVENT` | button_state | 物理启动按钮（比赛开始信号） |

## 单一真源与代码生成

协议的唯一真源是 `core/protocol/messages.py` 的 `MESSAGES` 表，
C 头文件由脚本自动生成，**不要手改**：

```bash
python scripts/gen_protocol_header.py           # 重新生成 firmware/stm32/include/robogame_protocol.h
python scripts/gen_protocol_header.py --check   # 校验是否漂移（CI 会跑）
```

修改协议的流程：改 `messages.py` → 重新生成头文件 → STM32 固件包含新头同步编译。
两侧字段定义永远一致，杜绝"手抄错一个偏移"这类问题。

## 关联代码

| 位置 | 职责 |
|---|---|
| `core/protocol/messages.py` | 消息表 + payload 编解码（struct 打包） |
| `core/protocol/frames.py` | 帧编解码（SOF/VER/LEN/CRC 校验） |
| `core/protocol/stream.py` | 字节流解析器（重同步） |
| `core/protocol/crc16.py` | CRC16-CCITT 实现 |
| `core/hardware/mcu_client.py` | 面向业务的心跳/命令/遥测客户端 |
| `core/mock/fake_stm32.py` | 协议级 STM32 仿真（mock 用） |
| `firmware/stm32/include/robogame_protocol.h` | 生成给固件的 C 头 |
