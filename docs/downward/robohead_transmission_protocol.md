# RoboGame 上下位机通信协议与接口合约

版本：v1.1（定稿）
日期：2026-09-18

## 1. 通信方案

- 上位机：Raspberry Pi 5
- 下位机：STM32
- 物理链路：USB 转 UART
- 波特率：115200
- 串口格式：8N1
  - 8 data bits
  - No parity
  - 1 stop bit
- 多字节数据：Little Endian（低字节在前）
- 数据格式：自定义二进制帧
- 不使用 JSON
- 不直接传输 ROS 2 消息

---

## 2. 帧格式

```text
| 0xAA | 0x55 | VER | MSG_ID | SEQ | LEN | PAYLOAD | CRC16 |
```

| 字段 | 长度 | 含义 |
|---|---:|---|
| Header | 2 B | 固定为 `0xAA 0x55` |
| VER | 1 B | 协议版本，当前固定 `0x01`；接收方应丢弃其他版本的帧 |
| MSG_ID | 1 B | 消息类型（见第 4/5 节编号表） |
| SEQ | 1 B | 包序号，发送方每发一帧自增 1，溢出回绕（0→255→0）；仅用于诊断与丢包统计，**不要求应答** |
| LEN | 1 B | Payload 字节长度（0~255） |
| PAYLOAD | LEN B | 实际数据 |
| CRC16 | 2 B | 校验值，Little Endian（见第 3 节） |

帧头固定 6 字节，整帧长度 = 6 + LEN + 2。

### 2.1 示例帧（CMD_VEL，调试用基准向量）

vx = +500 mm/s，vy = -200 mm/s，wz = +300 mrad/s，SEQ = 0x07：

```text
AA 55 01 01 07 06 F4 01 38 FF 2C 01 CD 0B
│  │  │  │  │  │  └─ payload (6 B) ─┘  │└─
│  │  │  │  │  │                        └ CRC16 = 0x0BCD (LE: CD 0B)
│  │  │  │  │  └ LEN = 6
│  │  │  │  └ SEQ = 0x07
│  │  │  └ MSG_ID = 0x01 (CMD_VEL)
│  │  └ VER = 0x01
│  └ SOF2
└ SOF1
```

双方联调第一件事：互发此帧比对 CRC。

---

## 3. CRC16 定义（必须双方完全一致）

| 参数 | 值 |
|---|---|
| 算法名 | CRC-16/CCITT-FALSE |
| 多项式 | 0x1021 |
| 初始值 | 0xFFFF |
| 输入字节反转 | 否 |
| 输出位反转 | 否 |
| 结果异或 | 0x0000（不异或） |

- **覆盖范围**：从 `VER` 到 `PAYLOAD` 的最后一个字节，即 SOF 之后、CRC 之前的全部字节（不含 `0xAA 0x55`，不含 CRC 自身）
- **发送字节序**：Little Endian（低字节在前）

C 参考实现（STM32 可直接使用，或配置硬件 CRC 外设：Poly=0x1021，Init=0xFFFF，无反转）：

```c
uint16_t rg_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}
```

Python 参考实现（上位机，见 `core/protocol/crc16.py`）：

```python
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc
```

---

## 4. 数据单位

通信层统一使用整数，不使用 `float`。

| 数据 | 单位 | 类型 |
|---|---|---|
| 位置 | mm | `int32` |
| 线速度 | mm/s | `int16` |
| 角度 | mrad | `int32` |
| 角速度 | mrad/s | `int16` |

示例：

```text
0.5 m/s → 500 mm/s
-0.2 m/s → -200 mm/s
0.3 rad/s → 300 mrad/s
```

**角度连续性约定**：ODOM 中的 `theta_mrad` 为**连续累计角，不做 ±π 回绕**（转过 +179° 后继续增长到 +180°、+181°，而不是跳回 -179°）。`int32` 可容纳约 ±2147483 rad（约 ±34 万圈），不会溢出。

---

## 5. 上位机 → 下位机

### 5.1 CMD_VEL

**MSG_ID：`0x01`**

用途：发送底盘目标速度（车体系 body frame：x 前、y 左、wz 逆时针为正）。

```c
int16 vx_mm_s;
int16 vy_mm_s;
int16 wz_mrad_s;
```

| 字段 | 含义 |
|---|---|
| vx_mm_s | 前后方向速度（前为正） |
| vy_mm_s | 横向速度（左为正） |
| wz_mrad_s | 旋转角速度（逆时针为正） |

发送频率：

```text
50 Hz
```

下位机收到后解算为四麦轮转速并闭环执行。

### 5.2 CMD_ACTION

**MSG_ID：`0x02`**

用途：发送机构动作命令。

```c
uint8 action_id;
int16 param;
```

`action_id` 编号表（双方必须一致）：

| action_id | 动作 | param 含义 |
|---|---|---|
| `0x01` | 夹爪打开 | 保留，填 0 |
| `0x02` | 夹爪关闭 | 保留，填 0 |
| `0x03` | 丝杆上升 | 目标高度 mm（相对丝杆下限零点）；0 = 上升到机械上限 |
| `0x04` | 丝杆下降 | 目标高度 mm；0 = 下降到机械下限 |
| `0x05` | 全部停止 | 保留，填 0；夹爪/丝杆立即停止，底盘目标速度置零 |

该消息采用事件触发方式发送（非周期）。动作完成情况通过 ROBOT_STATE（`0x83`）回显，上位机据此判断动作是否完成，不需要 ACK 帧。

---

## 6. 下位机 → 上位机

### 6.1 ODOM

**MSG_ID：`0x81`**

用途：返回底盘里程计与当前速度。

```c
int32 x_mm;
int32 y_mm;
int32 theta_mrad;

int16 vx_mm_s;
int16 vy_mm_s;
int16 wz_mrad_s;
```

| 字段 | 含义 |
|---|---|
| x_mm, y_mm | 里程计累计位置（开机为零点，世界系） |
| theta_mrad | 连续累计航向角，**不回绕**（见第 4 节） |
| vx/vy/wz | 当前实际速度（车体系） |

发送频率：

```text
50 Hz
```

### 6.2 LINE_SENSOR

**MSG_ID：`0x82`**

用途：返回巡线模块检测结果。

```c
uint8 line_detected;
int16 offset_mm;
uint8 confidence;
```

| 字段 | 含义 |
|---|---|
| line_detected | 0 = 丢线，1 = 检测到线 |
| offset_mm | 横向偏差，机器人偏在线行进方向**右侧为正**（丢线时仍上报最近一次有效估计） |
| confidence | 置信度 0~100 |

发送频率：

```text
50 Hz
```

> **草案说明**：此 Payload 为当前草案（与上位机约定）。若巡线模块最终输出格式不同，双方协商后修订本节，MSG_ID 不变。

### 6.3 ROBOT_STATE

**MSG_ID：`0x83`**

用途：返回机器人执行机构与故障状态。

```c
uint8 motor_state;
uint8 gripper_state;
uint8 lift_state;
uint8 error_code;
```

发送频率：

```text
10 Hz
```

`motor_state` 编号表：

| 值 | 含义 |
|---|---|
| 0 | DISABLED — 电机未使能 |
| 1 | IDLE — 使能、静止（目标速度为 0） |
| 2 | RUNNING — 正在执行速度指令 |
| 3 | TIMEOUT — 通信超时保护中（速度已置零，见第 7 节） |
| 4 | FAULT — 底盘电机故障 |

`gripper_state` 编号表：

| 值 | 含义 |
|---|---|
| 0 | UNKNOWN — 上电初始/未知 |
| 1 | OPEN — 已打开到位 |
| 2 | CLOSED — 已关闭（未夹到物） |
| 3 | HOLDING — 夹持中（夹到物体） |
| 4 | FAULT — 夹爪故障 |

`lift_state` 编号表：

| 值 | 含义 |
|---|---|
| 0 | UNKNOWN — 上电初始/未知 |
| 1 | IDLE — 静止（含到位后） |
| 2 | MOVING_UP — 丝杆上升中 |
| 3 | MOVING_DOWN — 丝杆下降中 |
| 4 | FAULT — 丝杆故障 |

`error_code` 位定义（多故障按位或）：

| 位 | 值 | 含义 |
|---|---|---|
| bit0 | 0x01 | 底盘电机故障 |
| bit1 | 0x02 | 夹爪故障 |
| bit2 | 0x04 | 丝杆故障 |
| bit3 | 0x08 | 通信超时（已触发保护，恢复通信后清零） |
| bit4 | 0x10 | 低压报警 |
| — | 0x00 | 无故障 |

---

## 7. 通信安全机制

STM32 必须实现通信超时保护：

```text
超过 200 ms 未收到新的 CMD_VEL
→ 底盘目标速度立即置零（电机抱闸或缓停，按实现而定）
→ motor_state 置为 TIMEOUT (3)，error_code 置 bit3
→ 恢复收到 CMD_VEL 后清除（error_code bit3 同步清除）
```

CRC16 校验失败、VER 不为 0x01 的数据帧直接丢弃，不做任何处理。

上位机侧链路监测（上位机自行实现，不涉下位机）：

```text
超过 500 ms 未收到任何下位机帧（ODOM 50 Hz 即为心跳）
→ 判定链路断开，任务层进入安全停机流程
```

---

## 8. ROS 2 接口边界

ROS 2 仅运行在树莓派侧。

```text
ROS 2
  ↓
stm32_bridge（MissionStack 内的 McuClient + McuChassis/McuManipulator）
  ↓
二进制串口协议
  ↓
STM32
```

桥接层负责：

```text
ROS 2 消息
↕
单位转换（m ↔ mm，rad ↔ mrad，float ↔ int）
↕
二进制串口帧
```

ROS 2 内部使用标准 SI 单位：

```text
m
m/s
rad
rad/s
```

串口通信统一转换为：

```text
mm
mm/s
mrad
mrad/s
```

比赛开始信号不经过串口协议：上位机通过 ROS 2 服务（`~/start_match`）启动任务。

---

## 9. 预留与扩展

以下 MSG_ID 段预留，双方协商一致后方可启用，未约定前接收方应丢弃：

```text
0x03 ~ 0x7F   上位机 → 下位机 扩展段
0x84 ~ 0xBF   下位机 → 上位机 扩展段
```

---

## 10. 变更记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.0 | — | 初版：帧格式、5 条消息、单位约定 |
| v1.1 | 2026-09-18 | 定稿：明确 CRC-16/CCITT-FALSE 参数与覆盖范围；补 action_id / motor_state / gripper_state / lift_state / error_code 编号表；明确 theta 连续累计不回绕；LINE_SENSOR 草案 payload；上位机链路监测 500 ms；预留扩展 MSG_ID 段 |
