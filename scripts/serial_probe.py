#!/usr/bin/env python3
"""serial_probe — STM32 联调工具（协议 v1.1）.

打开串口，按 50 Hz 发送 CMD_VEL（默认 0 速，即心跳），解析并统计
下位机回传的 ODOM / LINE_SENSOR / ROBOT_STATE 帧，用于链路联调：

  1. 波特率/接线检查     —— 能否收到帧、CRC 错误率
  2. CRC 基准向量校验    —— --selftest 输出文档 §2.1 的示例帧
  3. 里程计方向校验      —— --vx 0.2 让机器人前进，观察 x_mm 增长
  4. 机构动作测试        —— --action gripper_open / gripper_close /
                             lift_up / lift_down / stop

用法（在 Pi 上）:
  python scripts/serial_probe.py --device /dev/ttyUSB0
  python scripts/serial_probe.py --device /dev/ttyUSB0 --vx 0.2 --duration 5
  python scripts/serial_probe.py --selftest
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from core.protocol import (  # noqa: E402
    ACTION_GRIPPER_CLOSE,
    ACTION_GRIPPER_OPEN,
    ACTION_LIFT_DOWN,
    ACTION_LIFT_UP,
    ACTION_STOP_ALL,
    Frame,
    StreamParser,
    decode_payload,
    encode_frame,
    encode_payload,
    spec_by_id,
    spec_by_name,
)

ACTIONS = {
    "gripper_open": ACTION_GRIPPER_OPEN,
    "gripper_close": ACTION_GRIPPER_CLOSE,
    "lift_up": ACTION_LIFT_UP,
    "lift_down": ACTION_LIFT_DOWN,
    "stop": ACTION_STOP_ALL,
}

# 文档 §2.1 基准向量：vx=+500 vy=-200 wz=+300 SEQ=7
REFERENCE_BYTES = bytes.fromhex("AA5501010706F40138FF2C01CD0B")


def selftest() -> int:
    payload = encode_payload("CMD_VEL", vx_mm_s=500, vy_mm_s=-200,
                             wz_mrad_s=300)
    frame = encode_frame(Frame(seq=0x07, msg_id=0x01, payload=payload))
    hexstr = " ".join(f"{b:02X}" for b in frame)
    print(f"generated : {hexstr}")
    print(f"reference : {' '.join(f'{b:02X}' for b in REFERENCE_BYTES)}")
    if frame == REFERENCE_BYTES:
        print("MATCH — encoder agrees with protocol doc §2.1")
        return 0
    print("MISMATCH — check core/protocol (frames/crc16/messages)")
    return 1


class ProbeStats:
    def __init__(self) -> None:
        self.counts: dict[str, int] = {}
        self.start = time.monotonic()

    def count(self, name: str) -> None:
        self.counts[name] = self.counts.get(name, 0) + 1

    def report(self) -> str:
        elapsed = max(1e-6, time.monotonic() - self.start)
        lines = [f"uptime {elapsed:6.1f}s"]
        parser_keys = {"crc_errors", "garbage_bytes", "version_errors",
                       "unknown_msg_ids", "length_errors"}
        for name, n in sorted(self.counts.items()):
            if name in parser_keys:
                lines.append(f"{name:<14} {n:>6}")
            else:
                lines.append(f"{name:<14} {n:>6}  ({n / elapsed:5.1f} Hz)")
        return " | ".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", help="串口设备，如 /dev/ttyUSB0")
    ap.add_argument("--baudrate", type=int, default=115200)
    ap.add_argument("--vx", type=float, default=0.0, help="前向速度 m/s")
    ap.add_argument("--vy", type=float, default=0.0, help="横向速度 m/s")
    ap.add_argument("--wz", type=float, default=0.0, help="角速度 rad/s")
    ap.add_argument("--action", choices=ACTIONS, help="发送一次机构动作")
    ap.add_argument("--duration", type=float, default=10.0, help="运行秒数")
    ap.add_argument("--selftest", action="store_true",
                    help="输出协议文档 §2.1 基准帧并比对")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.device:
        ap.error("--device is required (or use --selftest)")

    import serial  # pyserial

    parser = StreamParser()
    stats = ProbeStats()
    clamp = lambda v: max(-32768, min(32767, round(v * 1000)))  # noqa: E731
    vel = (clamp(args.vx), clamp(args.vy), clamp(args.wz))
    print(f"probe: {args.device} @ {args.baudrate} — CMD_VEL {vel} mm/s, "
          f"{args.duration:.0f}s")

    with serial.Serial(args.device, args.baudrate, timeout=0.01) as ser:
        seq = 0
        last_sent = 0.0
        last_report = 0.0
        deadline = time.monotonic() + args.duration
        action_sent = False

        while time.monotonic() < deadline:
            now = time.monotonic()

            # 50 Hz CMD_VEL（同时是下位机心跳）
            if now - last_sent >= 0.02:
                last_sent = now
                payload = encode_payload("CMD_VEL", vx_mm_s=vel[0],
                                         vy_mm_s=vel[1], wz_mrad_s=vel[2])
                ser.write(encode_frame(Frame(seq=seq, msg_id=0x01,
                                             payload=payload)))
                seq = (seq + 1) & 0xFF
                stats.count("cmd_vel_tx")

            if args.action and not action_sent:
                action_sent = True
                payload = encode_payload("CMD_ACTION",
                                         action_id=ACTIONS[args.action], param=0)
                ser.write(encode_frame(Frame(seq=seq, msg_id=0x02,
                                             payload=payload)))
                seq = (seq + 1) & 0xFF
                print(f"action sent: {args.action}")

            # 接收
            n = ser.in_waiting
            if n:
                for frame in parser.feed(ser.read(n)):
                    stats.count(f"rx_{spec_by_id(frame.msg_id).name}")
                    if frame.msg_id == 0x81:
                        v = decode_payload("ODOM", frame.payload)
                        print(f"ODOM  x={v['x_mm']/1000:7.3f}m "
                              f"y={v['y_mm']/1000:7.3f}m "
                              f"th={v['theta_mrad']/1000:7.3f}rad "
                              f"v=({v['vx_mm_s']}, {v['vy_mm_s']}, "
                              f"{v['wz_mrad_s']})mm/s")
                    elif frame.msg_id == 0x83:
                        v = decode_payload("ROBOT_STATE", frame.payload)
                        print(f"STATE motor={v['motor_state']} "
                              f"gripper={v['gripper_state']} "
                              f"lift={v['lift_state']} "
                              f"err={v['error_code']:#04x}")

            if now - last_report >= 2.0:
                last_report = now
                for key, val in vars(parser.stats).items():
                    if val:
                        stats.counts[key] = val
                print(f"[stats] {stats.report()}")

    print(f"[final] {stats.report()}")
    ok = stats.counts.get("rx_ODOM", 0) > 0
    crc = parser.stats.crc_errors
    if not ok:
        print("RESULT: FAIL — 没有收到 ODOM。检查接线/波特率/下位机固件。")
        return 1
    if crc:
        print(f"RESULT: WARN — 收到 ODOM 但有 {crc} 个 CRC 错误帧，"
              "检查波特率/干扰/共地。")
    print("RESULT: PASS — 链路正常。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
