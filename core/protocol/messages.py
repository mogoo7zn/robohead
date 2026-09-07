"""Protocol message definitions — SINGLE SOURCE OF TRUTH.

The same table drives:
  * Python payload encode/decode (this file)
  * firmware/stm32/include/robogame_protocol.h (via scripts/gen_protocol_header.py)

All payload fields are little-endian, struct-packed, unaligned (packed).
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

# ---------------------------------------------------------------- constants
SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x01
HEADER_SIZE = 7          # SOF(2) + VERSION(1) + SEQ(1) + MSG_ID(1) + LEN(2)
CRC_SIZE = 2
MAX_PAYLOAD = 256

# struct fmt -> (python type, C type, size)
_C_TYPES = {
    "B": ("uint8_t", 1),
    "b": ("int8_t", 1),
    "H": ("uint16_t", 2),
    "h": ("int16_t", 2),
    "I": ("uint32_t", 4),
    "i": ("int32_t", 4),
    "f": ("float", 4),
}


@dataclass(frozen=True)
class FieldSpec:
    name: str
    fmt: str
    comment: str = ""

    @property
    def c_type(self) -> str:
        return _C_TYPES[self.fmt][0]

    @property
    def size(self) -> int:
        return _C_TYPES[self.fmt][1]


@dataclass(frozen=True)
class MessageSpec:
    name: str
    msg_id: int
    direction: str          # "pi_to_mcu" | "mcu_to_pi"
    fields: tuple[FieldSpec, ...]

    @property
    def struct_fmt(self) -> str:
        return "<" + "".join(f.fmt for f in self.fields)

    @property
    def payload_size(self) -> int:
        return struct.calcsize(self.struct_fmt) if self.fields else 0


# Axis codes for CMD_LINEAR_AXIS / CMD_HOME
AXIS_X = 0
AXIS_Z = 1
AXIS_ALL = 0xFF

# Gripper commands
GRIPPER_OPEN = 0
GRIPPER_CLOSE = 1

# Line-follow controller states (STM32 side)
LINE_CTRL_IDLE = 0
LINE_CTRL_RUNNING = 1
LINE_CTRL_LOST = 2
LINE_CTRL_FAULT = 3


MESSAGES: dict[str, MessageSpec] = {}
_MESSAGES_BY_ID: dict[int, MessageSpec] = {}


def _register(name: str, msg_id: int, direction: str, *fields: FieldSpec) -> None:
    spec = MessageSpec(name=name, msg_id=msg_id, direction=direction, fields=fields)
    MESSAGES[name] = spec
    _MESSAGES_BY_ID[msg_id] = spec


# ---------------------------------------------------------------- Pi -> STM32
_register("HEARTBEAT", 0x01, "pi_to_mcu",
          FieldSpec("uptime_ms", "I", "Pi uptime in ms"))

_register("CMD_STOP", 0x02, "pi_to_mcu",
          FieldSpec("mode", "B", "0=soft stop, 1=brake"))

_register("CMD_VELOCITY", 0x03, "pi_to_mcu",
          FieldSpec("vx", "f", "body forward velocity m/s"),
          FieldSpec("vy", "f", "body lateral velocity m/s"),
          FieldSpec("wz", "f", "yaw rate rad/s"))

_register("CMD_LINE_FOLLOW_START", 0x04, "pi_to_mcu",
          FieldSpec("segment_id", "H", "route segment identifier"),
          FieldSpec("target_speed", "f", "m/s"))

_register("CMD_LINE_FOLLOW_STOP", 0x05, "pi_to_mcu")

_register("CMD_LINE_FOLLOW_CONFIG", 0x06, "pi_to_mcu",
          FieldSpec("kp", "f"),
          FieldSpec("ki", "f"),
          FieldSpec("kd", "f"))

_register("CMD_LINEAR_AXIS", 0x07, "pi_to_mcu",
          FieldSpec("axis", "B", "0=X, 1=Z"),
          FieldSpec("position_mm", "f", "target position"),
          FieldSpec("speed", "f", "mm/s, 0=default"))

_register("CMD_GRIPPER", 0x08, "pi_to_mcu",
          FieldSpec("command", "B", "0=open, 1=close"))

_register("CMD_HOME", 0x09, "pi_to_mcu",
          FieldSpec("axis", "B", "0=X, 1=Z, 0xFF=all"))

# ---------------------------------------------------------------- STM32 -> Pi
_register("MCU_HEARTBEAT", 0x81, "mcu_to_pi",
          FieldSpec("uptime_ms", "I"),
          FieldSpec("fault_flags", "I", "bitmask"))

_register("ODOMETRY", 0x82, "mcu_to_pi",
          FieldSpec("x", "f", "map frame, m"),
          FieldSpec("y", "f"),
          FieldSpec("yaw", "f", "rad"),
          FieldSpec("vx", "f", "body frame, m/s"),
          FieldSpec("vy", "f"),
          FieldSpec("wz", "f", "rad/s"))

_register("IMU", 0x83, "mcu_to_pi",
          FieldSpec("yaw", "f", "integrated heading rad"),
          FieldSpec("gyro_z", "f", "rad/s"),
          FieldSpec("accel_x", "f", "m/s^2"),
          FieldSpec("accel_y", "f", "m/s^2"))

_register("LINE_STATE", 0x84, "mcu_to_pi",
          FieldSpec("line_detected", "B"),
          FieldSpec("line_error", "f", "normalized lateral error"),
          FieldSpec("confidence", "f", "0..1"),
          FieldSpec("controller_state", "B", "0=idle 1=running 2=lost 3=fault"),
          FieldSpec("intersection_detected", "B"),
          FieldSpec("fault", "B"))

_register("MANIPULATOR_STATE", 0x85, "mcu_to_pi",
          FieldSpec("x_mm", "f"),
          FieldSpec("z_mm", "f"),
          FieldSpec("homed", "B"),
          FieldSpec("gripper_state", "B", "0=unknown 1=open 2=closed 3=holding 4=fault"),
          FieldSpec("grip_detected", "B"),
          FieldSpec("moving", "B"),
          FieldSpec("fault", "B"),
          FieldSpec("fault_code", "I"))

_register("LIMIT_STATE", 0x86, "mcu_to_pi",
          FieldSpec("bitmask", "I", "limit switch bitmap"))

_register("FAULT", 0x87, "mcu_to_pi",
          FieldSpec("fault_code", "I"),
          FieldSpec("detail", "I"))

_register("START_EVENT", 0x88, "mcu_to_pi",
          FieldSpec("button_state", "B", "1=pressed"))


def spec_by_id(msg_id: int) -> MessageSpec | None:
    return _MESSAGES_BY_ID.get(msg_id)


def spec_by_name(name: str) -> MessageSpec:
    if name not in MESSAGES:
        raise KeyError(f"unknown message name: {name}")
    return MESSAGES[name]


def encode_payload(name: str, **values) -> bytes:
    spec = spec_by_name(name)
    if not spec.fields:
        return b""
    ordered = []
    for f in spec.fields:
        if f.name not in values:
            raise KeyError(f"message {name}: missing field '{f.name}'")
        ordered.append(values[f.name])
    return struct.pack(spec.struct_fmt, *ordered)


def decode_payload(name: str, payload: bytes) -> dict:
    spec = spec_by_name(name)
    if not spec.fields:
        return {}
    expected = spec.payload_size
    if len(payload) != expected:
        raise ValueError(
            f"message {name}: payload size {len(payload)} != {expected}")
    unpacked = struct.unpack(spec.struct_fmt, payload)
    return {f.name: v for f, v in zip(spec.fields, unpacked)}


def payload_size(name: str) -> int:
    return spec_by_name(name).payload_size
