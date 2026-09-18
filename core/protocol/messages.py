"""Protocol message definitions — SINGLE SOURCE OF TRUTH.

Matches docs/downward/robohead_transmission_protocol.md (v1.1) exactly.
The same table drives:
  * Python payload encode/decode (this file)
  * firmware/stm32/include/robogame_protocol.h (via scripts/gen_protocol_header.py)

Frame layout (little endian):
  SOF(2) VER(1) MSG_ID(1) SEQ(1) LEN(1) PAYLOAD CRC16(2, LE)
CRC-16/CCITT-FALSE over VER..PAYLOAD.

All payload fields are little-endian, struct-packed, unaligned (packed).
Units are integers: mm, mm/s, mrad, mrad/s — never floats on the wire.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

# ---------------------------------------------------------------- constants
SOF1 = 0xAA
SOF2 = 0x55
VERSION = 0x01
HEADER_SIZE = 6          # SOF(2) + VER(1) + MSG_ID(1) + SEQ(1) + LEN(1)
CRC_SIZE = 2
MAX_PAYLOAD = 255        # LEN is a single byte

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


# ---------------------------------------------------------------- enums
# CMD_ACTION action_id (protocol doc §5.2)
ACTION_GRIPPER_OPEN = 0x01     # param reserved (0)
ACTION_GRIPPER_CLOSE = 0x02    # param reserved (0)
ACTION_LIFT_UP = 0x03          # param = target height mm (0 = top limit)
ACTION_LIFT_DOWN = 0x04        # param = target height mm (0 = bottom limit)
ACTION_STOP_ALL = 0x05         # param reserved (0): stop gripper/lift, zero chassis

# ROBOT_STATE motor_state (protocol doc §6.3)
MOTOR_DISABLED = 0
MOTOR_IDLE = 1
MOTOR_RUNNING = 2
MOTOR_TIMEOUT = 3
MOTOR_FAULT = 4

# ROBOT_STATE gripper_state
GRIPPER_UNKNOWN = 0
GRIPPER_OPENED = 1
GRIPPER_CLOSED = 2
GRIPPER_HOLDING = 3
GRIPPER_FAULT = 4

# ROBOT_STATE lift_state
LIFT_UNKNOWN = 0
LIFT_IDLE = 1
LIFT_MOVING_UP = 2
LIFT_MOVING_DOWN = 3
LIFT_FAULT = 4

# ROBOT_STATE error_code bits
ERR_CHASSIS = 0x01
ERR_GRIPPER = 0x02
ERR_LIFT = 0x04
ERR_COMM_TIMEOUT = 0x08
ERR_LOW_BATTERY = 0x10


MESSAGES: dict[str, MessageSpec] = {}
_MESSAGES_BY_ID: dict[int, MessageSpec] = {}


def _register(name: str, msg_id: int, direction: str, *fields: FieldSpec) -> None:
    spec = MessageSpec(name=name, msg_id=msg_id, direction=direction, fields=fields)
    MESSAGES[name] = spec
    _MESSAGES_BY_ID[msg_id] = spec


# ---------------------------------------------------------------- Pi -> STM32
_register("CMD_VEL", 0x01, "pi_to_mcu",
          FieldSpec("vx_mm_s", "h", "body forward velocity mm/s"),
          FieldSpec("vy_mm_s", "h", "body lateral velocity mm/s (left +)"),
          FieldSpec("wz_mrad_s", "h", "yaw rate mrad/s (ccw +)"))

_register("CMD_ACTION", 0x02, "pi_to_mcu",
          FieldSpec("action_id", "B", "ACTION_* constant"),
          FieldSpec("param", "h", "action parameter (see protocol doc)"))

# ---------------------------------------------------------------- STM32 -> Pi
_register("ODOM", 0x81, "mcu_to_pi",
          FieldSpec("x_mm", "i", "accumulated position, world frame"),
          FieldSpec("y_mm", "i"),
          FieldSpec("theta_mrad", "i", "continuous heading, never wrapped"),
          FieldSpec("vx_mm_s", "h", "actual body velocity"),
          FieldSpec("vy_mm_s", "h"),
          FieldSpec("wz_mrad_s", "h"))

_register("LINE_SENSOR", 0x82, "mcu_to_pi",
          FieldSpec("line_detected", "B", "0=lost, 1=detected"),
          FieldSpec("offset_mm", "h", "lateral offset, right of travel dir +"),
          FieldSpec("confidence", "B", "0..100"))

_register("ROBOT_STATE", 0x83, "mcu_to_pi",
          FieldSpec("motor_state", "B", "MOTOR_* constant"),
          FieldSpec("gripper_state", "B", "GRIPPER_* constant"),
          FieldSpec("lift_state", "B", "LIFT_* constant"),
          FieldSpec("error_code", "B", "ERR_* bitmask"))


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
