"""Frame encode/decode for the Pi <-> STM32 binary protocol.

Frame layout (little endian, protocol doc v1.1):

  +------+------+---------+--------+-----+--------+---------+--------+
  | SOF1 | SOF2 | VERSION | MSG_ID | SEQ | LEN(1) | PAYLOAD | CRC16  |
  | 0xAA | 0x55 |   0x01  |   1    |  1  |   1    |  LEN    | 2 (LE) |
  +------+------+---------+--------+-----+--------+---------+--------+

CRC16-CCITT-FALSE over VERSION..PAYLOAD (everything after SOF, before CRC).
"""
from __future__ import annotations

from dataclasses import dataclass

from core.protocol.crc16 import crc16_ccitt
from core.protocol.messages import (
    CRC_SIZE,
    HEADER_SIZE,
    MAX_PAYLOAD,
    SOF1,
    SOF2,
    VERSION,
)


class ProtocolError(Exception):
    pass


@dataclass
class Frame:
    seq: int
    msg_id: int
    payload: bytes = b""
    version: int = VERSION


def encode_frame(frame: Frame) -> bytes:
    if len(frame.payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload too large: {len(frame.payload)}")
    header = bytes([SOF1, SOF2, frame.version, frame.msg_id,
                    frame.seq & 0xFF, len(frame.payload)])
    body = header[2:] + frame.payload   # VERSION..PAYLOAD
    crc = crc16_ccitt(body)
    return header + frame.payload + crc.to_bytes(2, "little")


def try_decode_frame(buf: bytes, offset: int = 0) -> tuple[Frame, int] | None:
    """Try to decode one frame at buf[offset:].

    Returns (frame, total_bytes_consumed) or None if more data is needed.
    Raises ProtocolError on definite corruption (bad CRC / version / len).
    """
    n = len(buf) - offset
    if n < HEADER_SIZE:
        return None
    if buf[offset] != SOF1 or buf[offset + 1] != SOF2:
        raise ProtocolError("SOF mismatch")
    version = buf[offset + 2]
    if version != VERSION:
        raise ProtocolError(f"unsupported version {version}")
    msg_id = buf[offset + 3]
    seq = buf[offset + 4]
    length = buf[offset + 5]
    if length > MAX_PAYLOAD:
        raise ProtocolError(f"length {length} > {MAX_PAYLOAD}")
    total = HEADER_SIZE + length + CRC_SIZE
    if n < total:
        return None  # half frame — need more bytes
    payload = bytes(buf[offset + HEADER_SIZE: offset + HEADER_SIZE + length])
    crc_rx = buf[offset + total - 2] | (buf[offset + total - 1] << 8)
    crc_calc = crc16_ccitt(buf[offset + 2: offset + HEADER_SIZE + length])
    if crc_rx != crc_calc:
        raise ProtocolError(f"CRC mismatch (rx {crc_rx:#06x} != calc {crc_calc:#06x})")
    return Frame(seq=seq, msg_id=msg_id, payload=payload, version=version), total
