"""CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0).

Matches the standard implementation expected on STM32 (e.g. hardware CRC
units configured for CCITT, or the classic table-less bit loop below).
"""
from __future__ import annotations

CRC16_POLY = 0x1021
CRC16_INIT = 0xFFFF


def crc16_ccitt(data: bytes, crc: int = CRC16_INIT) -> int:
    crc &= 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC16_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc16_ccitt_bytes(data: bytes) -> bytes:
    """CRC as little-endian bytes (as appended to frames)."""
    return crc16_ccitt(data).to_bytes(2, "little")
