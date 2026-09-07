"""CRC-16/CCITT-FALSE tests against known check values."""
from core.protocol.crc16 import crc16_ccitt, crc16_ccitt_bytes


def test_empty_payload():
    # CRC-16/CCITT-FALSE of empty data is the init value
    assert crc16_ccitt(b"") == 0xFFFF


def test_known_vector():
    # Standard check value for "123456789" is 0x29B1
    assert crc16_ccitt(b"123456789") == 0x29B1


def test_crc_changes_with_data():
    a = crc16_ccitt(b"\x01\x02")
    b = crc16_ccitt(b"\x02\x01")
    assert a != b


def test_crc_bytes_little_endian():
    raw = crc16_ccitt_bytes(b"123456789")
    assert raw == bytes([0xB1, 0x29])  # LE of 0x29B1
    assert raw[0] | (raw[1] << 8) == 0x29B1


def test_init_parameter_changes_result():
    # custom seed differs from standard init
    assert crc16_ccitt(b"hello", 0x0000) != crc16_ccitt(b"hello")


def test_deterministic():
    assert crc16_ccitt(b"robogame") == crc16_ccitt(b"robogame")
