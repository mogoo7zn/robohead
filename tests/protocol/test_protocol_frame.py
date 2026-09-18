"""Frame encode/decode round trips and header layout checks (protocol v1.1)."""
import pytest

from core.protocol import (
    Frame,
    ProtocolError,
    decode_payload,
    encode_frame,
    encode_payload,
    spec_by_name,
)
from core.protocol.messages import HEADER_SIZE, SOF1, SOF2, VERSION


def test_frame_layout():
    payload = encode_payload("CMD_VEL", vx_mm_s=500, vy_mm_s=-200,
                             wz_mrad_s=300)
    frame = Frame(seq=7, msg_id=spec_by_name("CMD_VEL").msg_id, payload=payload)
    raw = encode_frame(frame)
    assert raw[0] == SOF1 and raw[1] == SOF2
    assert raw[2] == VERSION
    assert raw[3] == 0x01                   # CMD_VEL msg id
    assert raw[4] == 7                      # seq
    assert raw[5] == 6                      # LEN: 3 x int16
    assert len(raw) == HEADER_SIZE + 6 + 2


def test_reference_frame_from_doc():
    """Protocol doc §2.1 golden vector: locks frame layout + CRC + units."""
    payload = encode_payload("CMD_VEL", vx_mm_s=500, vy_mm_s=-200,
                             wz_mrad_s=300)
    frame = encode_frame(Frame(seq=0x07, msg_id=0x01, payload=payload))
    assert frame == bytes.fromhex("AA5501010706F40138FF2C01CD0B")


def test_roundtrip_all_messages():
    samples = {
        "CMD_VEL": {"vx_mm_s": 400, "vy_mm_s": -200, "wz_mrad_s": 300},
        "CMD_ACTION": {"action_id": 0x03, "param": 120},
        "ODOM": {"x_mm": 1500, "y_mm": -200, "theta_mrad": 3141,
                 "vx_mm_s": 100, "vy_mm_s": 0, "wz_mrad_s": -500},
        "LINE_SENSOR": {"line_detected": 1, "offset_mm": -25,
                        "confidence": 90},
        "ROBOT_STATE": {"motor_state": 2, "gripper_state": 3,
                        "lift_state": 1, "error_code": 0x08},
    }
    for name, values in samples.items():
        payload = encode_payload(name, **values)
        spec = spec_by_name(name)
        assert len(payload) == spec.payload_size, name
        decoded = decode_payload(name, payload)
        for key, value in values.items():
            assert decoded[key] == value, f"{name}.{key}"


def test_encode_missing_field_raises():
    with pytest.raises(KeyError):
        encode_payload("CMD_VEL", vx_mm_s=1)  # vy / wz missing


def test_decode_wrong_size_raises():
    with pytest.raises(ValueError):
        decode_payload("CMD_VEL", b"\x00" * 5)


def test_payload_too_large_rejected():
    frame = Frame(seq=0, msg_id=0x01, payload=b"\x00" * 300)
    with pytest.raises(ProtocolError):
        encode_frame(frame)
