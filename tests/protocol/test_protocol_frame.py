"""Frame encode/decode round trips and header layout checks."""
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
    payload = encode_payload("CMD_VELOCITY", vx=0.5, vy=-0.1, wz=0.2)
    frame = Frame(seq=7, msg_id=spec_by_name("CMD_VELOCITY").msg_id, payload=payload)
    raw = encode_frame(frame)
    assert raw[0] == SOF1 and raw[1] == SOF2
    assert raw[2] == VERSION
    assert raw[3] == 7                      # seq
    assert raw[4] == 0x03                   # CMD_VELOCITY id
    length = raw[5] | (raw[6] << 8)
    assert length == 12                     # 3 x float32
    assert len(raw) == HEADER_SIZE + 12 + 2


def test_roundtrip_all_messages():
    samples = {
        "HEARTBEAT": {"uptime_ms": 123456},
        "CMD_STOP": {"mode": 1},
        "CMD_VELOCITY": {"vx": 0.4, "vy": -0.2, "wz": 1.1},
        "CMD_LINE_FOLLOW_START": {"segment_id": 3, "target_speed": 0.35},
        "CMD_LINE_FOLLOW_STOP": {},
        "CMD_LINE_FOLLOW_CONFIG": {"kp": 2.0, "ki": 0.0, "kd": 0.05},
        "CMD_LINEAR_AXIS": {"axis": 1, "position_mm": 120.5, "speed": 40.0},
        "CMD_GRIPPER": {"command": 1},
        "CMD_HOME": {"axis": 0xFF},
        "MCU_HEARTBEAT": {"uptime_ms": 999, "fault_flags": 0},
        "ODOMETRY": {"x": 1.5, "y": 0.2, "yaw": 3.0, "vx": 0.1, "vy": 0.0, "wz": -0.5},
        "IMU": {"yaw": 0.1, "gyro_z": 0.02, "accel_x": 0.0, "accel_y": 0.01},
        "LINE_STATE": {"line_detected": 1, "line_error": 0.02, "confidence": 0.9,
                       "controller_state": 1, "intersection_detected": 0, "fault": 0},
        "MANIPULATOR_STATE": {"x_mm": 150.0, "z_mm": 30.0, "homed": 1,
                               "gripper_state": 3, "grip_detected": 1,
                               "moving": 0, "fault": 0, "fault_code": 0},
        "LIMIT_STATE": {"bitmask": 0x00000005},
        "FAULT": {"fault_code": 42, "detail": 7},
        "START_EVENT": {"button_state": 1},
    }
    for name, values in samples.items():
        payload = encode_payload(name, **values)
        spec = spec_by_name(name)
        assert len(payload) == spec.payload_size, name
        decoded = decode_payload(name, payload)
        for key, value in values.items():
            assert decoded[key] == pytest.approx(value), f"{name}.{key}"


def test_encode_missing_field_raises():
    with pytest.raises(KeyError):
        encode_payload("CMD_VELOCITY", vx=1.0)  # vy / wz missing


def test_decode_wrong_size_raises():
    with pytest.raises(ValueError):
        decode_payload("CMD_VELOCITY", b"\x00" * 5)


def test_payload_too_large_rejected():
    frame = Frame(seq=0, msg_id=0x01, payload=b"\x00" * 300)
    with pytest.raises(ProtocolError):
        encode_frame(frame)
