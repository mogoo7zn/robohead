"""Stream parser: half frames, sticky frames, bad CRC, garbage, unknown IDs."""
from core.protocol import (
    Frame,
    StreamParser,
    encode_frame,
    encode_payload,
    spec_by_name,
)


def make_frame(msg_name: str, seq: int = 0, **values) -> bytes:
    payload = encode_payload(msg_name, **values)
    frame = Frame(seq=seq, msg_id=spec_by_name(msg_name).msg_id, payload=payload)
    return encode_frame(frame)


def test_single_frame():
    parser = StreamParser()
    frames = parser.feed(make_frame("ODOM", x_mm=1000, y_mm=2000,
                                    theta_mrad=3000, vx_mm_s=0, vy_mm_s=0,
                                    wz_mrad_s=0))
    assert len(frames) == 1
    assert frames[0].msg_id == spec_by_name("ODOM").msg_id


def test_half_frame_reassembled():
    parser = StreamParser()
    raw = make_frame("LINE_SENSOR", line_detected=1, offset_mm=-30,
                     confidence=80)
    first, second = raw[:5], raw[5:]
    assert parser.feed(first) == []
    frames = parser.feed(second)
    assert len(frames) == 1


def test_sticky_frames():
    parser = StreamParser()
    blob = (make_frame("CMD_VEL", vx_mm_s=0, vy_mm_s=0, wz_mrad_s=0)
            + make_frame("CMD_ACTION", action_id=1, param=0, seq=1)
            + make_frame("ROBOT_STATE", motor_state=1, gripper_state=1,
                         lift_state=1, error_code=0, seq=2))
    frames = parser.feed(blob)
    assert len(frames) == 3
    assert [f.seq for f in frames] == [0, 1, 2]


def test_garbage_prefix_ignored():
    parser = StreamParser()
    garbage = b"\x00\xff\x10hello\xAA" + make_frame("CMD_VEL", vx_mm_s=0,
                                                    vy_mm_s=0, wz_mrad_s=0)
    frames = parser.feed(garbage)
    assert len(frames) == 1
    assert parser.stats.garbage_bytes >= 8


def test_bad_crc_frame_skipped():
    parser = StreamParser()
    good = make_frame("CMD_VEL", vx_mm_s=0, vy_mm_s=0, wz_mrad_s=0)
    bad = bytearray(make_frame("CMD_ACTION", action_id=1, param=0, seq=1))
    bad[-1] ^= 0xFF  # corrupt CRC
    frames = parser.feed(bytes(bad) + good)
    assert len(frames) == 1                    # good one survives
    assert parser.stats.crc_errors == 1


def test_random_garbage_never_crashes():
    import random
    rng = random.Random(42)
    parser = StreamParser()
    for _ in range(50):
        chunk = bytes(rng.randrange(256) for _ in range(rng.randrange(64)))
        parser.feed(chunk)
    # stream still usable after garbage
    frames = parser.feed(make_frame("CMD_VEL", vx_mm_s=0, vy_mm_s=0,
                                    wz_mrad_s=0))
    assert len(frames) == 1


def test_unknown_msg_id_is_counted_not_fatal():
    parser = StreamParser()
    payload = encode_payload("CMD_VEL", vx_mm_s=0, vy_mm_s=0, wz_mrad_s=0)
    frame = Frame(seq=0, msg_id=0xEE, payload=payload)  # unregistered id
    frames = parser.feed(encode_frame(frame) + make_frame("CMD_VEL", vx_mm_s=0,
                                                          vy_mm_s=0,
                                                          wz_mrad_s=0, seq=1))
    assert len(frames) == 1
    assert parser.stats.unknown_msg_ids == 1


def test_corrupted_length_resyncs():
    """A lying LEN byte desyncs the frame span -> CRC fails -> resync."""
    parser = StreamParser()
    raw = bytearray(make_frame("CMD_VEL", vx_mm_s=100, vy_mm_s=0, wz_mrad_s=0))
    # lie about length (real 6 -> claim 5): frame span 13 <= 14 real bytes,
    # so the parser runs the CRC check, fails, and resyncs on the next frame
    raw[5] = 0x05
    frames = parser.feed(bytes(raw) + make_frame("CMD_VEL", vx_mm_s=0,
                                                 vy_mm_s=0, wz_mrad_s=0, seq=1))
    assert len(frames) == 1
    assert parser.stats.crc_errors >= 1


def test_bad_version_skipped():
    parser = StreamParser()
    raw = bytearray(make_frame("CMD_VEL", vx_mm_s=0, vy_mm_s=0, wz_mrad_s=0))
    raw[2] = 0x99  # wrong version
    frames = parser.feed(bytes(raw) + make_frame("CMD_VEL", vx_mm_s=0,
                                                 vy_mm_s=0, wz_mrad_s=0, seq=1))
    assert len(frames) == 1
    assert parser.stats.version_errors == 1


def test_parser_stats_ok_counted():
    parser = StreamParser()
    parser.feed(make_frame("CMD_VEL", vx_mm_s=0, vy_mm_s=0, wz_mrad_s=0))
    parser.feed(make_frame("CMD_ACTION", action_id=1, param=0, seq=1))
    assert parser.stats.frames_ok == 2
