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
    frames = parser.feed(make_frame("ODOMETRY", x=1.0, y=2.0, yaw=3.0,
                                    vx=0, vy=0, wz=0))
    assert len(frames) == 1
    assert frames[0].msg_id == spec_by_name("ODOMETRY").msg_id


def test_half_frame_reassembled():
    parser = StreamParser()
    raw = make_frame("IMU", yaw=0.1, gyro_z=0.0, accel_x=0.0, accel_y=0.0)
    first, second = raw[:5], raw[5:]
    assert parser.feed(first) == []
    frames = parser.feed(second)
    assert len(frames) == 1


def test_sticky_frames():
    parser = StreamParser()
    blob = (make_frame("CMD_STOP", mode=0)
            + make_frame("CMD_GRIPPER", command=1, seq=1)
            + make_frame("START_EVENT", button_state=1, seq=2))
    frames = parser.feed(blob)
    assert len(frames) == 3
    assert [f.seq for f in frames] == [0, 1, 2]


def test_garbage_prefix_ignored():
    parser = StreamParser()
    garbage = b"\x00\xff\x10hello\xAA" + make_frame("CMD_STOP", mode=0)
    frames = parser.feed(garbage)
    assert len(frames) == 1
    assert parser.stats.garbage_bytes >= 8


def test_bad_crc_frame_skipped():
    parser = StreamParser()
    good = make_frame("CMD_STOP", mode=0)
    bad = bytearray(make_frame("CMD_GRIPPER", command=1, seq=1))
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
    frames = parser.feed(make_frame("CMD_STOP", mode=0))
    assert len(frames) == 1


def test_unknown_msg_id_is_counted_not_fatal():
    parser = StreamParser()
    payload = encode_payload("CMD_STOP", mode=0)
    frame = Frame(seq=0, msg_id=0xEE, payload=payload)  # unregistered id
    frames = parser.feed(encode_frame(frame) + make_frame("CMD_STOP", mode=0, seq=1))
    assert len(frames) == 1
    assert parser.stats.unknown_msg_ids == 1


def test_corrupted_length_resyncs():
    parser = StreamParser()
    raw = bytearray(make_frame("CMD_VELOCITY", vx=1.0, vy=0.0, wz=0.0))
    raw[5] = 0xFF  # lie about length (0x0FFF > MAX? no, but mismatch)
    raw[6] = 0x0F   # len = 0x0FFF = 4095 > 256 -> length error path
    frames = parser.feed(bytes(raw) + make_frame("CMD_STOP", mode=0, seq=1))
    assert len(frames) == 1
    assert parser.stats.length_errors >= 1


def test_bad_version_skipped():
    parser = StreamParser()
    raw = bytearray(make_frame("CMD_STOP", mode=0))
    raw[2] = 0x99  # wrong version
    frames = parser.feed(bytes(raw) + make_frame("CMD_STOP", mode=0, seq=1))
    assert len(frames) == 1
    assert parser.stats.version_errors == 1


def test_parser_stats_ok_counted():
    parser = StreamParser()
    parser.feed(make_frame("CMD_STOP", mode=0))
    parser.feed(make_frame("CMD_GRIPPER", command=1, seq=1))
    assert parser.stats.frames_ok == 2
