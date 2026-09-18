"""End-to-end protocol tests: Pi client <-> FakeSTM32 over MemoryTransport,
and Python <-> generated C header consistency."""
import math

import pytest

from core.hardware import McuClient, MemoryTransport
from core.mock import FakeSTM32, LineSegment, SimWorld
from core.model.pose import Pose2D
from core.protocol import (
    ACTION_GRIPPER_CLOSE,
    ACTION_LIFT_UP,
    MESSAGES,
    MOTOR_TIMEOUT,
    SOF1,
    SOF2,
    VERSION,
)
from core.utils.clock import FakeClock
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def _make_link():
    host, device = MemoryTransport.create_pair()
    clock = FakeClock()
    world = SimWorld()
    client = McuClient(host, clock)
    mcu = FakeSTM32(device, world, clock)
    return client, mcu, world, clock


def test_client_fake_mcu_roundtrip():
    client, mcu, world, clock = _make_link()
    client.set_velocity(0.5, 0.0, 0.1)
    mcu.tick(0.02)
    assert world.velocity_cmd == pytest.approx((0.5, 0.0, 0.1), abs=1e-6)
    assert world.stopped is False

    client.set_velocity(0.0, 0.0, 0.0)
    mcu.tick(0.02)
    assert world.stopped is True

    client.send_gripper(close=True)
    mcu.tick(0.02)

    client.send_action(ACTION_LIFT_UP, 100)
    mcu.tick(0.02)
    assert world.manip_moving is True
    assert world._manip_target_z == pytest.approx(100.0)


def test_velocity_unit_conversion_and_clamp():
    """SI floats in, integers on the wire, SI floats back out."""
    client, mcu, world, clock = _make_link()
    client.set_velocity(0.123, -0.456, 0.789)
    frames = mcu._parser.feed(mcu._transport.read())
    values = None
    from core.protocol import decode_payload, spec_by_id
    for f in frames:
        spec = spec_by_id(f.msg_id)
        if spec and spec.name == "CMD_VEL":
            values = decode_payload("CMD_VEL", f.payload)
    assert values == {"vx_mm_s": 123, "vy_mm_s": -456, "wz_mrad_s": 789}

    # clamp: 100 m/s far beyond int16 -> saturated, never struct.error
    client.set_velocity(100.0, -100.0, 100.0)
    mcu.tick(0.02)   # must not raise


def test_telemetry_reaches_client():
    client, mcu, world, clock = _make_link()
    world.odom_x = 1.234
    world.odom_y = 5.678
    world.odom_yaw = 4.0          # > pi: continuous heading must survive
    mcu.tick(0.02)
    client.poll()
    odom = client.telemetry.odom
    assert odom.x == pytest.approx(1.234, abs=1e-5)
    assert odom.y == pytest.approx(5.678, abs=1e-5)
    assert odom.yaw == pytest.approx(4.0, abs=1e-5)   # not wrapped
    assert client.link_alive


def test_cmd_vel_watchdog_trips_and_recovers():
    """Protocol doc §7: 200 ms without CMD_VEL -> zero velocity."""
    client, mcu, world, clock = _make_link()
    client.set_velocity(0.4, 0.0, 0.0)
    mcu.tick(0.02)
    assert world.velocity_cmd[0] == pytest.approx(0.4)

    clock.advance(1.0)              # no CMD_VEL for 1 s
    mcu.tick(0.02)
    assert mcu.watchdog_tripped is True
    assert world.stopped is True
    assert world.velocity_cmd == (0.0, 0.0, 0.0)

    client.set_velocity(0.2, 0.0, 0.0)   # stream resumes
    clock.advance(0.02)
    mcu.tick(0.02)
    assert mcu.watchdog_tripped is False
    assert world.velocity_cmd[0] == pytest.approx(0.2)

    # TIMEOUT state + ERR_COMM_TIMEOUT bit reported to the Pi
    clock.advance(1.0)
    mcu.tick(0.02)
    client.poll()
    assert client.telemetry.state.motor_state == MOTOR_TIMEOUT
    assert client.telemetry.state.comm_timeout is True


def test_line_sensor_and_robot_state_telemetry():
    client, mcu, world, clock = _make_link()
    # Real geometry instead of poking sensor fields by hand (the sensor
    # model in world.step() overwrites them): a line along x=0 from
    # y=-1 to y=1, robot 25 mm right of it heading +y (along the line)
    # -> positive lateral offset, detected, confidence from half-width.
    world.line_segments = [LineSegment(0.0, -1.0, 0.0, 1.0)]
    world.robot_pose = Pose2D(0.025, 0.0, math.pi / 2)
    mcu.tick(0.02)
    client.poll()
    line = client.telemetry.line
    assert line.line_detected is True
    assert line.offset_m == pytest.approx(0.025, abs=1e-6)
    assert line.confidence == pytest.approx(1.0 - 0.025 / 0.06, abs=0.011)
    state = client.telemetry.state
    assert state.motor_state is not None
    assert state.gripper_state >= 0


# ---------------------------------------------------------------- C header

def test_c_header_exists_and_is_current():
    header = REPO / "firmware" / "stm32" / "include" / "robogame_protocol.h"
    assert header.exists(), "run: python3 scripts/gen_protocol_header.py"
    text = header.read_text()

    # constants
    assert f"{SOF1:#04x}" in text
    assert f"{SOF2:#04x}" in text
    assert f"{VERSION:#04x}" in text
    assert "RG_HEADER_SIZE     6u" in text

    # every message id appears with the right value
    for name, spec in MESSAGES.items():
        assert f"RG_{name} = {spec.msg_id:#04x}" in text, name
        # every field name appears in the C struct
        for f in spec.fields:
            assert f"{f.name};" in text, f"{name}.{f.name}"

    # action / state enums are exported for the firmware
    for define in ("RG_ACTION_GRIPPER_OPEN", "RG_ACTION_STOP_ALL",
                   "RG_MOTOR_TIMEOUT", "RG_GRIPPER_HOLDING",
                   "RG_LIFT_MOVING_UP", "RG_ERR_COMM_TIMEOUT"):
        assert define in text, define

    # no trailing comma on the last enum entry (C89 compilers)
    import re
    enum_body = re.search(r"typedef enum \{(.*?)\} rg_msg_id_t;", text, re.S).group(1)
    assert not re.search(r",\s*\}", enum_body)


def test_all_message_ids_unique():
    ids = [s.msg_id for s in MESSAGES.values()]
    assert len(ids) == len(set(ids))
