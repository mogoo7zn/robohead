"""End-to-end protocol tests: Pi client <-> FakeSTM32 over MemoryTransport,
and Python <-> generated C header consistency."""
import pytest

from core.hardware import McuClient, MemoryTransport
from core.mock import FakeSTM32, SimWorld
from core.protocol import MESSAGES, SOF1, SOF2, VERSION
from core.utils.clock import FakeClock
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]


def _make_link():
    host, device = MemoryTransport.create_pair()
    clock = FakeClock()
    world = SimWorld()
    client = McuClient(host, clock, heartbeat_period=0.05)
    mcu = FakeSTM32(device, world, clock)
    return client, mcu, world, clock


def test_client_fake_mcu_roundtrip():
    client, mcu, world, clock = _make_link()
    client.send_velocity(0.5, 0.0, 0.1)
    mcu.tick(0.02)
    assert world.velocity_cmd == pytest.approx((0.5, 0.0, 0.1), abs=1e-6)

    client.send_stop()
    mcu.tick(0.02)
    assert world.stopped is True

    client.send_line_follow_start(segment_id=2, target_speed=0.3)
    mcu.tick(0.02)
    assert world.line_follow_active is True

    client.send_gripper(close=True)
    mcu.tick(0.02)

    client.send_home()
    mcu.tick(0.02)
    assert world.manip_homed is True

    client.send_linear_axis(axis=1, position_mm=100.0, speed=50.0)
    mcu.tick(0.02)
    assert world.manip_moving is True
    assert world._manip_target_z == pytest.approx(100.0)


def test_telemetry_reaches_client():
    client, mcu, world, clock = _make_link()
    world.odom_x = 1.234
    world.odom_y = 5.678
    mcu.tick(0.02)
    client.poll()
    odom = client.telemetry.odom
    assert odom.x == pytest.approx(1.234, abs=1e-5)
    assert odom.y == pytest.approx(5.678, abs=1e-5)
    assert client.link_alive


def test_heartbeat_wakes_watchdog():
    client, mcu, world, clock = _make_link()
    client.send_heartbeat()
    mcu.tick(0.02)
    # simulate heartbeat loss: long advance without heartbeats
    clock.advance(1.0)
    mcu.tick(0.02)
    assert mcu.watchdog_tripped is True
    assert world.stopped is True
    # heartbeat resumes -> watchdog clears
    client.send_heartbeat()
    clock.advance(0.02)
    mcu.tick(0.02)
    assert mcu.watchdog_tripped is False


def test_start_event_edge_triggered():
    client, mcu, world, clock = _make_link()
    world.start_button = True
    mcu.tick(0.02)
    client.poll()
    assert client.telemetry.start_button is True
    # after the event is consumed it is not repeated
    mcu.tick(0.02)
    client.poll()
    assert client.telemetry.start_button is True  # sticky until reset
    # (telemetry keeps last value; the MCU emits a single edge event)


# ---------------------------------------------------------------- C header

def test_c_header_exists_and_is_current():
    header = REPO / "firmware" / "stm32" / "include" / "robogame_protocol.h"
    assert header.exists(), "run: python3 scripts/gen_protocol_header.py"
    text = header.read_text()

    # constants
    assert f"{SOF1:#04x}" in text
    assert f"{SOF2:#04x}" in text
    assert f"{VERSION:#04x}" in text

    # every message id appears with the right value
    for name, spec in MESSAGES.items():
        assert f"RG_{name} = {spec.msg_id:#04x}" in text, name
        # every field name appears in the C struct
        for f in spec.fields:
            assert f"{f.name};" in text, f"{name}.{f.name}"

    # no trailing comma on the last enum entry (C89 compilers)
    import re
    enum_body = re.search(r"typedef enum \{(.*?)\} rg_msg_id_t;", text, re.S).group(1)
    assert not re.search(r",\s*\}", enum_body)


def test_all_message_ids_unique():
    ids = [s.msg_id for s in MESSAGES.values()]
    assert len(ids) == len(set(ids))
