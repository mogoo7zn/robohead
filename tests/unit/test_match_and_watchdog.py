"""MatchManager and Watchdog tests (all on FakeClock — never real waits)."""
import pytest

from core.model.enums import MatchPhase
from core.mission.match_manager import MatchManager
from core.mission.watchdog import Watchdog, WatchdogVerdict
from core.state.store import WorldStateStore
from core.utils.clock import FakeClock


# ---------------------------------------------------------------- match
def test_match_phases():
    clock = FakeClock()
    store = WorldStateStore()
    mm = MatchManager(store, clock, total_time=360.0,
                      normal_end_time=240.0, safe_end_time=320.0)
    mm.start()
    assert mm.phase is MatchPhase.NORMAL

    clock.advance(241.0)
    mm.tick()
    assert mm.phase is MatchPhase.SAFE
    assert store.snapshot().match.phase is MatchPhase.SAFE

    clock.advance(80.0)
    mm.tick()
    assert mm.phase is MatchPhase.ENDGAME
    assert mm.remaining == pytest.approx(360.0 - 321.0)


def test_match_finishes_at_time_up():
    clock = FakeClock()
    store = WorldStateStore()
    mm = MatchManager(store, clock)
    mm.start()
    clock.advance(360.5)
    mm.tick()
    assert mm.finished is True
    assert store.snapshot().match.finished is True
    # tick after finish is a no-op
    mm.tick()
    assert store.snapshot().match.elapsed_time == pytest.approx(360.5)


def test_match_not_started_reports_zero():
    clock = FakeClock()
    mm = MatchManager(WorldStateStore(), clock)
    assert mm.elapsed == 0.0
    assert mm.remaining == 360.0
    assert mm.started is False


def test_match_tick_updates_worldstate():
    clock = FakeClock()
    store = WorldStateStore()
    mm = MatchManager(store, clock)
    mm.start()
    clock.advance(5.0)
    mm.tick()
    snap = store.snapshot()
    assert snap.match.elapsed_time == 5.0
    assert snap.match.remaining_time == 355.0
    assert snap.match.started is True


# ---------------------------------------------------------------- watchdog
def _watchdog(clock, store, cfg=None):
    return Watchdog(store, clock, cfg or {"watchdog": {
        "high_level_timeout": 1.0, "camera_timeout": 5.0,
        "localization_timeout": 10.0, "imu_timeout": 1.0, "odom_timeout": 1.0}})


def test_watchdog_ok_when_everything_fresh():
    clock = FakeClock()
    store = WorldStateStore()
    wd = _watchdog(clock, store)
    wd.observe_mcu()
    wd.observe_localization_camera()
    wd.observe_block_camera()
    wd.observe_localization()
    wd.observe_imu()
    wd.observe_odom()
    assert wd.tick() is WatchdogVerdict.OK
    snap = store.snapshot()
    assert snap.hardware.mcu_ok is True
    assert snap.hardware.camera_localization_ok is True


def test_watchdog_mcu_loss_is_safe_stop():
    clock = FakeClock()
    store = WorldStateStore()
    wd = _watchdog(clock, store)
    wd.observe_mcu()
    clock.advance(2.0)
    assert wd.tick() is WatchdogVerdict.SAFE_STOP
    assert store.snapshot().hardware.mcu_ok is False


def test_watchdog_localization_loss_is_recover():
    clock = FakeClock()
    store = WorldStateStore()
    wd = _watchdog(clock, store)
    wd.observe_mcu()
    wd.observe_localization()
    clock.advance(15.0)
    wd.observe_mcu()  # MCU still fresh
    assert wd.tick() is WatchdogVerdict.RECOVER


def test_watchdog_emergency_stop():
    clock = FakeClock()
    wd = _watchdog(clock, WorldStateStore())
    wd.emergency_stop = True
    assert wd.tick() is WatchdogVerdict.SAFE_STOP
