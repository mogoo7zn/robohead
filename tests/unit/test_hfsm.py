"""HFSM engine tests: transitions, hierarchy, history, DONE, timeout."""
import pytest

from core.mission.hfsm import DONE, CompositeState, Machine, State
from core.utils.clock import FakeClock


class _Ctx:
    pass


# --------------------------------------------------------------- flat machine
class IdleState(State):
    name = "IDLE"
    ticks = 0

    def update(self):
        self.ticks += 1
        return None


class GoState(State):
    name = "GO"

    def update(self):
        return "IDLE"


class DoneState(State):
    name = "DONE_STATE"

    def update(self):
        return DONE


def build_flat():
    clock = FakeClock()
    m = Machine("flat", _Ctx(), clock, initial="IDLE")
    m.add_state(IdleState(m))
    m.add_state(GoState(m))
    m.add_state(DoneState(m))
    return m, clock


def test_machine_transitions():
    m, clock = build_flat()
    m.start()
    assert m.state_name == "IDLE"
    m.tick()
    assert m.state_name == "IDLE"
    m.transition("GO")
    m.tick()
    assert m.state_name == "IDLE"   # GO transitions back to IDLE


def test_machine_history_recorded():
    m, clock = build_flat()
    m.start()
    m.transition("GO")
    m.tick()
    assert m.history == ["IDLE", "GO", "IDLE"]


def test_machine_done():
    m, clock = build_flat()
    m.start()
    m.transition("DONE_STATE")
    assert m.tick() == DONE


def test_unknown_state_raises():
    m, clock = build_flat()
    m.start()
    with pytest.raises(KeyError):
        m.transition("NOPE")


def test_tick_before_start_raises():
    m, clock = build_flat()
    with pytest.raises(RuntimeError):
        m.tick()


# --------------------------------------------------------------- composite
class SubWork(State):
    name = "SUB_WORK"

    def update(self):
        return DONE


def build_hierarchical():
    clock = FakeClock()
    top = Machine("top", _Ctx(), clock, initial="OUTER")
    sub = Machine("sub", top.ctx, clock, initial="SUB_WORK")
    sub.add_state(SubWork(sub))
    outer = CompositeState(top, sub)
    outer.name = "OUTER"
    top.add_state(outer)
    return top, sub, clock


def test_composite_state_runs_submachine():
    top, sub, clock = build_hierarchical()
    top.start()
    assert sub.state_name == "SUB_WORK"
    assert top.full_state_name == "OUTER/SUB_WORK"
    # submachine completes -> composite reports DONE
    assert top.tick() == DONE


# --------------------------------------------------------------- timeout
class SlowState(State):
    name = "SLOW"
    timeout = 1.0

    def update(self):
        return None

    def on_timeout(self):
        return "RECOVERED"


class RecoveredState(State):
    name = "RECOVERED"

    def update(self):
        return DONE


def test_timeout_triggers_recovery_path():
    clock = FakeClock()
    m = Machine("t", _Ctx(), clock, initial="SLOW")
    m.add_state(SlowState(m))
    m.add_state(RecoveredState(m))
    m.start()
    assert m.tick() is None
    clock.advance(2.0)
    m.tick()
    assert m.state_name == "RECOVERED"
    assert m.timeout_count == 1


class TimeoutDoneState(State):
    name = "TIMEOUT_DONE"
    timeout = 0.5

    def update(self):
        return None

    def on_timeout(self):
        return DONE


def test_timeout_can_signal_done():
    clock = FakeClock()
    m = Machine("t2", _Ctx(), clock, initial="TIMEOUT_DONE")
    m.add_state(TimeoutDoneState(m))
    m.start()
    clock.advance(1.0)
    assert m.tick() == DONE
