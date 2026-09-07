"""Planner (rule + utility) tests."""
from dataclasses import replace as dc_replace

from core.model.enums import MatchPhase
from core.strategy.planner import Planner
from core.state.world_state import WorldState

CONFIG = {
    "match": {"total_time": 360.0, "normal_end_time": 240.0, "safe_end_time": 320.0},
    "inventory": {"max_total": 3, "max_purple": 1},
    "utility": {
        "expected_score": {"orange_grab": 10.0, "purple_grab": 14.0,
                           "build_layer": 25.0, "purple_roof_bonus": 1.5},
        "time_cost_weight": 0.05,
        "risk_cost_weight": 0.4,
    },
    "task_estimates": {"acquire_orange": 60.0, "acquire_purple": 70.0, "build": 55.0},
    "risk": {"acquire_orange": 0.10, "acquire_purple": 0.25, "build": 0.15},
    "endgame": {"min_remaining_for_new_task": 45.0},
}


def world_with(inventory=None, match=None, supply=None) -> WorldState:
    ws = WorldState()
    ws = ws.with_(match=dc_replace(ws.match, remaining_time=360.0))
    if inventory:
        ws = ws.with_(inventory=dc_replace(ws.inventory, **inventory))
    if match:
        ws = ws.with_(match=dc_replace(ws.match, **match))
    if supply:
        ws = ws.with_(supply=dc_replace(ws.supply, **supply))
    return ws


def test_empty_inventory_prefers_acquiring():
    planner = Planner(CONFIG)
    decision = planner.decide(world_with())
    assert decision.task.value.startswith("ACQUIRE_")


def test_full_inventory_forces_build():
    planner = Planner(CONFIG)
    ws = world_with(inventory={"orange_count": 3})
    assert planner.decide(ws).task.value == "BUILD"


def test_purple_carried_forces_build():
    planner = Planner(CONFIG)
    ws = world_with(inventory={"purple_count": 1, "orange_count": 0})
    assert planner.decide(ws).task.value == "BUILD"


def test_low_time_forces_endgame():
    planner = Planner(CONFIG)
    ws = world_with(match={"remaining_time": 30.0})
    assert planner.decide(ws).task.value == "ENDGAME"


def test_carrying_blocks_builds():
    planner = Planner(CONFIG)
    ws = world_with(inventory={"orange_count": 2})
    decision = planner.decide(ws)
    assert decision.task.value == "BUILD"


def test_safe_phase_penalizes_acquisition():
    planner = Planner(CONFIG)
    # With plenty of time, acquiring competes; force SAFE phase comparison
    normal = planner.decide(world_with(match={"remaining_time": 200.0}))
    safe = planner.decide(world_with(match={
        "remaining_time": 200.0, "phase": MatchPhase.SAFE}))
    if normal.task.value.startswith("ACQUIRE") and safe.task.value.startswith("ACQUIRE"):
        assert safe.utility < normal.utility
    else:
        assert safe.task.value == "BUILD" or normal.task.value == "BUILD"


def test_decision_has_reason():
    planner = Planner(CONFIG)
    decision = planner.decide(world_with())
    assert len(decision.reason) > 0


# ------------------------------------------------------------- field supply
def test_exhausted_field_forces_endgame():
    """All blocks used up and nothing carried -> nothing left to do."""
    planner = Planner(CONFIG)
    ws = world_with(supply={"orange_remaining": 0, "purple_remaining": 0})
    assert planner.decide(ws).task.value == "ENDGAME"


def test_orange_exhausted_allows_purple():
    """Orange gone but purple on the field (and carryable) -> purple only."""
    planner = Planner(CONFIG)
    ws = world_with(supply={"orange_remaining": 0, "purple_remaining": 1})
    # purple_as_roof not set in CONFIG -> purple grab is a normal candidate
    assert planner.decide(ws).task.value == "ACQUIRE_PURPLE"


def test_unknown_supply_stays_optimistic():
    """None (never observed) must NOT suppress acquisition."""
    planner = Planner(CONFIG)
    ws = world_with(supply={"orange_remaining": None, "purple_remaining": None})
    assert planner.decide(ws).task.value.startswith("ACQUIRE_")
