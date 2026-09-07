"""WorldStateStore: single-writer semantics, event application, snapshots."""
import pytest

from core.model.enums import BlockType, MatchPhase, SkillStatus, TaskType
from core.model.pose import Pose2D, Velocity2D
from core.state.events import (
    BlockGrabbedEvent,
    BlockPlacedEvent,
    BlockReleasedEvent,
    BlockSeenEvent,
    FieldSupplyEvent,
    HardwareStatusEvent,
    ManipulatorEvent,
    MarkerSeenEvent,
    MatchStartedEvent,
    MatchTickEvent,
    MissionEvent,
    PoseUpdatedEvent,
    RouteNodeEvent,
)
from core.state.store import WorldStateStore
from core.state.world_state import WorldState


def test_initial_state_defaults():
    store = WorldStateStore()
    snap = store.snapshot()
    assert snap.inventory.orange_count == 0
    assert snap.inventory.purple_count == 0
    assert snap.robot.localization_confidence == 0.0
    assert snap.match.phase is MatchPhase.NORMAL


def test_pose_event_updates_robot():
    store = WorldStateStore()
    store.apply_event(PoseUpdatedEvent(
        pose=Pose2D(1.0, 2.0, 0.5),
        velocity=Velocity2D(0.1, 0.0, 0.01),
        confidence=0.9,
    ))
    snap = store.snapshot()
    assert snap.robot.x == pytest.approx(1.0)
    assert snap.robot.yaw == pytest.approx(0.5)
    assert snap.robot.localization_confidence == pytest.approx(0.9)


def test_snapshot_is_immutable():
    store = WorldStateStore()
    snap1 = store.snapshot()
    store.apply_event(PoseUpdatedEvent(pose=Pose2D(9, 9, 9)))
    # old snapshot must be unaffected by later events
    assert snap1.robot.x == 0.0
    assert store.snapshot().robot.x == 9.0


def test_match_events():
    store = WorldStateStore()
    store.apply_event(MatchStartedEvent())
    assert store.snapshot().match.started is True
    store.apply_event(MatchTickEvent(elapsed_time=10.0, remaining_time=350.0,
                                     phase=MatchPhase.NORMAL))
    snap = store.snapshot()
    assert snap.match.elapsed_time == 10.0
    assert snap.match.remaining_time == 350.0


def test_mission_event():
    store = WorldStateStore()
    store.apply_event(MissionEvent(
        current_task=TaskType.ACQUIRE_ORANGE,
        current_skill="FollowRoute",
        hfsm_state="MISSION.ACQUIRE.NAVIGATE_TO_MATERIAL",
        retry_count=1,
        last_result=SkillStatus.RUNNING,
    ))
    snap = store.snapshot()
    assert snap.mission.current_task is TaskType.ACQUIRE_ORANGE
    assert snap.mission.retry_count == 1


def test_route_event():
    store = WorldStateStore()
    store.apply_event(RouteNodeEvent(
        current_node="mid_1",
        current_segment="start->mid_1",
        expected_marker="marker_6",
        line_follow_active=True,
    ))
    snap = store.snapshot()
    assert snap.route.current_node == "mid_1"
    assert snap.route.line_follow_active is True


def test_perception_events():
    store = WorldStateStore()
    store.apply_event(MarkerSeenEvent(marker_id="marker_2", timestamp=12.5))
    store.apply_event(BlockSeenEvent(block_type=BlockType.ORANGE, timestamp=13.0))
    snap = store.snapshot()
    assert snap.perception.last_marker == "marker_2"
    assert snap.perception.last_marker_time == 12.5
    assert snap.perception.last_block == "ORANGE"


def test_hardware_partial_update_keeps_other_flags():
    store = WorldStateStore()
    store.apply_event(HardwareStatusEvent(mcu_ok=True, chassis_ok=True))
    store.apply_event(HardwareStatusEvent(emergency_stop=True))  # only this flag
    snap = store.snapshot()
    assert snap.hardware.mcu_ok is True      # unchanged
    assert snap.hardware.emergency_stop is True


def test_manipulator_event_partial_update():
    store = WorldStateStore()
    store.apply_event(ManipulatorEvent(homed=True, axis_x_position=150.0))
    store.apply_event(ManipulatorEvent(grip_detected=True))
    snap = store.snapshot()
    assert snap.manipulator.homed is True
    assert snap.manipulator.axis_x_position == 150.0
    assert snap.manipulator.grip_detected is True


def test_subscribe_listener_called():
    store = WorldStateStore()
    seen = []
    store.subscribe(seen.append)
    store.apply_event(MatchStartedEvent())
    assert len(seen) == 1
    assert seen[0].match.started is True


def test_worldstate_frozen():
    ws = WorldState()
    with pytest.raises(Exception):
        ws.robot.x = 5.0  # type: ignore[misc]


# ------------------------------------------------------------- field supply
def test_supply_event_sets_counts():
    store = WorldStateStore()
    store.apply_event(FieldSupplyEvent(orange_remaining=4, purple_remaining=2))
    snap = store.snapshot()
    assert snap.supply.orange_remaining == 4
    assert snap.supply.purple_remaining == 2


def test_grab_decrements_supply():
    store = WorldStateStore()
    store.apply_event(FieldSupplyEvent(orange_remaining=2, purple_remaining=1))
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    snap = store.snapshot()
    assert snap.supply.orange_remaining == 1
    assert snap.supply.purple_remaining == 1   # untouched
    assert snap.inventory.orange_count == 1


def test_grab_floors_supply_at_zero():
    store = WorldStateStore()
    store.apply_event(FieldSupplyEvent(orange_remaining=0, purple_remaining=0))
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    assert store.snapshot().supply.orange_remaining == 0


def test_release_returns_block_to_field():
    store = WorldStateStore()
    store.apply_event(FieldSupplyEvent(orange_remaining=1, purple_remaining=0))
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    assert store.snapshot().supply.orange_remaining == 0
    store.apply_event(BlockReleasedEvent(block_type=BlockType.ORANGE))
    snap = store.snapshot()
    assert snap.supply.orange_remaining == 1
    assert snap.inventory.orange_count == 0


def test_unknown_supply_unchanged_by_grab():
    """None (never initialized) stays None — the planner stays optimistic."""
    store = WorldStateStore()
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    assert store.snapshot().supply.orange_remaining is None
