"""Manipulator sequencer + GrabSkill / PlaceSkill over SimWorld."""
import pytest

from core.manipulation.manipulator import (
    ManipulatorSequencer,
    SimManipulator,
)
from core.mock.sim_world import SimBlock, SimWorld
from core.model.enums import GripperState, SkillStatus
from core.model.pose import Pose2D
from core.skill.grab import GrabSkill
from core.skill.place import PlaceSkill
from core.utils.clock import FakeClock


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock(0.0)


@pytest.fixture
def world() -> SimWorld:
    ws = SimWorld()
    ws.robot_pose = Pose2D(2.75, 0.30, 0.0)        # over a block
    ws.blocks = [SimBlock("ORANGE", 2.75, 0.30, 0.0)]
    return ws


@pytest.fixture
def sequencer(world, clock) -> ManipulatorSequencer:
    return ManipulatorSequencer(SimManipulator(world, clock), clock=clock)


def run_skill(update_fn, clock, world, dt=0.02, budget=40.0):
    """Advance sim time + physics + skill until it leaves RUNNING."""
    result = None
    for _ in range(int(budget / dt)):
        clock.advance(dt)
        world.step(dt, clock)
        result = update_fn()
        if result.status is not SkillStatus.RUNNING:
            return result
    return result


# ---------------------------------------------------------------- sequencer
def test_axis_move_completes(sequencer, world, clock):
    sequencer.begin_move_to(150.0, 80.0)
    result = None
    for _ in range(500):
        clock.advance(0.02)
        world.step(0.02, clock)
        result = sequencer.tick()
        if result == "DONE":
            break
    assert result == "DONE"
    assert world.manip_x_mm == pytest.approx(150.0, abs=0.1)
    assert world.manip_z_mm == pytest.approx(80.0, abs=0.1)


def test_gripper_close_waits_for_grip_sensor(sequencer, world, clock):
    sequencer.begin_gripper(close=True)
    assert sequencer.tick() == "RUNNING"      # gripper still closing
    for _ in range(100):
        clock.advance(0.02)
        world.step(0.02, clock)
        if sequencer.tick() == "DONE":
            break
    assert world.manip_gripper is GripperState.HOLDING
    assert world.manip_grip_detected
    assert world.held_block is not None


def test_gripper_open_waits_for_release(sequencer, world, clock):
    sequencer.begin_gripper(close=True)
    for _ in range(100):
        clock.advance(0.02)
        world.step(0.02, clock)
        if sequencer.tick() == "DONE":
            break
    sequencer.begin_gripper(close=False)
    for _ in range(100):
        clock.advance(0.02)
        world.step(0.02, clock)
        if sequencer.tick() == "DONE":
            break
    assert world.manip_gripper is GripperState.OPEN
    assert not world.manip_grip_detected
    assert world.held_block is None


def test_step_timeout_detected(sequencer, world, clock):
    sequencer.begin_move_to(150.0, 80.0)
    clock.advance(30.0)                        # > move timeout 20 s
    assert sequencer.tick() == "TIMEOUT"


# ---------------------------------------------------------------- grab skill
def test_grab_success_full_cycle(sequencer, world, clock):
    skill = GrabSkill(sequencer, clock=clock)
    skill.start()
    result = run_skill(skill.update, clock, world)
    assert result is not None
    assert result.status is SkillStatus.SUCCESS, result.message
    assert result.verified
    assert result.attempts == 1
    assert world.held_block is not None
    assert world.held_block.block_type == "ORANGE"
    # ended homed
    assert world.manip_homed
    assert world.manip_x_mm == pytest.approx(0.0, abs=0.1)


def test_grab_failure_when_no_block(sequencer, world, clock):
    world.blocks = []                          # nothing to grab
    skill = GrabSkill(sequencer, {"max_retries": 1}, clock=clock)
    skill.start()
    result = run_skill(skill.update, clock, world)
    assert result is not None
    assert result.status is SkillStatus.FAILURE
    assert result.attempts == 2               # 1 + 1 retry


def test_grab_not_started(sequencer, clock):
    skill = GrabSkill(sequencer, clock=clock)
    result = skill.update()
    assert result.status is SkillStatus.FAILURE
    assert "not started" in result.message


def test_grab_retry_on_empty_close(sequencer, world, clock):
    """First close gets nothing; a block appears before the retry."""
    world.blocks = []
    skill = GrabSkill(sequencer, {"max_retries": 2}, clock=clock)
    skill.start()
    result = None
    for _ in range(1500):
        clock.advance(0.02)
        world.step(0.02, clock)
        if skill.state == "PREGRASP" and skill.attempts == 2:
            # block appears within reach for the retry
            if not world.blocks:
                world.blocks = [SimBlock("PURPLE", 2.75, 0.30, 0.0)]
        result = skill.update()
        if result.status is not SkillStatus.RUNNING:
            break
    assert result is not None
    assert result.status is SkillStatus.SUCCESS
    assert result.attempts == 2
    assert world.held_block.block_type == "PURPLE"


# ---------------------------------------------------------------- place skill
def test_place_success_with_stability_wait(sequencer, world, clock):
    # grab first
    grab = GrabSkill(sequencer, clock=clock)
    grab.start()
    result = run_skill(grab.update, clock, world)
    assert result.status is SkillStatus.SUCCESS

    place = PlaceSkill(sequencer, clock=clock)
    place.start(layer=0)
    result = run_skill(place.update, clock, world)
    assert result.status is SkillStatus.SUCCESS
    assert result.released
    assert result.stable
    # block is on the tower now
    placed = [b for b in world.blocks if b.placed]
    assert len(placed) == 1


def test_place_stability_wait_is_three_seconds(sequencer, world, clock):
    grab = GrabSkill(sequencer, clock=clock)
    grab.start()
    run_skill(grab.update, clock, world)

    place = PlaceSkill(sequencer, clock=clock)
    place.start(layer=1)
    # run until we are in the stability wait
    for _ in range(1500):
        clock.advance(0.02)
        world.step(0.02, clock)
        r = place.update()
        if place.state == "STABILITY_WAIT":
            break
    assert place.state == "STABILITY_WAIT"

    t_wait_start = clock.now()
    result = None
    for _ in range(500):
        clock.advance(0.02)
        result = place.update()
        if result.status is not SkillStatus.RUNNING:
            break
    waited = clock.now() - t_wait_start
    assert result.status is SkillStatus.SUCCESS
    assert waited >= 3.0 - 0.05               # rule: >= 3 s


def test_place_not_started(sequencer, clock):
    place = PlaceSkill(sequencer, clock=clock)
    result = place.update()
    assert result.status is SkillStatus.FAILURE


def test_place_retreats_then_stops_chassis(sequencer, world, clock):
    from core.navigation.chassis import MockChassis
    chassis = MockChassis(world)

    grab = GrabSkill(sequencer, clock=clock)
    grab.start()
    run_skill(grab.update, clock, world)

    place = PlaceSkill(sequencer, chassis=chassis, clock=clock)
    place.start(layer=0)
    result = run_skill(place.update, clock, world)
    assert result.status is SkillStatus.SUCCESS

    velocities = [c for c in chassis.commands if c[0] == "velocity"]
    stops = [c for c in chassis.commands if c[0] == "stop"]
    assert velocities and velocities[-1][1] < 0     # backwards retreat
    assert stops                                    # ... and a stop after it
    assert chassis.commands.index(stops[-1]) > chassis.commands.index(velocities[-1])


def test_place_higher_layer_descends_to_layer_height(sequencer, world, clock):
    grab = GrabSkill(sequencer, clock=clock)
    grab.start()
    run_skill(grab.update, clock, world)

    place = PlaceSkill(sequencer, clock=clock)
    place.start(layer=2)                       # two blocks already stacked
    result = run_skill(place.update, clock, world)
    assert result.status is SkillStatus.SUCCESS
