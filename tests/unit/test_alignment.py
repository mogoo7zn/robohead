"""AlignmentController: visual servoing onto a block, state machine,
timeouts, and closed-loop convergence through a simulated robot."""
import pytest

from core.localization.camera import CameraParams
from core.mock.sim_world import SimWorld, SimBlock
from core.model.enums import AlignmentState, BlockType, SkillStatus
from core.model.pose import Pose2D
from core.perception.block_detector import MockBlockDetector
from core.skill.alignment import AlignmentController
from core.utils.clock import FakeClock
from core.utils.config import load_yaml


@pytest.fixture(scope="module")
def mock_field() -> dict:
    return load_yaml("config/mock_field.yaml")


@pytest.fixture(scope="module")
def camera(mock_field) -> CameraParams:
    return CameraParams.from_config(mock_field["block_camera"])


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock(0.0)


@pytest.fixture
def controller(clock) -> AlignmentController:
    return AlignmentController(clock=clock)


def make_detection(u, v, angle=0.0, block_type=BlockType.ORANGE):
    from core.perception.block_detector import BlockDetection
    return BlockDetection(block_type=block_type, u=u, v=v, area=5000.0,
                          angle=angle, timestamp=0.0)


# ------------------------------------------------------------- state machine
def test_starts_in_search(controller):
    assert controller.state is AlignmentState.SEARCH


def test_no_target_search_rotates(controller):
    out = controller.update([])
    assert out.state is AlignmentState.SEARCH
    assert out.velocity.wz > 0        # gentle search rotation
    assert out.status is SkillStatus.RUNNING


def test_search_times_out(controller, clock):
    controller.start(BlockType.ORANGE)
    clock.advance(25.0)               # > search_timeout 20 s
    out = controller.update([])
    assert out.state is AlignmentState.FAILURE
    assert out.status is SkillStatus.FAILURE


def test_target_transitions_to_align(controller):
    controller.start(BlockType.ORANGE)
    out = controller.update([make_detection(380.0, 300.0)])
    assert out.state is AlignmentState.ALIGN
    assert out.status is SkillStatus.RUNNING
    assert out.target is not None


def test_aligned_within_tolerance(controller):
    controller.start(BlockType.ORANGE)
    out = controller.update([make_detection(325.0, 245.0, 0.05)])
    assert out.state is AlignmentState.ALIGNED
    assert out.status is SkillStatus.SUCCESS
    assert out.velocity.is_zero()


def test_filters_unwanted_type(controller):
    controller.start(BlockType.PURPLE)
    out = controller.update([make_detection(380.0, 300.0,
                                            block_type=BlockType.ORANGE)])
    assert out.state is AlignmentState.SEARCH   # no purple found


def test_picks_closest_to_grab_point(controller):
    controller.start(BlockType.ORANGE)
    dets = [make_detection(500.0, 300.0), make_detection(340.0, 250.0)]
    out = controller.update(dets)
    assert out.target is not None
    assert out.target.u == pytest.approx(340.0)


def test_target_lost_after_timeout(controller, clock):
    controller.start(BlockType.ORANGE)
    controller.update([make_detection(380.0, 300.0)])
    clock.advance(3.0)                # > target_lost_timeout 2 s
    out = controller.update([])
    assert out.state is AlignmentState.TARGET_LOST
    assert out.status is SkillStatus.FAILURE
    assert out.velocity.is_zero()


def test_target_recovers_when_seen_again(controller, clock):
    controller.start(BlockType.ORANGE)
    controller.update([make_detection(380.0, 300.0)])
    clock.advance(3.0)
    lost = controller.update([])       # -> TARGET_LOST
    assert lost.state is AlignmentState.TARGET_LOST
    out = controller.update([make_detection(380.0, 300.0)])
    # re-acquired: back to servoing (TRACK is transient within the frame)
    assert out.state in (AlignmentState.TRACK, AlignmentState.ALIGN)
    assert out.target is not None
    assert out.status is SkillStatus.RUNNING


# ------------------------------------------------------------- control signs
def test_control_signs(controller):
    """Block right of centre -> strafe right (vy<0); block too close
    (v large) -> back up (vx<0); block CCW of robot -> turn CCW (wz>0)."""
    controller.start(BlockType.ORANGE)
    out = controller.update([make_detection(400.0, 300.0, angle=0.3)])
    assert out.velocity.vy < 0      # u error +80 px -> vy negative
    assert out.velocity.vx < 0      # v error +60 px -> too close -> back up
    assert out.velocity.wz > 0      # angle +0.3 -> robot turns CCW to match


def test_control_signs_far_block(controller):
    controller.start(BlockType.ORANGE)
    out = controller.update([make_detection(300.0, 150.0, angle=-0.3)])
    assert out.velocity.vy > 0      # u error -20 -> strafe left
    assert out.velocity.vx > 0      # v small -> far -> approach
    assert out.velocity.wz < 0


def test_velocities_clamped(controller):
    controller.start(BlockType.ORANGE)
    out = controller.update([make_detection(6000.0, 5000.0, angle=5.0)])
    assert abs(out.velocity.vx) <= 0.15 + 1e-9
    assert abs(out.velocity.vy) <= 0.15 + 1e-9
    assert abs(out.velocity.wz) <= 0.8 + 1e-9


# ------------------------------------------------------------- closed loop
def test_closed_loop_converges_with_sim(camera, clock):
    """Full loop: detector renders SimWorld -> controller commands ->
    SimWorld moves -> converges onto the calibrated grab point."""
    from core.model.pose import integrate_pose

    ws = SimWorld()
    ws.robot_pose = Pose2D(0.0, 0.05, 0.0)
    ws.blocks = [SimBlock("ORANGE", 0.20, 0.02, 0.0)]
    det = MockBlockDetector(
        camera, blocks_provider=lambda: ws.blocks,
        pose_provider=lambda: ws.robot_pose, block_size=0.07)
    ctrl = AlignmentController(clock=clock)
    ctrl.start(BlockType.ORANGE)

    dt = 0.05
    outcome = None
    for _ in range(400):              # 20 s max
        clock.advance(dt)
        detections = det.detect(clock.now())
        out = ctrl.update(detections, timestamp=clock.now())
        if out.status is not SkillStatus.RUNNING:
            outcome = out
            break
        ws.robot_pose = integrate_pose(ws.robot_pose, out.velocity, dt)

    assert outcome is not None, "alignment did not finish"
    assert outcome.state is AlignmentState.ALIGNED
    # robot ended near the calibrated grab geometry: block ~0.2 m ahead,
    # ~0 lateral offset
    assert ws.robot_pose.x == pytest.approx(0.0, abs=0.04)
    assert ws.robot_pose.y == pytest.approx(0.02, abs=0.04)
    assert abs(outcome.error_u) <= 15.0
    assert abs(outcome.error_v) <= 12.0


def test_stop_resets_state(controller):
    controller.start(BlockType.ORANGE)
    controller.update([make_detection(380.0, 300.0)])
    controller.stop()
    assert controller.state is AlignmentState.SEARCH
    assert controller.target is None
