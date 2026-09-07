"""BlockDetector: mock rendering + HSV backend on synthetic frames."""
import pytest

from core.localization.camera import CameraParams
from core.mock.sim_world import SimWorld, SimBlock
from core.model.enums import BlockType
from core.model.pose import Pose2D
from core.perception.block_detector import (
    HsvBlockDetector,
    MockBlockDetector,
)
from core.utils.config import load_yaml


@pytest.fixture(scope="module")
def mock_field() -> dict:
    return load_yaml("config/mock_field.yaml")


@pytest.fixture(scope="module")
def camera(mock_field) -> CameraParams:
    return CameraParams.from_config(mock_field["block_camera"])


@pytest.fixture
def sim(camera) -> tuple[SimWorld, MockBlockDetector]:
    ws = SimWorld()
    ws.robot_pose = Pose2D(0.0, 0.0, 0.0)
    det = MockBlockDetector(
        camera,
        blocks_provider=lambda: ws.blocks,
        pose_provider=lambda: ws.robot_pose,
        block_size=0.07)
    return ws, det


# ----------------------------------------------------------------- mock
def test_block_ahead_at_grab_point(sim):
    ws, det = sim
    ws.blocks = [SimBlock("ORANGE", 0.20, 0.0, 0.0)]
    out = det.detect(0.0)
    assert len(out) == 1
    d = out[0]
    assert d.block_type is BlockType.ORANGE
    assert d.u == pytest.approx(320.0, abs=1.0)
    assert d.v == pytest.approx(240.0, abs=10.0)   # calibrated grab point


def test_lateral_offset_shifts_u(sim):
    ws, det = sim
    ws.blocks = [SimBlock("ORANGE", 0.20, 0.0, 0.0),
                 SimBlock("PURPLE", 0.20, 0.10, 0.0)]
    out = det.detect(0.0)
    by_type = {d.block_type: d for d in out}
    # block 10 cm to the LEFT (map +y) appears right of centre -> u < 320
    assert by_type[BlockType.ORANGE].u == pytest.approx(320.0, abs=1.0)
    assert by_type[BlockType.PURPLE].u < 280.0


def test_block_behind_camera_invisible(sim):
    ws, det = sim
    ws.blocks = [SimBlock("ORANGE", -0.20, 0.0, 0.0)]
    assert det.detect(0.0) == []


def test_far_block_beyond_max_distance_invisible(sim):
    ws, det = sim
    ws.blocks = [SimBlock("ORANGE", 5.0, 0.0, 0.0)]
    assert det.detect(0.0) == []


def test_grabbed_blocks_not_detected(sim):
    ws, det = sim
    b = SimBlock("ORANGE", 0.20, 0.0, 0.0)
    b.grabbed = True
    ws.blocks = [b]
    assert det.detect(0.0) == []


def test_placed_blocks_not_detected(sim):
    ws, det = sim
    b = SimBlock("ORANGE", 0.20, 0.0, 0.0)
    b.placed = True
    ws.blocks = [b]
    assert det.detect(0.0) == []


def test_detection_tracks_robot_motion(sim):
    """Robot strafes toward the block -> block image u moves to centre."""
    ws, det = sim
    ws.blocks = [SimBlock("ORANGE", 0.30, 0.15, 0.0)]
    ws.robot_pose = Pose2D(0.0, 0.0, 0.0)
    u0 = det.detect(0.0)[0].u
    ws.robot_pose = Pose2D(0.0, 0.10, 0.0)  # robot moved 10 cm towards block
    u1 = det.detect(0.0)[0].u
    assert u1 > u0


def test_block_angle_reflects_relative_yaw(sim):
    ws, det = sim
    ws.blocks = [SimBlock("ORANGE", 0.20, 0.0, 0.5)]
    ws.robot_pose = Pose2D(0.0, 0.0, 0.0)
    angle = det.detect(0.0)[0].angle
    assert angle == pytest.approx(0.5, abs=1e-6)

    ws.robot_pose = Pose2D(0.0, 0.0, 0.5)  # robot turned with the block
    angle = det.detect(0.0)[0].angle
    assert abs(angle) < 1e-6


def test_pixel_noise_keeps_detection(sim, camera):
    ws = SimWorld()
    ws.robot_pose = Pose2D(0.0, 0.0, 0.0)
    ws.blocks = [SimBlock("ORANGE", 0.20, 0.0, 0.0)]
    det = MockBlockDetector(camera,
                            blocks_provider=lambda: ws.blocks,
                            pose_provider=lambda: ws.robot_pose,
                            pixel_noise=1.0, seed=3)
    d = det.detect(0.0)[0]
    assert d.u == pytest.approx(320.0, abs=10.0)
    assert d.v == pytest.approx(240.0, abs=10.0)


# ----------------------------------------------------------------- HSV backend
def _synthetic_frame(color_bgr, cx, cy, size_px):
    """BGR image with one colored square on a dark background."""
    import numpy as np
    try:
        import cv2
    except ImportError:
        pytest.skip("cv2 unavailable")
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (40, 40, 40)
    x0, y0 = int(cx - size_px / 2), int(cy - size_px / 2)
    img[y0:y0 + size_px, x0:x0 + size_px] = color_bgr
    return img


def test_hsv_detects_orange_block():
    try:
        import cv2  # noqa: F401
    except ImportError:
        pytest.skip("cv2 unavailable")

    orange_bgr = (30, 120, 255)          # vivid orange in BGR
    frame = _synthetic_frame(orange_bgr, 200, 300, 80)
    cfg = load_yaml("config/perception.yaml")["block_detector"]
    det = HsvBlockDetector(cfg)
    out = det.detect_in_image(frame, timestamp=1.0)
    oranges = [d for d in out if d.block_type is BlockType.ORANGE]
    assert len(oranges) == 1
    d = oranges[0]
    assert d.u == pytest.approx(200.0, abs=3.0)
    assert d.v == pytest.approx(300.0, abs=3.0)
    assert d.area == pytest.approx(80 * 80, rel=0.15)
    assert d.timestamp == 1.0


def test_hsv_rejects_noise_blobs():
    try:
        import cv2
    except ImportError:
        pytest.skip("cv2 unavailable")
    import numpy as np

    cfg = load_yaml("config/perception.yaml")["block_detector"]
    det = HsvBlockDetector(cfg)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (40, 40, 40)
    # tiny orange specks below min_area
    for i in range(20):
        frame[100:104, 100 + 10 * i:104 + 10 * i] = (30, 120, 255)
    out = det.detect_in_image(frame, timestamp=0.0)
    assert out == []
