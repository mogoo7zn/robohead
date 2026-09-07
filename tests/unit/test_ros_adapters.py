"""ROS adapter tests — run on Mac, no ROS installation required.

The adapters import cleanly without rclpy (they only need cv2 + numpy +
core); the ROS node itself just wires them to topics. These
tests validate the real-backend logic with synthetic images:

  * image_bridge: sensor_msgs-like message -> BGR ndarray
  * LatestFrameStore: newest-frame-wins, consume-once, staleness
  * CameraBlockDetector: HSV pipeline over pushed frames
  * CameraMarkerDetector: AprilTag generated with cv2.aruco is detected
    and mapped to the MarkerMap id convention
"""
import sys
from pathlib import Path

import pytest

ROS2_SRC = Path(__file__).resolve().parents[2] / "ros2_ws" / "src"
for _pkg in ("robogame_adapters", "robogame_bringup"):
    sys.path.insert(0, str(ROS2_SRC / _pkg))

from robogame_adapters.camera_block_detector import CameraBlockDetector  # noqa: E402
from robogame_adapters.camera_marker_detector import CameraMarkerDetector  # noqa: E402
from robogame_adapters.image_bridge import image_to_ndarray  # noqa: E402
from robogame_adapters.latest_frame import LatestFrameStore  # noqa: E402
from core.model.enums import BlockType  # noqa: E402
from core.utils.config import load_yaml  # noqa: E402

cv2 = pytest.importorskip("cv2")


class FakeImage:
    """Duck-typed sensor_msgs/Image (no ROS needed)."""

    def __init__(self, array_bgr):
        self.height, self.width = array_bgr.shape[:2]
        self.encoding = "bgr8"
        self.data = array_bgr.tobytes()


# ------------------------------------------------------------ image bridge
def test_bridge_bgr8_roundtrip():
    import numpy as np
    img = np.zeros((10, 12, 3), dtype=np.uint8)
    img[3, 4] = (255, 128, 10)                 # B, G, R
    out = image_to_ndarray(FakeImage(img))
    assert out.shape == (10, 12, 3)
    assert tuple(out[3, 4]) == (255, 128, 10)


def test_bridge_rgb8_swaps_to_bgr():
    import numpy as np
    img = np.zeros((4, 4, 3), dtype=np.uint8)
    img[1, 1] = (10, 128, 255)                # R, G, B
    out = image_to_ndarray(FakeImage(img))     # constructed as bgr8 below
    # build an rgb8 message manually
    msg = FakeImage(img)
    msg.encoding = "rgb8"
    out = image_to_ndarray(msg)
    assert tuple(out[1, 1]) == (255, 128, 10)  # B, G, R


def test_bridge_rejects_unknown_encoding():
    import numpy as np
    msg = FakeImage(np.zeros((2, 2, 3), dtype=np.uint8))
    msg.encoding = "32FC1"
    with pytest.raises(Exception):
        image_to_ndarray(msg)


# ------------------------------------------------------------ frame store
def test_frame_store_newest_wins_and_consumes_once():
    store = LatestFrameStore()
    store.push("frame1", 1.0)
    store.push("frame2", 2.0)          # replaces frame1
    assert store.take(2.0) == ("frame2", 2.0)
    assert store.take(2.1) is None     # consumed -> nothing until next push


def test_frame_store_stale_frame_dropped():
    store = LatestFrameStore(max_age=0.5)
    store.push("old", 1.0)
    assert store.take(2.0) is None     # older than max_age


def test_frame_store_empty():
    store = LatestFrameStore()
    assert store.take(0.0) is None


# ------------------------------------------------------------ block camera
def test_block_camera_adapter_detects_orange():
    import numpy as np
    cfg = load_yaml("config/perception.yaml")["block_detector"]
    det = CameraBlockDetector(cfg)

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (40, 40, 40)
    frame[260:340, 160:240] = (30, 120, 255)      # orange square
    det.on_frame(frame, 10.0)

    detections = det.detect(10.02)
    oranges = [d for d in detections if d.block_type is BlockType.ORANGE]
    assert len(oranges) == 1
    assert oranges[0].u == pytest.approx(200.0, abs=3.0)
    assert oranges[0].timestamp == pytest.approx(10.0)


def test_block_camera_adapter_consumes_frame_once():
    import numpy as np
    cfg = load_yaml("config/perception.yaml")["block_detector"]
    det = CameraBlockDetector(cfg)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[260:340, 160:240] = (30, 120, 255)
    det.on_frame(frame, 5.0)
    assert det.detect(5.01) != []
    assert det.detect(5.02) == []          # consumed
    det.on_frame(frame, 6.0)
    assert det.detect(6.01) != []           # next frame serves again


# ------------------------------------------------------------ marker camera
def _generate_apriltag(tag_id: int, size_px: int = 200):
    """Render one 36h11 AprilTag on a white background."""
    import cv2.aruco as aruco
    import numpy as np
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36H11)
    tag = aruco.generateImageMarker(dictionary, tag_id, size_px)
    img = np.full((size_px + 200, size_px + 200), 255, dtype=np.uint8)
    img[100:100 + size_px, 100:100 + size_px] = tag
    return img


def test_marker_camera_adapter_roundtrip():
    det = CameraMarkerDetector({
        "backend": "apriltag", "apriltag_family": "36h11"})
    det.on_frame(_generate_apriltag(7), 3.0)

    obs = det.detect(3.05)
    assert len(obs) == 1
    assert obs[0].marker_id == "marker_7"     # MarkerMap id convention
    assert obs[0].corners_px.shape == (4, 2)
    assert obs[0].timestamp == pytest.approx(3.0)


def test_marker_camera_adapter_id_map_override():
    det = CameraMarkerDetector({
        "backend": "apriltag", "apriltag_family": "36h11",
        "id_map": {7: "build_gate"}})
    det.on_frame(_generate_apriltag(7), 1.0)
    obs = det.detect(1.05)
    assert obs[0].marker_id == "build_gate"


def test_marker_camera_adapter_consumes_frame():
    det = CameraMarkerDetector({
        "backend": "apriltag", "apriltag_family": "36h11"})
    det.on_frame(_generate_apriltag(3), 1.0)
    assert det.detect(1.05) != []
    assert det.detect(1.06) == []            # consumed until next frame
