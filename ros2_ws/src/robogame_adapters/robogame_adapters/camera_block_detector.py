"""CameraBlockDetector — HSV backend for the block camera.

Real-hardware counterpart of MockBlockDetector: same
`detect(timestamp) -> list[BlockDetection]` contract. Frames arrive from
the ROS block-camera topic via on_frame(); the heavy HSV pipeline
(config/perception.yaml) runs on the mission control thread through the
already-tested HsvBlockDetector.
"""
from __future__ import annotations

from core.perception.block_detector import HsvBlockDetector
from core.utils.log import get_logger
from robogame_adapters.latest_frame import LatestFrameStore

log = get_logger("adapters.block_detector")


class CameraBlockDetector:
    """HSV-threshold block detector satisfying the BlockDetector protocol."""

    def __init__(self, config: dict) -> None:
        # config: the block_detector section of perception.yaml
        self._hsv = HsvBlockDetector(config)
        max_age = float(config.get("max_age", 0.5))
        self._store = LatestFrameStore(max_age=max_age)

    # ------------------------------------------------------------- ROS side
    def on_frame(self, image_bgr, timestamp: float) -> None:
        self._store.push(image_bgr, timestamp)

    # ---------------------------------------------------- protocol side
    def detect(self, timestamp: float) -> list:
        frame = self._store.take(timestamp)
        if frame is None:
            return []
        image, stamp = frame
        try:
            detections = self._hsv.detect_in_image(image, stamp)
        except Exception as exc:          # cv2 missing / bad frame
            log.warning("block detection failed: %s", exc)
            return []
        if detections:
            log.debug("blocks seen: %d (%s)", len(detections),
                      [d.block_type.value for d in detections])
        return detections
