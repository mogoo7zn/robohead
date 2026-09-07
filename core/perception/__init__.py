"""Perception subsystem: marker detection (localization camera) and block
detection (block camera). Hardware-agnostic interfaces + backends."""
from core.perception.block_detector import (
    BlockDetection,
    BlockDetector,
    HsvBlockDetector,
    MockBlockDetector,
)
from core.perception.marker_detector import (
    MarkerDetector,
    MockMarkerDetector,
)

__all__ = [
    "BlockDetection",
    "BlockDetector",
    "HsvBlockDetector",
    "MarkerDetector",
    "MockBlockDetector",
    "MockMarkerDetector",
]
