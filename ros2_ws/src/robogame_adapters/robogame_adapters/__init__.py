"""robogame_adapters — real-hardware backends for core.

Each adapter satisfies the same Protocol as its mock counterpart, so the
mission layer cannot tell them apart:

  MarkerDetector.detect(t) -> list[MarkerObservation]
  BlockDetector.detect(t)  -> list[BlockDetection]

The ROS 2 bringup node pushes camera frames into the adapters (via
on_frame()); the mission loop pulls detections via detect(). No cv_bridge
dependency: frames are converted with numpy directly.
"""
from robogame_adapters.camera_marker_detector import CameraMarkerDetector
from robogame_adapters.camera_block_detector import CameraBlockDetector
from robogame_adapters.image_bridge import image_to_ndarray

__all__ = [
    "CameraMarkerDetector",
    "CameraBlockDetector",
    "image_to_ndarray",
]
