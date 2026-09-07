"""Marker (visual tag) data model for the 6-marker field localization."""
from __future__ import annotations

from dataclasses import dataclass, field

from core.model.pose import Pose2D


@dataclass
class MarkerDefinition:
    """A marker fixed in the field (map frame).

    Real coordinates must come from config/field.example.yaml — never guessed
    in code. physical_width/height are in metres (used by PnP).
    """
    marker_id: str
    name: str = ""
    world_pose: Pose2D = field(default_factory=Pose2D)
    physical_width: float = 0.15
    physical_height: float = 0.15
    height_above_ground: float = 0.0  # z of marker centre in map frame

    def as_dict(self) -> dict:
        return {
            "id": self.marker_id,
            "name": self.name,
            "x": self.world_pose.x,
            "y": self.world_pose.y,
            "yaw": self.world_pose.yaw,
            "width": self.physical_width,
            "height": self.physical_height,
            "z": self.height_above_ground,
        }


@dataclass
class MarkerDetection:
    """One detected marker in the localization camera frame.

    Output of MarkerDetector backends (AprilTag / ArUco / custom). Corner
    order: top-left, top-right, bottom-right, bottom-left in image pixels.
    """
    marker_id: str
    corners: list[tuple[float, float]] = field(default_factory=list)
    center_u: float = 0.0
    center_v: float = 0.0
    confidence: float = 1.0
    timestamp: float = 0.0
    # Optional backend-specific relative pose (translation in camera optical
    # frame, rotation as XYZ euler). PoseEstimator must not require it.
    relative_translation: tuple[float, float, float] | None = None
    relative_rotation: tuple[float, float, float] | None = None

    @property
    def area(self) -> float:
        if len(self.corners) != 4:
            return 0.0
        (x1, y1), (x2, y2), (x3, y3), (x4, y4) = self.corners
        return abs(0.5 * (x1 * y2 - x2 * y1 + x2 * y3 - x3 * y2
                          + x3 * y4 - x4 * y3 + x4 * y1 - x1 * y4))
