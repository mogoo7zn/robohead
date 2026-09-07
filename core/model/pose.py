"""2D pose / velocity primitives and angle math for the planar field model."""
from __future__ import annotations

import math
from dataclasses import dataclass, field


def normalize_angle(angle: float) -> float:
    """Wrap angle to (-pi, pi]."""
    a = math.atan2(math.sin(angle), math.cos(angle))
    return a


def angle_diff(a: float, b: float) -> float:
    """Signed smallest difference a - b, wrapped to (-pi, pi]."""
    return normalize_angle(a - b)


def circular_mean(angles, weights=None) -> float:
    """Correct weighted circular mean of angles.

    Never average angles with a plain arithmetic mean.
    """
    angles = list(angles)
    if not angles:
        raise ValueError("circular_mean of empty list")
    if weights is None:
        weights = [1.0] * len(angles)
    weights = list(weights)
    if len(weights) != len(angles):
        raise ValueError("angles/weights length mismatch")
    s = sum(w * math.sin(a) for a, w in zip(angles, weights))
    c = sum(w * math.cos(a) for a, w in zip(angles, weights))
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return 0.0
    return math.atan2(s, c)


@dataclass
class Pose2D:
    """Robot pose in the 2D map frame (metres, radians)."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def distance_to(self, other: "Pose2D") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


@dataclass
class Velocity2D:
    """Planar body twist: vx (forward), vy (left), wz (yaw rate)."""
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.vx, self.vy, self.wz)

    def is_zero(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


@dataclass
class PoseWithConfidence:
    """Pose plus a localization confidence in [0, 1]."""
    pose: Pose2D = field(default_factory=Pose2D)
    confidence: float = 1.0


def integrate_pose(pose: Pose2D, velocity: Velocity2D, dt: float) -> Pose2D:
    """Body-frame Euler integration of a twist (good enough for mock/odom)."""
    half = 0.5 * velocity.wz * dt
    c = math.cos(pose.yaw + half)
    s = math.sin(pose.yaw + half)
    dx = (velocity.vx * c - velocity.vy * s) * dt
    dy = (velocity.vx * s + velocity.vy * c) * dt
    return Pose2D(pose.x + dx, pose.y + dy, normalize_angle(pose.yaw + velocity.wz * dt))
