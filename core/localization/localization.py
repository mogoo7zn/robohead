"""Localization — odometry dead-reckoning fused with absolute marker fixes.

Design (competition-proven odom/map split):
  * The MCU reports odometry in its own dead-reckoned frame (ODOMETRY msg).
  * Marker fixes give the *absolute* pose in the map frame.
  * We keep a correction transform T_map_odom and never touch the MCU's
    odom stream:  map_pose = T_map_odom ∘ odom_pose.
  * A marker fix updates (a blended version of) the correction, so
    dead-reckoning stays valid between fixes and never jumps backwards.

Confidence model (config/localization.yaml):
  * fresh consistent marker fix  -> high confidence
  * confidence decays per second while only odometry arrives
  * below `relocalize_threshold` the mission layer should RELOCALIZE
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from core.localization.pose_estimator import (
    MarkerObservation,
    PoseEstimator,
    weighted_mean_pose,
)
from core.model.pose import Pose2D, angle_diff, normalize_angle
from core.utils.clock import Clock, FakeClock
from core.utils.log import get_logger

log = get_logger("localization.fusion")

DEFAULT_CONF = {
    "marker_fresh_max_age": 2.0,
    "confidence_decay_rate": 0.02,
    "min_confidence": 0.0,
    "max_confidence": 1.0,
    "relocalize_threshold": 0.25,
    "marker_consistency_xy": 0.15,
    "marker_consistency_yaw": 0.175,
    "marker_correction_gain": 1.0,
}


@dataclass(frozen=True)
class Correction2D:
    """SE(2) offset of the odom frame inside the map frame."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def apply(self, odom: Pose2D) -> Pose2D:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(
            c * odom.x - s * odom.y + self.x,
            s * odom.x + c * odom.y + self.y,
            normalize_angle(odom.yaw + self.yaw),
        )

    def blend(self, target: "Correction2D", gain: float) -> "Correction2D":
        return Correction2D(
            self.x + gain * (target.x - self.x),
            self.y + gain * (target.y - self.y),
            normalize_angle(self.yaw + gain * angle_diff(target.yaw, self.yaw)),
        )


class Localization:
    """Fuses MCU odometry with marker-based absolute fixes."""

    def __init__(self, estimator: PoseEstimator,
                 config: dict | None = None,
                 clock: Clock | None = None) -> None:
        conf = dict(DEFAULT_CONF)
        if config:
            for section in ("confidence", "fusion"):
                conf.update(config.get(section, {}))
        self._conf = conf
        self._estimator = estimator
        self._clock = clock or FakeClock()

        self._correction = Correction2D()
        self._last_odom = Pose2D()
        self._pose = Pose2D()
        self._confidence = 0.0
        self._last_marker_time: float = -math.inf
        self._last_update_time: float = self._clock.now()
        self._last_fix_rejected = False

    # ------------------------------------------------------------- lifecycle
    def reset(self, pose: Pose2D, odom: Pose2D | None = None) -> None:
        """Initialize at a known pose (e.g. start pose / manual placement)."""
        odom = odom if odom is not None else pose
        self._last_odom = odom
        target = _correction_between(odom, pose)
        self._correction = target
        self._pose = pose
        self._confidence = self._conf["max_confidence"] * 0.5  # unverified start
        self._last_marker_time = -math.inf
        self._last_update_time = self._clock.now()

    # ------------------------------------------------------------- inputs
    def update_odometry(self, odom: Pose2D) -> Pose2D:
        """New MCU ODOMETRY sample: apply correction, decay confidence."""
        now = self._clock.now()
        dt = max(0.0, now - self._last_update_time)
        self._last_update_time = now

        self._last_odom = odom
        self._pose = self._correction.apply(odom)

        decay = float(self._conf["confidence_decay_rate"]) * dt
        self._confidence = max(float(self._conf["min_confidence"]),
                               self._confidence - decay)
        return self._pose

    def update_markers(self, observations: list[MarkerObservation]) -> bool:
        """New marker frame. Returns True if a fix was accepted."""
        if not observations:
            return False
        now = self._clock.now()
        self._last_update_time = now

        estimates = self._estimator.estimate_all(observations,
                                                 prior=self._pose)
        if not estimates:
            self._last_fix_rejected = True
            return False

        if not self._consistent(estimates):
            self._last_fix_rejected = True
            log.warning("marker fix rejected: estimates inconsistent (%d markers)",
                        len(estimates))
            return False

        fused = weighted_mean_pose(estimates)
        target = _correction_between(self._last_odom, fused)
        gain = float(self._conf["marker_correction_gain"])
        self._correction = self._correction.blend(target, gain)
        self._pose = self._correction.apply(self._last_odom)

        quality = max(e.quality for e in estimates)
        self._confidence = min(float(self._conf["max_confidence"]),
                               max(self._confidence, quality))
        self._last_marker_time = now
        self._last_fix_rejected = False
        return True

    def update_imu_yaw(self, imu_yaw: float, gain: float = 0.0) -> None:
        """Optional heading aid. With gain 0 (default) this only records
        the sample; the mock IMU is unbiased so odometry already carries it."""
        # kept for interface parity with the real IMU stream (0x83)
        _ = imu_yaw, gain

    # ------------------------------------------------------------- outputs
    @property
    def pose(self) -> Pose2D:
        return self._pose

    @property
    def confidence(self) -> float:
        return self._confidence

    @property
    def correction(self) -> Correction2D:
        return self._correction

    @property
    def last_marker_time(self) -> float:
        return self._last_marker_time

    @property
    def markers_fresh(self) -> bool:
        age = self._clock.now() - self._last_marker_time
        return age <= float(self._conf["marker_fresh_max_age"])

    def needs_relocalization(self) -> bool:
        return self._confidence < float(self._conf["relocalize_threshold"])

    # ------------------------------------------------------------- internal
    def _consistent(self, estimates) -> bool:
        """Pairwise spread check across marker estimates."""
        xy_tol = float(self._conf["marker_consistency_xy"])
        yaw_tol = float(self._conf["marker_consistency_yaw"])
        for i in range(len(estimates)):
            for j in range(i + 1, len(estimates)):
                d = estimates[i].pose.distance_to(estimates[j].pose)
                dyaw = abs(angle_diff(estimates[i].pose.yaw,
                                      estimates[j].pose.yaw))
                if d > xy_tol or dyaw > yaw_tol:
                    return False
        return True


def _correction_between(odom: Pose2D, map_pose: Pose2D) -> Correction2D:
    """Correction c such that c.apply(odom) == map_pose."""
    theta = angle_diff(map_pose.yaw, odom.yaw)
    c, s = math.cos(theta), math.sin(theta)
    x = map_pose.x - (c * odom.x - s * odom.y)
    y = map_pose.y - (s * odom.x + c * odom.y)
    return Correction2D(x, y, theta)
