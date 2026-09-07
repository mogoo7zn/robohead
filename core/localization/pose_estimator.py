"""PoseEstimator — robot pose from marker observations.

Pipeline per observation:
  1. solve PnP for T_cam_marker from the 4 image corners
  2. chain transforms:  T_map_base = T_map_marker @ inv(T_cam_marker) @ inv(T_base_cam)
  3. project down to planar Pose2D

Hardware note: works with or without OpenCV (numpy homography fallback),
so the same code runs on Mac and on the Pi.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from core.localization.camera import CameraParams
from core.localization.geometry import (
    se3_compose,
    se3_inverse,
    pose2d_from_se3,
    solve_pnp_candidates,
)
from core.localization.marker_map import MarkerMap
from core.model.pose import Pose2D, angle_diff
from core.utils.log import get_logger

log = get_logger("localization.pose_estimator")

# distance-based quality model: full trust up to NEAR, zero beyond FAR
NEAR_DISTANCE = 0.5   # m
FAR_DISTANCE = 4.0     # m


@dataclass(frozen=True)
class MarkerObservation:
    """One detected marker: id + pixel corners (TL, TR, BR, BL)."""

    marker_id: str
    corners_px: np.ndarray            # (4, 2) float
    timestamp: float = 0.0
    camera: str = "localization"


@dataclass(frozen=True)
class MarkerPoseEstimate:
    """Absolute robot pose in map frame derived from one marker."""

    marker_id: str
    pose: Pose2D
    distance: float                    # camera-to-marker distance, m
    quality: float                    # 0..1, distance-based trust
    timestamp: float = 0.0


class PoseEstimator:
    """Turns MarkerObservations into absolute robot pose estimates."""

    def __init__(self, marker_map: MarkerMap, camera: CameraParams) -> None:
        self._map = marker_map
        self._camera = camera

    @property
    def marker_map(self) -> MarkerMap:
        return self._map

    def estimate(self, obs: MarkerObservation,
                 prior: Pose2D | None = None) -> MarkerPoseEstimate | None:
        """Robot pose implied by one marker observation.

        `prior` (e.g. the current odometry-corrected estimate) resolves
        the planar-flip ambiguity of near-frontal markers: of the PnP
        candidates, the one closest to the prior is chosen. Without a
        prior the lowest-reprojection-error candidate is used.
        """
        marker = self._map.get(obs.marker_id)
        if marker is None:
            log.warning("unknown marker id in observation: %s", obs.marker_id)
            return None
        corners = np.asarray(obs.corners_px, dtype=float)
        if corners.shape != (4, 2):
            log.warning("marker %s: expected 4 corners, got shape %s",
                        obs.marker_id, corners.shape)
            return None

        candidates = solve_pnp_candidates(
            self._camera.k, self._camera.distortion,
            marker.corners_3d, corners)
        if not candidates:
            log.warning("marker %s: PnP failed (degenerate geometry)", obs.marker_id)
            return None

        best_t = None
        best_pose = None
        best_score = None
        for t_cam_marker in candidates:
            # T_map_base = T_map_marker @ inv(T_cam_marker) @ inv(T_base_cam)
            t_map_base = se3_compose(
                marker.t_map_marker,
                se3_compose(se3_inverse(t_cam_marker),
                            se3_inverse(self._camera.t_base_cam)))
            pose = pose2d_from_se3(t_map_base)
            if prior is not None and len(candidates) > 1:
                score = (pose.distance_to(prior)
                         + abs(angle_diff(pose.yaw, prior.yaw)))
                if best_score is None or score < best_score:
                    best_score, best_t, best_pose = score, t_cam_marker, pose
            elif best_t is None:
                best_t, best_pose = t_cam_marker, pose

        assert best_t is not None and best_pose is not None
        distance = float(np.linalg.norm(best_t[:3, 3]))
        return MarkerPoseEstimate(
            marker_id=obs.marker_id,
            pose=best_pose,
            distance=distance,
            quality=distance_quality(distance),
            timestamp=obs.timestamp,
        )

    def estimate_all(self, observations: list[MarkerObservation],
                     prior: Pose2D | None = None
                     ) -> list[MarkerPoseEstimate]:
        out = []
        for obs in observations:
            est = self.estimate(obs, prior=prior)
            if est is not None:
                out.append(est)
        return out


def distance_quality(distance: float) -> float:
    """Trust model: 1.0 near, linearly down to 0.0 at FAR_DISTANCE."""
    if distance <= NEAR_DISTANCE:
        return 1.0
    if distance >= FAR_DISTANCE:
        return 0.0
    return 1.0 - (distance - NEAR_DISTANCE) / (FAR_DISTANCE - NEAR_DISTANCE)


def weighted_mean_pose(estimates: list[MarkerPoseEstimate]) -> Pose2D:
    """Quality-weighted planar mean (circular mean for yaw)."""
    if not estimates:
        raise ValueError("weighted_mean_pose of empty list")
    w_total = sum(e.quality for e in estimates)
    if w_total <= 1e-9:
        # all-zero qualities: fall back to plain mean
        w_total = len(estimates)
        weights = [1.0] * len(estimates)
    else:
        weights = [e.quality for e in estimates]
    x = sum(e.pose.x * w for e, w in zip(estimates, weights)) / w_total
    y = sum(e.pose.y * w for e, w in zip(estimates, weights)) / w_total
    s = sum(w * math.sin(e.pose.yaw) for e, w in zip(estimates, weights))
    c = sum(w * math.cos(e.pose.yaw) for e, w in zip(estimates, weights))
    yaw = math.atan2(s, c) if (abs(s) > 1e-12 or abs(c) > 1e-12) else estimates[0].pose.yaw
    return Pose2D(x, y, yaw)
