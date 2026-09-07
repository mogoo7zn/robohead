"""MarkerDetector — interface + mock backend for the localization camera.

Real backend (Pi): AprilTag/ArUco detection on the localization camera
topic, publishing the same MarkerObservation list. High-level code only
ever sees this interface, so swapping mock -> real is a one-line change.
"""
from __future__ import annotations

from typing import Protocol

import numpy as np

from core.localization.camera import CameraParams
from core.localization.geometry import (
    project_points,
    se3_compose,
    se3_from_pose2d,
    se3_inverse,
)
from core.localization.marker_map import MarkerMap
from core.localization.pose_estimator import MarkerObservation
from core.model.pose import Pose2D
from core.utils.log import get_logger

log = get_logger("perception.marker_detector")


class MarkerDetector(Protocol):
    """Anything that yields marker observations for one camera frame."""

    def detect(self, timestamp: float) -> list[MarkerObservation]:
        """Return all markers visible in the current frame."""
        ...


class MockMarkerDetector:
    """Renders markers exactly as the localization camera would see them.

    Reads the ground-truth robot pose (from SimWorld) and projects every
    field marker through the camera model. Visible = all 4 corners in
    front of the camera and inside the image. Optional pixel noise
    exercises the estimator's robustness without hardware.
    """

    def __init__(self, marker_map: MarkerMap, camera: CameraParams,
                 pose_provider=None, pixel_noise: float = 0.0,
                 seed: int | None = None) -> None:
        self._map = marker_map
        self._camera = camera
        self._pose_provider = pose_provider or (lambda: Pose2D())
        self._pixel_noise = float(pixel_noise)
        self._rng = np.random.default_rng(seed)

    def bind_pose_provider(self, provider) -> None:
        """Attach a callable returning the current ground-truth pose."""
        self._pose_provider = provider

    def detect(self, timestamp: float) -> list[MarkerObservation]:
        robot_pose = self._pose_provider()
        t_map_base = se3_from_pose2d(robot_pose)
        # T_cam_marker = inv(T_base_cam) @ inv(T_map_base) @ T_map_marker
        t_cam_base = se3_inverse(self._camera.t_base_cam)
        t_base_map = se3_inverse(t_map_base)

        observations: list[MarkerObservation] = []
        w, h = self._camera.width, self._camera.height
        for marker in self._map.all():
            t_cam_marker = se3_compose(
                t_cam_base, se3_compose(t_base_map, marker.t_map_marker))
            corners = project_points(self._camera.k, t_cam_marker,
                                    marker.corners_3d)
            if len(corners) != 4:
                continue  # behind camera
            if (corners[:, 0].min() < 0 or corners[:, 0].max() > w
                    or corners[:, 1].min() < 0 or corners[:, 1].max() > h):
                continue  # partially out of frame -> not a valid detection
            if self._pixel_noise > 0:
                corners = corners + self._rng.normal(
                    0.0, self._pixel_noise, size=corners.shape)
            observations.append(MarkerObservation(
                marker_id=marker.id,
                corners_px=corners,
                timestamp=timestamp,
            ))
        return observations
