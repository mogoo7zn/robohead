"""PoseEstimator + MockMarkerDetector roundtrip.

The mock detector renders markers from a ground-truth pose through the
camera model; the estimator must recover that pose. This validates the
whole geometry chain (extrinsics, intrinsics, PnP, marker conventions)
in both directions.
"""
import math

import numpy as np
import pytest

from core.localization.camera import CameraParams
from core.localization.marker_map import MarkerMap
from core.localization.pose_estimator import (
    MarkerObservation,
    PoseEstimator,
    distance_quality,
    weighted_mean_pose,
)
from core.model.pose import Pose2D
from core.perception.marker_detector import MockMarkerDetector
from core.utils.config import load_yaml


@pytest.fixture(scope="module")
def mock_field() -> dict:
    return load_yaml("config/mock_field.yaml")


@pytest.fixture(scope="module")
def camera(mock_field) -> CameraParams:
    return CameraParams.from_config(mock_field["localization_camera"])


@pytest.fixture(scope="module")
def marker_map(mock_field) -> MarkerMap:
    return MarkerMap.from_config(mock_field)


@pytest.fixture(scope="module")
def detector(marker_map, camera) -> MockMarkerDetector:
    return MockMarkerDetector(marker_map, camera)


@pytest.fixture(scope="module")
def estimator(marker_map, camera) -> PoseEstimator:
    return PoseEstimator(marker_map, camera)


def _roundtrip(detector, estimator, pose, timestamp=0.0):
    detector.bind_pose_provider(lambda: pose)
    observations = detector.detect(timestamp)
    estimates = estimator.estimate_all(observations)
    return observations, estimates


def test_start_pose_sees_marker_1(detector, estimator):
    """Robot at the start pose must see start_area marker_1 (it faces -x)."""
    pose = Pose2D(0.3, 0.5, 0.0)
    observations, estimates = _roundtrip(detector, estimator, pose)
    ids = [o.marker_id for o in observations]
    assert "marker_1" in ids
    assert estimates, "at least one estimate from visible markers"


def test_roundtrip_recovers_pose(detector, estimator):
    """detected markers -> estimated pose == ground truth."""
    for pose in (Pose2D(0.3, 0.5, 0.0),
                 Pose2D(1.2, 0.55, 0.15),
                 Pose2D(2.0, 0.5, -0.3),
                 Pose2D(2.3, 1.2, 1.4)):
        _, estimates = _roundtrip(detector, estimator, pose)
        assert estimates, f"no visible markers from {pose}"
        fused = weighted_mean_pose(estimates)
        assert fused.x == pytest.approx(pose.x, abs=0.02), f"x at {pose}"
        assert fused.y == pytest.approx(pose.y, abs=0.02), f"y at {pose}"
        assert fused.yaw == pytest.approx(pose.yaw, abs=0.02), f"yaw at {pose}"


def test_roundtrip_with_pixel_noise(detector):
    """Small corner noise must not break the estimate (robustness).

    A prior pose (odometry-like, ~10 cm off) is supplied to resolve the
    planar-flip ambiguity of near-frontal markers, exactly as the fusion
    layer does on the real robot.
    """
    noisy = MockMarkerDetector(detector._map, detector._camera,
                               pixel_noise=0.5, seed=7)
    true_pose = Pose2D(1.0, 0.5, 0.0)
    noisy.bind_pose_provider(lambda: true_pose)
    estimator = PoseEstimator(detector._map, detector._camera)
    observations = noisy.detect(0.0)
    assert observations
    prior = Pose2D(1.05, 0.55, 0.08)  # rough odometry prior
    estimates = estimator.estimate_all(observations, prior=prior)
    assert estimates
    fused = weighted_mean_pose(estimates)
    assert fused.x == pytest.approx(true_pose.x, abs=0.03)
    assert fused.y == pytest.approx(true_pose.y, abs=0.03)
    assert fused.yaw == pytest.approx(true_pose.yaw, abs=0.03)


def test_marker_behind_camera_not_visible(detector):
    """Facing -x from the start, markers ahead (x>0.7) are behind the camera."""
    detector.bind_pose_provider(lambda: Pose2D(0.3, 0.5, 0.0))
    forward = {o.marker_id for o in detector.detect(0.0)}
    assert forward  # sees some markers looking +x

    detector.bind_pose_provider(lambda: Pose2D(0.3, 0.5, math.pi))
    backward = {o.marker_id for o in detector.detect(0.0)}
    assert backward == set()  # all markers are behind the camera


def test_turning_back_sees_other_markers(detector):
    detector.bind_pose_provider(lambda: Pose2D(0.3, 0.5, 0.0))
    forward = {o.marker_id for o in detector.detect(0.0)}
    detector.bind_pose_provider(lambda: Pose2D(0.3, 0.5, math.pi))
    backward = {o.marker_id for o in detector.detect(0.0)}
    assert forward != backward


def test_unknown_marker_id_rejected(estimator):
    obs = MarkerObservation(
        marker_id="marker_99",
        corners_px=np.array([[100, 100], [200, 100], [200, 200], [100, 200]]),
        timestamp=0.0)
    assert estimator.estimate(obs) is None


def test_bad_corner_count_rejected(estimator):
    obs = MarkerObservation(
        marker_id="marker_1",
        corners_px=np.array([[100, 100], [200, 100]]),
        timestamp=0.0)
    assert estimator.estimate(obs) is None


def test_distance_quality_model():
    assert distance_quality(0.3) == pytest.approx(1.0)
    assert distance_quality(0.5) == pytest.approx(1.0)
    assert distance_quality(4.0) == pytest.approx(0.0)
    assert distance_quality(5.0) == pytest.approx(0.0)
    assert 0.0 < distance_quality(2.0) < 1.0


def test_estimate_carries_distance_and_quality(detector, estimator):
    detector.bind_pose_provider(lambda: Pose2D(0.3, 0.5, 0.0))
    observations = detector.detect(0.0)
    est = estimator.estimate(observations[0])
    assert est is not None
    assert est.distance > 0.1
    assert 0.0 <= est.quality <= 1.0


def test_estimator_with_cv2_backend(detector):
    """cv2.solvePnP backend must agree with the pure-numpy fallback."""
    import core.localization.geometry as geom

    detector.bind_pose_provider(lambda: Pose2D(0.9, 0.55, 0.2))
    obs = detector.detect(0.0)
    assert obs
    marker = detector._map.get(obs[0].marker_id)

    t_cv2 = geom.solve_pnp_cv2(detector._camera.k,
                               detector._camera.distortion,
                               marker.corners_3d,
                               obs[0].corners_px)
    if t_cv2 is None:  # cv2 not installed on this host
        pytest.skip("cv2 unavailable")
    t_np = geom.solve_pnp_homography(detector._camera.k,
                                     marker.corners_3d,
                                     obs[0].corners_px)
    # translations must match closely; rotation may differ at the poles
    assert np.allclose(t_cv2[:3, 3], t_np[:3, 3], atol=0.02)
