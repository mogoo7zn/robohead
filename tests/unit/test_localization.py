"""Localization fusion: odometry dead-reckoning + marker absolute fixes.

Uses MockMarkerDetector as ground truth renderer, FakeClock for time.
"""
import math

import pytest

from core.localization.camera import CameraParams
from core.localization.localization import Correction2D, Localization
from core.localization.marker_map import MarkerMap
from core.localization.pose_estimator import MarkerPoseEstimate, PoseEstimator
from core.model.pose import Pose2D
from core.perception.marker_detector import MockMarkerDetector
from core.utils.clock import FakeClock
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


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock(0.0)


@pytest.fixture
def detector(marker_map, camera) -> MockMarkerDetector:
    return MockMarkerDetector(marker_map, camera)


@pytest.fixture
def localization(marker_map, camera, clock) -> Localization:
    return Localization(PoseEstimator(marker_map, camera), clock=clock)


# ------------------------------------------------------------ Correction2D
def test_correction_identity():
    c = Correction2D()
    p = Pose2D(1.0, 2.0, 0.5)
    out = c.apply(p)
    assert out.x == pytest.approx(1.0)
    assert out.y == pytest.approx(2.0)
    assert out.yaw == pytest.approx(0.5)


def test_correction_maps_odom_to_map():
    c = Correction2D(x=0.5, y=-0.25, yaw=math.pi / 2)
    out = c.apply(Pose2D(1.0, 0.0, 0.0))
    # rotate (1,0) by +90deg -> (0,1), then translate
    assert out.x == pytest.approx(0.5)
    assert out.y == pytest.approx(0.75)
    assert out.yaw == pytest.approx(math.pi / 2)


def test_correction_blend_interpolates():
    a = Correction2D(0.0, 0.0, 0.0)
    b = Correction2D(1.0, 1.0, math.pi / 2)
    half = a.blend(b, 0.5)
    assert half.x == pytest.approx(0.5)
    assert half.y == pytest.approx(0.5)
    assert half.yaw == pytest.approx(math.pi / 4)


def test_correction_blend_wraps_angles():
    a = Correction2D(0, 0, math.pi - 0.1)
    b = Correction2D(0, 0, -math.pi + 0.1)
    half = a.blend(b, 0.5)
    # shortest path crosses pi, not zero
    assert half.yaw == pytest.approx(math.pi, abs=1e-9) or \
        half.yaw == pytest.approx(-math.pi, abs=1e-9)


def test_correction_roundtrip_property():
    """_correction_between(odom, m).apply(odom) == m for any poses."""
    from core.localization.localization import _correction_between
    cases = [
        (Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 2.0, 0.5)),
        (Pose2D(1.0, 2.0, 0.5), Pose2D(0.0, 0.0, 0.0)),
        (Pose2D(1.2, 0.65, 0.1), Pose2D(1.0, 0.5, 0.0)),
        (Pose2D(-0.5, 0.3, 2.9), Pose2D(2.5, -1.0, -2.8)),
    ]
    for odom, target in cases:
        corr = _correction_between(odom, target)
        out = corr.apply(odom)
        assert out.x == pytest.approx(target.x, abs=1e-9), (odom, target)
        assert out.y == pytest.approx(target.y, abs=1e-9), (odom, target)
        assert out.yaw == pytest.approx(target.yaw, abs=1e-9), (odom, target)


# ------------------------------------------------------------ reset + odom
def test_reset_sets_start_pose(localization):
    localization.reset(Pose2D(0.3, 0.5, 0.0))
    assert localization.pose.x == pytest.approx(0.3)
    assert localization.pose.y == pytest.approx(0.5)
    assert localization.pose.yaw == pytest.approx(0.0)
    assert localization.confidence == pytest.approx(0.5)  # unverified start


def test_odometry_moves_pose(localization):
    localization.reset(Pose2D(0.3, 0.5, 0.0))
    localization.update_odometry(Pose2D(0.6, 0.5, 0.0))
    assert localization.pose.x == pytest.approx(0.6)


def test_confidence_decays_without_markers(localization, clock):
    localization.reset(Pose2D(0.3, 0.5, 0.0))
    clock.advance(10.0)
    localization.update_odometry(Pose2D(0.4, 0.5, 0.0))
    # 0.5 - 0.02*10 = 0.3
    assert localization.confidence == pytest.approx(0.3, abs=1e-6)


def test_needs_relocalization_after_long_blind_period(localization, clock):
    localization.reset(Pose2D(0.3, 0.5, 0.0))
    clock.advance(10.0)
    localization.update_odometry(Pose2D(0.4, 0.5, 0.0))
    # 0.5 - 0.02*10 = 0.3 > 0.25 threshold
    assert not localization.needs_relocalization()

    clock.advance(20.0)
    localization.update_odometry(Pose2D(0.5, 0.5, 0.0))
    # 0.3 - 0.02*20 = -0.1 -> clamped to 0.0 < 0.25 threshold
    assert localization.needs_relocalization()


# ------------------------------------------------------------ marker fixes
def test_marker_fix_corrects_odometry_drift(localization, detector, clock):
    """Odom drifts while driving; a marker fix snaps the pose back to truth."""
    localization.reset(Pose2D(0.3, 0.5, 0.0))

    # robot truly at (1.0, 0.5, 0); odometry claims (1.2, 0.65, 0.1)
    true_pose = Pose2D(1.0, 0.5, 0.0)
    odom_pose = Pose2D(1.2, 0.65, 0.1)
    localization.update_odometry(odom_pose)
    assert localization.pose.x == pytest.approx(1.2)  # pre-fix: follows odom

    detector.bind_pose_provider(lambda: true_pose)
    observations = detector.detect(clock.now())
    assert observations
    assert localization.update_markers(observations) is True

    assert localization.pose.x == pytest.approx(true_pose.x, abs=0.02)
    assert localization.pose.y == pytest.approx(true_pose.y, abs=0.02)
    assert localization.pose.yaw == pytest.approx(true_pose.yaw, abs=0.02)
    assert localization.confidence > 0.5


def test_correction_persists_between_fixes(localization, detector, clock):
    """After a fix, further odometry follows the corrected frame."""
    localization.reset(Pose2D(0.3, 0.5, 0.0))

    true_pose = Pose2D(1.0, 0.5, 0.0)
    odom_pose = Pose2D(1.2, 0.65, 0.1)   # drift: +0.2 x, +0.15 y
    localization.update_odometry(odom_pose)
    detector.bind_pose_provider(lambda: true_pose)
    localization.update_markers(detector.detect(clock.now()))

    # drive on: same systematic drift
    localization.update_odometry(Pose2D(1.5, 0.85, 0.1))
    # correction removed the offset: x back on track, y partly corrected
    assert localization.pose.x == pytest.approx(1.3, abs=0.05)
    assert localization.pose.y == pytest.approx(0.7, abs=0.05)


def test_marker_fix_after_motion_tracks_truth(localization, detector, clock):
    """Full loop: drive, drift, fix, drive, drift, fix -> pose stays true."""
    localization.reset(Pose2D(0.3, 0.5, 0.0))
    for true_xy, odom_xy in (((0.6, 0.5), (0.62, 0.53)),
                             ((1.0, 0.5), (1.05, 0.58)),
                             ((1.4, 0.5), (1.48, 0.62))):
        clock.advance(1.0)
        true_pose = Pose2D(true_xy[0], true_xy[1], 0.0)
        localization.update_odometry(Pose2D(odom_xy[0], odom_xy[1], 0.0))
        detector.bind_pose_provider(lambda: true_pose)
        localization.update_markers(detector.detect(clock.now()))
        assert localization.pose.x == pytest.approx(true_pose.x, abs=0.03), true_xy
        assert localization.pose.y == pytest.approx(true_pose.y, abs=0.03), true_xy


def test_empty_marker_list_rejected(localization):
    localization.reset(Pose2D(0.3, 0.5, 0.0))
    assert localization.update_markers([]) is False


def test_inconsistent_markers_rejected(marker_map, camera, clock):
    """Two markers disagreeing by > tolerance must be rejected entirely."""

    class StubEstimator:
        def __init__(self):
            self.marker_map = marker_map

        def estimate_all(self, observations, prior=None):
            return [
                MarkerPoseEstimate("marker_2", Pose2D(1.0, 0.5, 0.0), 1.0, 1.0, 0.0),
                MarkerPoseEstimate("marker_3", Pose2D(1.0, 1.5, 0.0), 1.0, 1.0, 0.0),
            ]

    loc = Localization(StubEstimator(), clock=clock)
    loc.reset(Pose2D(0.3, 0.5, 0.0))
    loc.update_odometry(Pose2D(0.4, 0.5, 0.0))
    conf_before = loc.confidence

    from core.localization.pose_estimator import MarkerObservation
    import numpy as np
    corners = np.zeros((4, 2))
    ok = loc.update_markers([
        MarkerObservation("marker_2", corners, 0.0),
        MarkerObservation("marker_3", corners, 0.0),
    ])
    assert ok is False
    assert loc.confidence == conf_before   # no confidence boost on rejection
    assert loc.pose.x == pytest.approx(0.4)  # pose untouched


def test_consistent_markers_accepted(marker_map, clock):
    class StubEstimator:
        def __init__(self):
            self.marker_map = marker_map

        def estimate_all(self, observations, prior=None):
            return [
                MarkerPoseEstimate("marker_2", Pose2D(1.00, 0.50, 0.00), 1.0, 1.0, 0.0),
                MarkerPoseEstimate("marker_3", Pose2D(1.05, 0.55, 0.02), 1.0, 1.0, 0.0),
            ]

    loc = Localization(StubEstimator(), clock=clock)
    loc.reset(Pose2D(0.3, 0.5, 0.0))
    loc.update_odometry(Pose2D(0.9, 0.6, 0.0))

    from core.localization.pose_estimator import MarkerObservation
    import numpy as np
    corners = np.zeros((4, 2))
    ok = loc.update_markers([
        MarkerObservation("marker_2", corners, 0.0),
        MarkerObservation("marker_3", corners, 0.0),
    ])
    assert ok is True
    # weighted mean of the two estimates
    assert loc.pose.x == pytest.approx(1.025, abs=1e-6)
    assert loc.pose.y == pytest.approx(0.525, abs=1e-6)


def test_partial_correction_gain(marker_map, clock):
    """gain < 1: the fix moves the pose only partway to the marker pose."""

    class StubEstimator:
        def __init__(self):
            self.marker_map = marker_map

        def estimate_all(self, observations, prior=None):
            return [MarkerPoseEstimate("marker_2", Pose2D(1.0, 0.5, 0.0), 1.0, 1.0, 0.0)]

    loc = Localization(StubEstimator(),
                       config={"fusion": {"marker_correction_gain": 0.5}},
                       clock=clock)
    loc.reset(Pose2D(0.3, 0.5, 0.0))
    loc.update_odometry(Pose2D(0.6, 0.5, 0.0))

    from core.localization.pose_estimator import MarkerObservation
    import numpy as np
    corners = np.zeros((4, 2))
    loc.update_markers([MarkerObservation("marker_2", corners, 0.0)])
    # halfway between odom pose 0.6 and marker pose 1.0
    assert loc.pose.x == pytest.approx(0.8, abs=1e-6)

    loc.update_markers([MarkerObservation("marker_2", corners, 0.0)])
    assert loc.pose.x == pytest.approx(0.9, abs=1e-6)  # converging


def test_marker_fix_with_rotated_drift(localization, detector, clock):
    """Odom drift including heading error must also be corrected."""
    localization.reset(Pose2D(0.3, 0.5, 0.0))
    true_pose = Pose2D(1.6, 0.9, 1.1)          # turned ~63 deg
    odom_pose = Pose2D(1.75, 0.7, 1.35)        # drifted in x, y AND yaw
    localization.update_odometry(odom_pose)

    detector.bind_pose_provider(lambda: true_pose)
    observations = detector.detect(clock.now())
    assert observations
    assert localization.update_markers(observations)

    assert localization.pose.x == pytest.approx(true_pose.x, abs=0.03)
    assert localization.pose.y == pytest.approx(true_pose.y, abs=0.03)
    assert localization.pose.yaw == pytest.approx(true_pose.yaw, abs=0.03)


def test_markers_fresh_flag(localization, detector, clock):
    localization.reset(Pose2D(0.3, 0.5, 0.0))
    assert not localization.markers_fresh
    localization.update_odometry(Pose2D(1.0, 0.5, 0.0))
    detector.bind_pose_provider(lambda: Pose2D(1.0, 0.5, 0.0))
    localization.update_markers(detector.detect(clock.now()))
    assert localization.markers_fresh
    clock.advance(3.0)  # > marker_fresh_max_age (2.0)
    localization.update_odometry(Pose2D(1.05, 0.5, 0.0))
    assert not localization.markers_fresh
