"""Clock abstraction and pose math utilities."""
import math
import time

import pytest

from core.model.pose import (
    Pose2D,
    Velocity2D,
    circular_mean,
    integrate_pose,
    normalize_angle,
)
from core.utils.clock import FakeClock, RealClock


def test_fake_clock_advance():
    clock = FakeClock(start=100.0)
    assert clock.now() == 100.0
    clock.sleep(0.5)
    assert clock.now() == 100.5
    clock.advance(10.0)
    assert clock.now() == 110.5


def test_fake_clock_sleep_does_not_block():
    clock = FakeClock()
    t0 = time.monotonic()
    clock.sleep(360.0)  # a whole match — instantly
    assert time.monotonic() - t0 < 0.01


def test_real_clock_roughly_monotonic():
    clock = RealClock()
    a = clock.now()
    assert clock.now() >= a


def test_real_clock_cannot_advance():
    clock = RealClock()
    with pytest.raises(NotImplementedError):
        clock.advance(1.0)


def test_normalize_angle():
    assert normalize_angle(0.0) == 0.0
    assert normalize_angle(2 * math.pi) == pytest.approx(0.0)
    assert abs(normalize_angle(3 * math.pi)) == pytest.approx(math.pi)
    assert abs(normalize_angle(-3 * math.pi)) == pytest.approx(math.pi)
    # -pi and +pi are the same angle; either representation is fine
    assert abs(normalize_angle(-math.pi)) == pytest.approx(math.pi)


def test_circular_mean_wraps_correctly():
    # plain average of 350deg and 10deg would wrongly be 180deg
    a = math.radians(350)
    b = math.radians(10)
    assert math.degrees(circular_mean([a, b])) <= 0.0 + 1e-6


def test_circular_mean_weighted():
    a = math.radians(10)
    b = math.radians(30)
    mean = circular_mean([a, b], weights=[3.0, 1.0])
    assert 10 < math.degrees(mean) < 16.5  # pulled toward the weight-3 angle


def test_integrate_pose_straight():
    p = Pose2D(0, 0, 0)
    p2 = integrate_pose(p, Velocity2D(vx=1.0), dt=1.0)
    assert p2.x == 1.0 and abs(p2.y) < 1e-9 and p2.yaw == 0.0


def test_integrate_pose_rotation():
    p = Pose2D(0, 0, 0)
    p2 = integrate_pose(p, Velocity2D(wz=math.pi / 2), dt=1.0)
    assert p2.yaw == pytest.approx(math.pi / 2)


def test_integrate_pose_arc():
    p = Pose2D(0, 0, 0)
    p2 = integrate_pose(p, Velocity2D(vx=1.0, wz=math.pi), dt=1.0)
    # half-turn arc: ends at heading pi, displaced to the robot's left (+y)
    assert p2.y > 0.3
    assert p2.yaw == pytest.approx(math.pi)
