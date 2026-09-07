"""SimWorld — a deterministic 2D simulation of robot + field for mock mode.

Simulates:
  * chassis kinematics (velocity / line-follow commands)
  * wheel odometry with optional drift
  * the dedicated line sensor (lateral error w.r.t. line segments)
  * a simple STM32-side line-follow PID controller
  * the cross-slide + gripper with realistic timing
  * start button

It backs FakeSTM32 so the whole mission runs without any hardware.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from core.model.enums import GripperState
from core.model.pose import Pose2D, normalize_angle
from core.utils.clock import Clock


@dataclass
class LineSegment:
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def length(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    def point_at(self, s: float) -> tuple[float, float]:
        """Point at arc-length s along the segment."""
        if self.length <= 1e-9:
            return (self.x1, self.y1)
        t = max(0.0, min(1.0, s / self.length))
        return (self.x1 + (self.x2 - self.x1) * t,
                self.y1 + (self.y2 - self.y1) * t)

    def project(self, x: float, y: float) -> tuple[float, float]:
        """Return (arc_length_of_projection, signed_lateral_error)."""
        dx, dy = self.x2 - self.x1, self.y2 - self.y1
        L2 = dx * dx + dy * dy
        if L2 <= 1e-12:
            return 0.0, math.hypot(x - self.x1, y - self.y1)
        t = ((x - self.x1) * dx + (y - self.y1) * dy) / L2
        t_clamped = max(0.0, min(1.0, t))
        lateral = (x - self.x1) * dy - (y - self.y1) * dx  # signed cross product
        return t_clamped * math.sqrt(L2), lateral


@dataclass
class SimBlock:
    block_type: str            # "ORANGE" | "PURPLE"
    x: float
    y: float
    yaw: float = 0.0
    grabbed: bool = False
    placed: bool = False


@dataclass
class SimWorld:
    """Field + robot state. FakeSTM32 drives this; perception mocks read it."""
    robot_pose: Pose2D = field(default_factory=Pose2D)
    # hardware command state
    velocity_cmd: tuple[float, float, float] = (0.0, 0.0, 0.0)
    stopped: bool = True

    # line follower (STM32 side)
    line_segments: list[LineSegment] = field(default_factory=list)
    line_follow_active: bool = False
    line_follow_speed: float = 0.3
    line_pid: tuple[float, float, float] = (2.0, 0.0, 0.05)
    _line_integral: float = 0.0
    _prev_line_error: float = 0.0
    line_detected: bool = False
    line_error: float = 0.0
    line_confidence: float = 0.0
    line_lost_time: float = 0.0

    # odometry (what the MCU reports — may drift from robot_pose)
    odom_x: float = 0.0
    odom_y: float = 0.0
    odom_yaw: float = 0.0
    odom_drift_rate: float = 0.0        # fraction of distance added as error
    imu_yaw: float = 0.0
    _odom_distance: float = 0.0

    # manipulator
    manip_x_mm: float = 0.0
    manip_z_mm: float = 0.0
    manip_homed: bool = False
    manip_moving: bool = False
    manip_gripper: GripperState = GripperState.OPEN
    manip_grip_detected: bool = False
    _manip_target_x: float = 0.0
    _manip_target_z: float = 0.0
    _manip_speed_x: float = 60.0
    _manip_speed_z: float = 40.0
    _gripper_closing: bool = False
    _gripper_until: float = 0.0
    axis_limits: dict = field(default_factory=lambda: {
        "X": (0.0, 300.0), "Z": (0.0, 200.0)})

    # environment
    blocks: list[SimBlock] = field(default_factory=list)
    start_button: bool = False
    held_block: SimBlock | None = None     # block currently in the gripper
    place_radius: float = 0.30            # m: tower within this when releasing

    # config knobs
    line_sensor_half_width: float = 0.06   # |lateral error| below this = on line
    line_lost_after: float = 0.5           # s off line -> controller LOST
    line_creep_speed: float = 0.08         # m/s creep while searching for the line
    manip_grab_radius: float = 0.25        # m: block within this + gripper closed -> grabbed
                                             # (covers the cross-slide reach ~0.2 m + margin)

    def reset(self, pose: Pose2D) -> None:
        self.robot_pose = Pose2D(pose.x, pose.y, pose.yaw)
        self.velocity_cmd = (0.0, 0.0, 0.0)
        self.stopped = True
        self.line_follow_active = False
        self.odom_x, self.odom_y, self.odom_yaw = pose.x, pose.y, pose.yaw
        self.imu_yaw = pose.yaw
        self._odom_distance = 0.0
        self._line_integral = 0.0
        self._prev_line_error = 0.0

    # ------------------------------------------------------------- physics
    def step(self, dt: float, clock: Clock) -> None:
        self._step_manipulator(dt, clock)

        vx, vy, wz = 0.0, 0.0, 0.0
        if self.line_follow_active and self.line_detected:
            # STM32-side line-follow closed loop: pure pursuit.
            # A pure P-law on lateral error is an (almost) undamped
            # oscillator (lambda^2 = -v*kp) that overshoots the sensor
            # width; steering toward a lookahead point on the line is
            # stable for any entry combination of lateral + heading
            # error. The same structure is what the firmware must run.
            wz = self._pursuit_wz()
            vx, vy = self.line_follow_speed, 0.0
        elif self.line_follow_active and not self.line_detected:
            # Line lost: creep forward and steer back toward the reported
            # lateral offset until the sensor re-acquires the line.
            vx = min(self.line_creep_speed, max(self.line_follow_speed, 0.01))
            wz = max(-0.8, min(0.8, 3.0 * self.line_error))
        elif not self.stopped:
            vx, vy, wz = self.velocity_cmd

        # integrate robot
        if abs(vx) + abs(vy) + abs(wz) > 1e-9:
            half = 0.5 * wz * dt
            c, s = math.cos(self.robot_pose.yaw + half), math.sin(self.robot_pose.yaw + half)
            self.robot_pose.x += (vx * c - vy * s) * dt
            self.robot_pose.y += (vx * s + vy * c) * dt
            self.robot_pose.yaw = normalize_angle(self.robot_pose.yaw + wz * dt)
            self._odom_distance += math.hypot(vx, vy) * dt

        # update line sensor from new pose
        self._update_line_sensor(dt)

        # odometry: follows the true motion plus configurable drift
        if abs(vx) + abs(vy) + abs(wz) > 1e-9:
            drift = self.odom_drift_rate * math.hypot(vx, vy) * dt
            oc, os_ = math.cos(self.odom_yaw + half), math.sin(self.odom_yaw + half)
            self.odom_x += (vx * oc - vy * os_) * dt + drift * 0.5
            self.odom_y += (vx * os_ + vy * oc) * dt + drift * 0.3
            self.odom_yaw = normalize_angle(self.odom_yaw + wz * dt + drift * 0.2)
            self.imu_yaw = normalize_angle(self.robot_pose.yaw)  # IMU: unbiased heading

    def _pursuit_wz(self) -> float:
        """Yaw rate steering to a lookahead point on the tracked segment."""
        seg = self._tracked_segment
        if seg is None:
            return 0.0
        s_along, _ = seg.project(self.robot_pose.x, self.robot_pose.y)
        lookahead = 0.15                                   # m ahead on the line
        s_target = s_along + lookahead * self._tracked_dir
        lx, ly = seg.point_at(s_target)
        desired = math.atan2(ly - self.robot_pose.y, lx - self.robot_pose.x)
        err = math.atan2(math.sin(desired - self.robot_pose.yaw),
                         math.cos(desired - self.robot_pose.yaw))
        kp = self.line_pid[0]
        return max(-1.5, min(1.5, kp * err))

    def _update_line_sensor(self, dt: float) -> None:
        """Pick the segment closest to the robot and report an egocentric
        lateral error (positive = robot right of the travel direction).

        Overlapping reverse-direction segments of the same physical line
        must not flip the error sign, so the lateral value is referred to
        the robot's heading: segments anti-aligned with the heading get
        their sign flipped before comparison.
        """
        best_lateral = None
        best_seg, best_dir = None, 1
        for seg in self.line_segments:
            _, lateral = seg.project(self.robot_pose.x, self.robot_pose.y)
            dx, dy = seg.x2 - seg.x1, seg.y2 - seg.y1
            along = (math.cos(self.robot_pose.yaw) * dx
                     + math.sin(self.robot_pose.yaw) * dy)
            direction = 1 if along >= 0.0 else -1
            if along < 0.0:
                lateral = -lateral
            if best_lateral is None or abs(lateral) < abs(best_lateral):
                best_lateral = lateral
                best_seg, best_dir = seg, direction
        self._tracked_segment = best_seg
        self._tracked_dir = best_dir
        if best_lateral is None:
            self.line_detected = False
            self.line_error = 0.0
            self.line_confidence = 0.0
            return
        self.line_error = best_lateral
        within = abs(best_lateral) <= self.line_sensor_half_width
        if within:
            self.line_detected = True
            self.line_confidence = max(0.0, 1.0 - abs(best_lateral) / self.line_sensor_half_width)
            self.line_lost_time = 0.0
        else:
            self.line_confidence = max(0.0, 1.0 - abs(best_lateral) / (2 * self.line_sensor_half_width))
            if self.line_detected:
                self.line_lost_time += dt
                if self.line_lost_time >= self.line_lost_after:
                    self.line_detected = False
                    self.line_lost_time = 0.0

    # ------------------------------------------------------------- manipulator
    def manip_home(self) -> None:
        self._manip_target_x = 0.0
        self._manip_target_z = 0.0
        self.manip_moving = True

    def manip_move_axis(self, axis: str, position_mm: float, speed: float) -> None:
        lo, hi = self.axis_limits.get(axis, (0.0, 300.0))
        position_mm = max(lo, min(hi, position_mm))
        if axis == "X":
            self._manip_target_x = position_mm
            if speed > 0:
                self._manip_speed_x = speed
        else:
            self._manip_target_z = position_mm
            if speed > 0:
                self._manip_speed_z = speed
        self.manip_moving = True

    def manip_set_gripper(self, close: bool, duration: float, clock: Clock) -> None:
        self._gripper_closing = close
        self._gripper_until = clock.now() + duration

    def _step_manipulator(self, dt: float, clock: Clock) -> None:
        # axis motion
        for attr, target, speed in (
            ("manip_x_mm", self._manip_target_x, self._manip_speed_x),
            ("manip_z_mm", self._manip_target_z, self._manip_speed_z),
        ):
            current = getattr(self, attr)
            step = speed * dt
            if abs(target - current) <= step:
                setattr(self, attr, target)
            else:
                setattr(self, attr, current + math.copysign(step, target - current))
        arrived = (abs(self.manip_x_mm - self._manip_target_x) < 1e-6 and
                   abs(self.manip_z_mm - self._manip_target_z) < 1e-6)
        homing = (self._manip_target_x == 0.0 and self._manip_target_z == 0.0)
        if arrived:
            self.manip_moving = False
            if homing and not self.manip_homed:
                self.manip_homed = True

        # gripper timing
        if self._gripper_closing is not None and clock.now() >= self._gripper_until:
            if self._gripper_closing:
                self.manip_gripper = GripperState.CLOSED
                grabbed = self.grab_block()
                self.manip_grip_detected = grabbed is not None
                if grabbed is not None:
                    self.manip_gripper = GripperState.HOLDING
                    self.held_block = grabbed
            else:
                self.manip_gripper = GripperState.OPEN
                released = self.place_block()
                self.manip_grip_detected = False
                if released is None and self.held_block is not None:
                    # opened in free space: block is dropped/lost
                    self.held_block = None
            self._gripper_closing = None  # type: ignore[assignment]

    def grab_block(self) -> SimBlock | None:
        """When grip closes: pick up the nearest in-range block."""
        best = None
        best_d = self.manip_grab_radius
        for b in self.blocks:
            if b.grabbed or b.placed:
                continue
            d = math.hypot(b.x - self.robot_pose.x, b.y - self.robot_pose.y)
            if d <= best_d:
                best, best_d = b, d
        if best is not None:
            best.grabbed = True
        return best

    def place_block(self) -> SimBlock | None:
        """When the gripper opens while holding: drop the block.

        The block is 'placed' at the robot's current position (the skill
        layer guarantees the robot is over a tower). Returns the block,
        or None if the gripper was empty.
        """
        if self.held_block is None:
            return None
        block = self.held_block
        block.placed = True
        block.grabbed = False
        # dropped over the cross-slide centre: 0.15 m along the heading
        block.x = self.robot_pose.x + 0.15 * math.cos(self.robot_pose.yaw)
        block.y = self.robot_pose.y + 0.15 * math.sin(self.robot_pose.yaw)
        self.held_block = None
        return block
