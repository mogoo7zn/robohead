"""AlignmentController — fine servoing onto a block with the block camera.

Closes the loop:  pixel error -> body twist -> chassis. Sub-states follow
AlignmentState; the skill result maps onto SkillStatus for the HFSM.

Image geometry (block camera, pitch down):
  * u > target_u : block right of grab point -> robot strafes right (vy<0)
  * v > target_v  : block low in image (close)  -> robot backs up (vx<0)
  * angle         : robot CCW of block          -> robot turns CW (wz<0)
"""
from __future__ import annotations

from dataclasses import dataclass

from core.model.enums import AlignmentState, BlockType, SkillStatus
from core.model.pose import Velocity2D
from core.perception.block_detector import BlockDetection
from core.utils.clock import Clock, FakeClock
from core.utils.log import get_logger

log = get_logger("skill.alignment")

DEFAULT_CONFIG = {
    "target_u": 320.0,
    "target_v": 240.0,
    "target_angle": 0.0,
    "u_tolerance": 15.0,
    "v_tolerance": 12.0,
    "angle_tolerance": 0.12,
    "max_vx": 0.15,
    "max_vy": 0.15,
    "max_wz": 0.8,
    "kp_u": 0.004,
    "kp_v": 0.003,
    "kp_angle": 1.2,
    "target_lost_timeout": 2.0,
    "search_timeout": 20.0,
}


@dataclass(frozen=True)
class AlignmentOutput:
    """One control step: command + state + what we track."""

    velocity: Velocity2D
    state: AlignmentState
    target: BlockDetection | None
    error_u: float = 0.0
    error_v: float = 0.0
    error_angle: float = 0.0

    @property
    def status(self) -> SkillStatus:
        if self.state is AlignmentState.ALIGNED:
            return SkillStatus.SUCCESS
        if self.state in (AlignmentState.FAILURE, AlignmentState.TARGET_LOST):
            return SkillStatus.FAILURE
        return SkillStatus.RUNNING


class AlignmentController:
    """Visual servoing onto a block of the requested type."""

    def __init__(self, config: dict | None = None, clock: Clock | None = None) -> None:
        self._cfg = {**DEFAULT_CONFIG, **(config or {})}
        self._clock = clock or FakeClock()
        self._state = AlignmentState.SEARCH
        self._target: BlockDetection | None = None
        self._target_type: BlockType | None = None
        self._last_seen_time: float = -1e9
        self._search_start: float = self._clock.now()

    # ------------------------------------------------------------- lifecycle
    def start(self, block_type: BlockType) -> None:
        self._target_type = block_type
        self._target = None
        self._state = AlignmentState.SEARCH
        self._search_start = self._clock.now()
        self._last_seen_time = -1e9

    def stop(self) -> None:
        self._state = AlignmentState.SEARCH
        self._target = None

    @property
    def state(self) -> AlignmentState:
        return self._state

    @property
    def target(self) -> BlockDetection | None:
        return self._target

    # ------------------------------------------------------------- control
    def update(self, detections: list[BlockDetection],
               timestamp: float | None = None) -> AlignmentOutput:
        now = timestamp if timestamp is not None else self._clock.now()
        target = self._select(detections)

        if target is not None:
            self._target = target
            self._last_seen_time = now
            if self._state in (AlignmentState.SEARCH, AlignmentState.TARGET_LOST):
                self._state = AlignmentState.TRACK
        elif self._target is not None and now - self._last_seen_time > \
                self._cfg["target_lost_timeout"]:
            self._state = AlignmentState.TARGET_LOST
            log.warning("alignment: target lost for %.1fs",
                        now - self._last_seen_time)
            return AlignmentOutput(Velocity2D(), self._state, None)

        if self._state is AlignmentState.SEARCH:
            if now - self._search_start > self._cfg["search_timeout"]:
                self._state = AlignmentState.FAILURE
                log.warning("alignment: search timed out after %.1fs",
                            now - self._search_start)
            # gentle rotation widens the search
            return AlignmentOutput(Velocity2D(wz=0.25), self._state, None)

        if self._state is AlignmentState.TARGET_LOST:
            return AlignmentOutput(Velocity2D(), self._state, None)

        target = self._target
        if target is None:
            return AlignmentOutput(Velocity2D(), self._state, None)

        err_u = target.u - self._cfg["target_u"]
        err_v = target.v - self._cfg["target_v"]
        err_a = target.angle - self._cfg["target_angle"]

        within = (abs(err_u) <= self._cfg["u_tolerance"]
                  and abs(err_v) <= self._cfg["v_tolerance"]
                  and abs(err_a) <= self._cfg["angle_tolerance"])

        if within:
            self._state = AlignmentState.ALIGNED
            return AlignmentOutput(Velocity2D(), self._state, target,
                                   err_u, err_v, err_a)

        self._state = AlignmentState.ALIGN
        vy = self._clamp(-self._cfg["kp_u"] * err_u, self._cfg["max_vy"])
        # v too large (block too close) -> back up; v too small -> approach
        vx = self._clamp(-self._cfg["kp_v"] * err_v, self._cfg["max_vx"])
        wz = self._clamp(self._cfg["kp_angle"] * err_a, self._cfg["max_wz"])
        return AlignmentOutput(Velocity2D(vx, vy, wz), self._state, target,
                               err_u, err_v, err_a)

    # ------------------------------------------------------------- internal
    def _select(self, detections: list[BlockDetection]) -> BlockDetection | None:
        """Closest detection of the wanted type to the grab point."""
        wanted = self._target_type
        best = None
        best_d = None
        for det in detections:
            if wanted is not None and det.block_type is not wanted:
                continue
            d = (abs(det.u - self._cfg["target_u"])
                 + abs(det.v - self._cfg["target_v"]))
            if best_d is None or d < best_d:
                best, best_d = det, d
        return best

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))
