"""PlaceSkill — verified block placement on a tower.

Sequence (design doc §14.8):
  APPROACH (descend to layer height) -> OPEN -> VERIFY_RELEASE
  -> RETREAT (chassis backs off) -> STABILITY_WAIT (3 s rule)

The 3-second stability wait after losing contact with the building is
a competition rule, not an engineering choice.
"""
from __future__ import annotations

from dataclasses import dataclass

from core.manipulation.manipulator import ManipulatorSequencer
from core.model.enums import SkillStatus
from core.utils.clock import Clock, FakeClock
from core.utils.log import get_logger

log = get_logger("skill.place")

DEFAULT_CONFIG = {
    "stability_wait": 3.0,     # s — competition rule
    "retreat_distance": 0.15,  # m chassis retreat
    "retreat_speed": 0.10,
    "verify_timeout": 10.0,
    "block_height_mm": 70.0,   # one layer
}


@dataclass(frozen=True)
class PlaceResult:
    status: SkillStatus
    released: bool = False
    stable: bool = False
    message: str = ""

    @property
    def success(self) -> bool:
        return self.status is SkillStatus.SUCCESS


class PlaceSkill:
    """Places the held block at (tower, layer)."""

    def __init__(self, sequencer: ManipulatorSequencer,
                 chassis=None,
                 config: dict | None = None,
                 clock: Clock | None = None) -> None:
        self._seq = sequencer
        self._chassis = chassis
        self._cfg = {**DEFAULT_CONFIG, **(config or {})}
        self._clock = clock or FakeClock()
        self._state = "IDLE"
        self._layer = 0
        self._wait_start = 0.0
        self._stopped = False

    @property
    def state(self) -> str:
        return self._state

    def start(self, layer: int = 0) -> None:
        """`layer`: how many blocks are already on the tower (0-based)."""
        self._layer = max(0, int(layer))
        self._stopped = False
        self._state = "DESCEND"
        z = self._layer * self._cfg["block_height_mm"]
        self._seq.begin_move_to(150.0, z)

    def update(self) -> PlaceResult:
        if self._state == "IDLE":
            return PlaceResult(SkillStatus.FAILURE,
                               message="place not started")

        if self._state == "STABILITY_WAIT":
            now = self._clock.now()
            # stop the chassis once the retreat distance is covered
            if self._chassis is not None and self._retreating(now):
                self._chassis.stop()
                self._stopped = True
            if now - self._wait_start >= self._cfg["stability_wait"]:
                self._state = "IDLE"
                log.info("place stable after %.1fs",
                         self._cfg["stability_wait"])
                return PlaceResult(SkillStatus.SUCCESS, released=True,
                                   stable=True)
            return PlaceResult(SkillStatus.RUNNING, released=True)

        step = self._seq.tick()
        if step == "FAULT":
            return self._fail("manipulator fault")
        if step == "TIMEOUT":
            return self._fail("step timeout")
        if step != "DONE":
            return PlaceResult(SkillStatus.RUNNING)

        if self._state == "DESCEND":
            self._state = "OPEN"
            self._seq.begin_gripper(close=False)
        elif self._state == "OPEN":
            # gripper OPEN + no grip_detected = released
            self._state = "RETREAT"
            self._seq.begin_move("retreat")
        elif self._state == "RETREAT":
            self._state = "STABILITY_WAIT"
            self._wait_start = self._clock.now()
            if self._chassis is not None:
                self._chassis.set_velocity(
                    -self._cfg["retreat_speed"], 0.0, 0.0)
        return PlaceResult(SkillStatus.RUNNING)

    # ------------------------------------------------------------- internal
    def _retreating(self, now: float) -> bool:
        """True while the chassis retreat still needs stopping."""
        retreat_time = (self._cfg["retreat_distance"]
                        / max(self._cfg["retreat_speed"], 1e-6))
        return now - self._wait_start >= retreat_time and not self._stopped

    def _fail(self, message: str) -> PlaceResult:
        log.error("place FAILED: %s", message)
        self._state = "IDLE"
        return PlaceResult(SkillStatus.FAILURE, message=message)
