"""GrabSkill — verified block grabbing.

Sequence (design doc §14.7: closing the gripper is NOT success):
  PREGRASP -> DESCEND -> CLOSE&VERIFY -> RETREAT -> HOME

The CLOSE step only completes when the hardware grip sensor confirms
HOLDING + grip_detected. On timeout/fault: retry up to max_retries
(Level-1 recovery), then FAILURE — never a silent success.
"""
from __future__ import annotations

from dataclasses import dataclass

from core.manipulation.manipulator import ManipulatorSequencer
from core.model.enums import SkillStatus
from core.utils.clock import Clock, FakeClock
from core.utils.log import get_logger

log = get_logger("skill.grab")


@dataclass(frozen=True)
class GrabResult:
    status: SkillStatus
    attempts: int = 0
    verified: bool = False
    message: str = ""

    @property
    def success(self) -> bool:
        return self.status is SkillStatus.SUCCESS


class GrabSkill:
    """One attempt = full pregrasp -> verify -> retreat cycle."""

    def __init__(self, sequencer: ManipulatorSequencer,
                 config: dict | None = None,
                 clock: Clock | None = None) -> None:
        self._seq = sequencer
        self._cfg = {**(config or {})}
        self._clock = clock or FakeClock()
        self._state = "IDLE"
        self._attempts = 0

    @property
    def state(self) -> str:
        return self._state

    @property
    def attempts(self) -> int:
        return self._attempts

    def start(self) -> None:
        self._attempts = 0
        self._begin_attempt()

    def update(self) -> GrabResult:
        if self._state == "IDLE":
            return GrabResult(SkillStatus.FAILURE, self._attempts,
                              message="grab not started")

        step = self._seq.tick()
        if step == "FAULT":
            return self._retry_or_fail("manipulator fault")
        if step == "TIMEOUT":
            return self._retry_or_fail("step timeout")
        if step != "DONE":
            return GrabResult(SkillStatus.RUNNING, self._attempts)

        if self._state == "PREGRASP":
            self._state = "DESCEND"
            self._seq.begin_move("grasp")
        elif self._state == "DESCEND":
            self._state = "CLOSE"
            self._seq.begin_gripper(close=True)
        elif self._state == "CLOSE":
            # grip sensor verified HOLDING -> carry it away
            log.info("grab verified on attempt %d", self._attempts)
            self._state = "RETREAT"
            self._seq.begin_move("retreat")
        elif self._state == "RETREAT":
            self._state = "HOME"
            self._seq.begin_move("home")
        elif self._state == "HOME":
            self._state = "IDLE"
            return GrabResult(SkillStatus.SUCCESS, self._attempts,
                              verified=True)

        return GrabResult(SkillStatus.RUNNING, self._attempts)

    # ------------------------------------------------------------- internal
    def _begin_attempt(self) -> None:
        self._attempts += 1
        self._state = "PREGRASP"
        self._seq.begin_move("pregrasp")

    def _retry_or_fail(self, reason: str) -> GrabResult:
        max_retries = int(self._cfg.get("max_retries", 2))
        if self._attempts > max_retries:
            log.error("grab FAILED: %s after %d attempts",
                      reason, self._attempts)
            self._state = "IDLE"
            return GrabResult(SkillStatus.FAILURE, self._attempts,
                              message=f"{reason} after {self._attempts} attempts")
        log.warning("grab retry (%s): attempt %d", reason, self._attempts)
        self._seq.begin_gripper(close=False)
        self._begin_attempt()
        return GrabResult(SkillStatus.RUNNING, self._attempts)
