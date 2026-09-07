"""Skill base class: long-running actions ticked by HFSM states.

All skills return SkillResult with the unified status set
(RUNNING / SUCCESS / FAILURE / TIMEOUT / CANCELLED).
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from core.model.result import SkillResult
from core.utils.clock import Clock


class Skill(ABC):
    """A skill owns no global state; everything it needs is injected."""

    name: str = "SKILL"

    def __init__(self, clock: Clock, timeout: float = 60.0) -> None:
        self._clock = clock
        self.timeout = timeout
        self._start_time: float | None = None
        self._last_result: SkillResult | None = None

    # -------------------------------------------------------------- control
    def start(self, **params: Any) -> None:
        self._start_time = self._clock.now()
        self._last_result = None
        self.on_start(**params)

    def tick(self) -> SkillResult:
        if self._start_time is None:
            return SkillResult.failure("skill not started")
        if self.timeout is not None and self.elapsed > self.timeout:
            self._last_result = SkillResult.timeout(
                f"{self.name} exceeded {self.timeout:.1f}s")
            return self._last_result
        self._last_result = self.on_tick()
        return self._last_result

    def cancel(self) -> None:
        self.on_cancel()

    # -------------------------------------------------------------- helpers
    @property
    def elapsed(self) -> float:
        if self._start_time is None:
            return 0.0
        return self._clock.now() - self._start_time

    @property
    def last_result(self) -> SkillResult | None:
        return self._last_result

    # -------------------------------------------------------------- hooks
    @abstractmethod
    def on_start(self, **params: Any) -> None:
        ...

    @abstractmethod
    def on_tick(self) -> SkillResult:
        ...

    def on_cancel(self) -> None:
        pass
