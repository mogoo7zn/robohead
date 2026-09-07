"""MatchManager — 360 s match clock and strategy phases.

Runs entirely on the Clock abstraction so tests use FakeClock and never
really wait. Emits MatchTickEvent / MatchStartedEvent / MatchFinishedEvent
into the WorldStateStore.
"""
from __future__ import annotations

from core.model.enums import MatchPhase
from core.state.events import (
    MatchFinishedEvent,
    MatchStartedEvent,
    MatchTickEvent,
)
from core.state.store import WorldStateStore
from core.utils.clock import Clock


class MatchManager:
    def __init__(self, store: WorldStateStore, clock: Clock,
                 total_time: float = 360.0,
                 normal_end_time: float = 240.0,
                 safe_end_time: float = 320.0) -> None:
        self._store = store
        self._clock = clock
        self.total_time = total_time
        self.normal_end_time = normal_end_time
        self.safe_end_time = safe_end_time
        self._start_time: float | None = None
        self._finished = False

    # -------------------------------------------------------------- control
    def start(self) -> None:
        self._start_time = self._clock.now()
        self._finished = False
        self._store.apply_event(MatchStartedEvent(start_time=self._start_time))

    @property
    def started(self) -> bool:
        return self._start_time is not None

    @property
    def finished(self) -> bool:
        return self._finished

    # -------------------------------------------------------------- time
    @property
    def elapsed(self) -> float:
        if self._start_time is None:
            return 0.0
        return max(0.0, self._clock.now() - self._start_time)

    @property
    def remaining(self) -> float:
        return max(0.0, self.total_time - self.elapsed)

    @property
    def phase(self) -> MatchPhase:
        if self.elapsed >= self.safe_end_time:
            return MatchPhase.ENDGAME
        if self.elapsed >= self.normal_end_time:
            return MatchPhase.SAFE
        return MatchPhase.NORMAL

    # -------------------------------------------------------------- ticking
    def tick(self) -> None:
        if self._start_time is None or self._finished:
            return
        if self.remaining <= 0.0:
            self._finished = True
            self._store.apply_event(MatchFinishedEvent(reason="time up"))
        self._store.apply_event(MatchTickEvent(
            elapsed_time=self.elapsed,
            remaining_time=self.remaining,
            phase=self.phase,
        ))

    def finish(self, reason: str = "") -> None:
        if not self._finished:
            self._finished = True
            self._store.apply_event(MatchFinishedEvent(reason=reason))
