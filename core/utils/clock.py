"""Clock abstraction: every timeout / 3 s stability wait / match timer must
run on Clock, so tests never really sleep 3 s or 360 s."""
from __future__ import annotations

import time


class Clock:
    """Base clock: seconds, monotonic semantics."""

    def now(self) -> float:
        raise NotImplementedError

    def sleep(self, seconds: float) -> None:
        """Block (or advance fake time) for the given seconds."""
        raise NotImplementedError

    def advance(self, seconds: float) -> float:
        """Fake clocks only: jump forward. Real clocks raise."""
        raise NotImplementedError("RealClock cannot advance time")


class RealClock(Clock):
    """Wall-clock implementation for the real robot."""

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds > 0:
            time.sleep(seconds)


class FakeClock(Clock):
    """Deterministic clock for tests and mock missions."""

    def __init__(self, start: float = 0.0) -> None:
        self._now = float(start)

    def now(self) -> float:
        return self._now

    def sleep(self, seconds: float) -> None:
        self._now += seconds

    def advance(self, seconds: float) -> float:
        self._now += seconds
        return self._now
