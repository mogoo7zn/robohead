"""Hierarchical Finite State Machine engine.

Every State has: entry, update/tick, timeout, success/failure exits and a
recovery path. A Machine runs States; a CompositeState embeds a sub-Machine
(hierarchy). States request transitions by returning the next state's name.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from core.utils.clock import Clock
from core.utils.log import get_logger

log = get_logger("mission.hfsm")

DONE = "__DONE__"


class State(ABC):
    """Base state. Subclasses set `timeout` and implement update()."""

    name: str = "STATE"
    timeout: float | None = None      # seconds; None disables timeout

    def __init__(self, machine: "Machine") -> None:
        self.machine = machine
        self.ctx = machine.ctx
        self._entry_time: float | None = None

    # -------------------------------------------------------------- lifecycle
    def on_entry(self) -> None:
        pass

    def on_exit(self) -> None:
        pass

    @abstractmethod
    def update(self) -> str | None:
        """Return next state name (same machine), DONE, or None to stay."""

    def on_timeout(self) -> str:
        """Where to go when the state timed out. Override for recovery."""
        return "RECOVERY"

    # -------------------------------------------------------------- helpers
    @property
    def elapsed(self) -> float:
        if self._entry_time is None:
            return 0.0
        return self.machine.clock.now() - self._entry_time

    def _enter(self) -> None:
        self._entry_time = self.machine.clock.now()
        log.debug("[%s] enter %s", self.machine.name, self.name)
        self.on_entry()

    def _exit(self) -> None:
        log.debug("[%s] exit %s", self.machine.name, self.name)
        self.on_exit()


class CompositeState(State):
    """A state that runs a whole sub-machine (HFSM hierarchy)."""

    def __init__(self, machine: "Machine", submachine: "Machine") -> None:
        super().__init__(machine)
        self.submachine = submachine

    def on_entry(self) -> None:
        self.submachine.start()

    def update(self) -> str | None:
        result = self.submachine.tick()
        if result == DONE:
            return self.on_submachine_done()
        return None

    def on_submachine_done(self) -> str | None:
        return DONE


class Machine:
    """Runs one level of the hierarchy."""

    def __init__(self, name: str, ctx: Any, clock: Clock,
                 initial: str = "INITIAL") -> None:
        self.name = name
        self.ctx = ctx
        self.clock = clock
        self.initial = initial
        self.states: dict[str, State] = {}
        self.current: State | None = None
        self.history: list[str] = []          # every state ever entered
        self._timeout_count = 0

    # -------------------------------------------------------------- building
    def add_state(self, state: State) -> State:
        if state.name in self.states:
            raise ValueError(f"duplicate state name: {state.name}")
        self.states[state.name] = state
        return state

    # -------------------------------------------------------------- running
    @property
    def state_name(self) -> str:
        return self.current.name if self.current else ""

    @property
    def full_state_name(self) -> str:
        """Path including sub-machines, e.g. MISSION/ACQUIRE/GRAB."""
        if self.current is None:
            return self.name
        if isinstance(self.current, CompositeState):
            return f"{self.current.name}/{self.current.submachine.full_state_name}"
        return f"{self.current.name}"

    def start(self) -> None:
        self.transition(self.initial)

    def transition(self, name: str) -> None:
        if name not in self.states:
            raise KeyError(f"[{self.name}] unknown state: {name}")
        if self.current is not None:
            self.current._exit()
        self.current = self.states[name]
        self.history.append(name)
        self.current._enter()

    def tick(self) -> str | None:
        """Advance the machine. Returns DONE when a state returned DONE."""
        if self.current is None:
            raise RuntimeError(f"[{self.name}] machine not started")

        # timeout check
        if self.current.timeout is not None and self.current.elapsed > self.current.timeout:
            self._timeout_count += 1
            log.warning("[%s] timeout in %s after %.1fs",
                        self.name, self.current.name, self.current.elapsed)
            next_state = self.current.on_timeout()
            if next_state == DONE:
                self.transition(self.current.name)  # re-enter for clean state
                return DONE
            self.transition(next_state)
            return None

        next_name = self.current.update()
        if next_name is None:
            return None
        if next_name == DONE:
            return DONE
        if next_name not in self.states:
            raise KeyError(f"[{self.name}] update() returned unknown state: {next_name}")
        self.transition(next_name)
        return None

    @property
    def timeout_count(self) -> int:
        return self._timeout_count
