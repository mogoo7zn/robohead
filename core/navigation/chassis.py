"""ChassisCommander — the Navigator's hardware interface.

Implementations:
  * McuChassis (Pi): wraps McuClient -> binary protocol -> real STM32
  * MockChassis (Mac/CI): drives SimWorld directly for unit tests

The Navigator only ever sees this interface, so no high-level code
changes when the backend swaps.
"""
from __future__ import annotations

from typing import Protocol

from core.mock.sim_world import SimWorld


class ChassisCommander(Protocol):
    """Low-level chassis orders the Navigator may issue."""

    def line_follow_start(self, segment_id: int, speed: float) -> None:
        """Start following line segment `segment_id` at `speed` m/s."""
        ...

    def line_follow_stop(self) -> None:
        ...

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Open-loop body twist (used for turning in place at nodes)."""
        ...

    def stop(self) -> None:
        ...


class McuChassis:
    """Real backend: forwards to McuClient (binary protocol)."""

    def __init__(self, client) -> None:
        self._client = client

    def line_follow_start(self, segment_id: int, speed: float) -> None:
        self._client.send_line_follow_start(segment_id, speed)

    def line_follow_stop(self) -> None:
        self._client.send_line_follow_stop()

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        self._client.send_velocity(vx, vy, wz)

    def stop(self) -> None:
        self._client.send_stop()


class MockChassis:
    """Mock backend: drives a SimWorld (used in unit tests without
    the full protocol stack)."""

    def __init__(self, world: SimWorld) -> None:
        self.world = world
        self.commands: list[tuple] = []   # audit trail for assertions

    def line_follow_start(self, segment_id: int, speed: float) -> None:
        self.world.line_follow_active = True
        self.world.line_follow_speed = speed
        self.world.stopped = False
        self.commands.append(("line_follow_start", segment_id, speed))

    def line_follow_stop(self) -> None:
        self.world.line_follow_active = False
        self.world.velocity_cmd = (0.0, 0.0, 0.0)
        self.world.stopped = True
        self.commands.append(("line_follow_stop",))

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        self.world.velocity_cmd = (vx, vy, wz)
        self.world.stopped = False
        self.world.line_follow_active = False
        self.commands.append(("velocity", vx, vy, wz))

    def stop(self) -> None:
        self.world.velocity_cmd = (0.0, 0.0, 0.0)
        self.world.stopped = True
        self.world.line_follow_active = False
        self.commands.append(("stop",))
