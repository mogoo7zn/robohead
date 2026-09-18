"""ChassisCommander — the Navigator's hardware interface.

Implementations:
  * McuChassis (Pi): wraps McuClient -> binary protocol -> real STM32.
    Line following is a Pi-side closed loop (LINE_SENSOR -> CMD_VEL),
    since protocol v1.1 has no line-follow command on the MCU.
  * MockChassis (Mac/CI): drives SimWorld directly for unit tests

The Navigator only ever sees this interface, so no high-level code
changes when the backend swaps.
"""
from __future__ import annotations

from typing import Protocol

from core.hardware.mcu_client import McuClient
from core.mock.sim_world import SimWorld
from core.navigation.line_follower import LineFollower


class ChassisCommander(Protocol):
    """Low-level chassis orders the Navigator may issue."""

    def line_follow_start(self, segment_id: int, speed: float) -> None:
        """Start following the line at `speed` m/s."""
        ...

    def line_follow_stop(self) -> None:
        ...

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Open-loop body twist (used for turning in place at nodes)."""
        ...

    def stop(self) -> None:
        ...


class McuChassis:
    """Real backend: CMD_VEL over the protocol; Pi-side line following.

    The LINE_SENSOR handler closes the loop at the sensor's own rate
    (50 Hz from the MCU), which is the control rate while FOLLOWing.
    """

    def __init__(self, client: McuClient,
                 line_cfg: dict | None = None) -> None:
        self._client = client
        self._follower = LineFollower(line_cfg)
        client.on_message("LINE_SENSOR", self._on_line_sensor)

    def line_follow_start(self, segment_id: int, speed: float) -> None:
        self._follower.start(speed)

    def line_follow_stop(self) -> None:
        self._follower.stop()
        self._client.set_velocity(0.0, 0.0, 0.0)

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        self._follower.stop()            # manual twist overrides following
        self._client.set_velocity(vx, vy, wz)

    def stop(self) -> None:
        self._follower.stop()
        self._client.set_velocity(0.0, 0.0, 0.0)

    # -------------------------------------------------------------- line loop
    def _on_line_sensor(self, values: dict) -> None:
        if not self._follower.active:
            return
        line = self._client.telemetry.line
        vx, vy, wz = self._follower.update(line)
        self._client.set_velocity(vx, vy, wz)


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
