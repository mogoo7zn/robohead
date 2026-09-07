"""Manipulator subsystem: cross-slide X/Z axes + gripper.

ManipulatorCommander is the hardware interface. Implementations:
  * McuManipulator (Pi): wraps McuClient (binary protocol -> STM32)
  * SimManipulator (Mac/CI): reads/writes SimWorld directly

Skills (grab/place) only ever see the interface, so swapping the backend
changes nothing above it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from core.model.enums import GripperState
from core.mock.sim_world import SimWorld
from core.utils.clock import Clock, FakeClock
from core.utils.log import get_logger

log = get_logger("manipulation.manipulator")

AXIS_X = "X"
AXIS_Z = "Z"


@dataclass(frozen=True)
class ManipulatorState:
    x_mm: float = 0.0
    z_mm: float = 0.0
    homed: bool = False
    gripper: GripperState = GripperState.UNKNOWN
    grip_detected: bool = False
    moving: bool = False
    fault: bool = False

    def at(self, x: float, z: float, tol: float = 1.0) -> bool:
        return (abs(self.x_mm - x) <= tol and abs(self.z_mm - z) <= tol)


class ManipulatorCommander(Protocol):
    """Axis + gripper orders and state feedback."""

    def home(self) -> None: ...
    def move_axis(self, axis: str, position_mm: float,
                  speed: float = 0.0) -> None: ...
    def gripper_close(self) -> None: ...
    def gripper_open(self) -> None: ...
    def state(self) -> ManipulatorState: ...


class McuManipulator:
    """Real backend: forwards commands through McuClient and reads back
    MANIPULATOR_STATE telemetry."""

    def __init__(self, client) -> None:
        self._client = client

    def home(self) -> None:
        self._client.send_home()

    def move_axis(self, axis: str, position_mm: float,
                  speed: float = 0.0) -> None:
        code = 0 if axis == AXIS_X else 1
        self._client.send_linear_axis(code, position_mm, speed)

    def gripper_close(self) -> None:
        self._client.send_gripper(close=True)

    def gripper_open(self) -> None:
        self._client.send_gripper(close=False)

    def state(self) -> ManipulatorState:
        t = self._client.telemetry
        return ManipulatorState(
            x_mm=t.manipulator_x_mm, z_mm=t.manipulator_z_mm,
            homed=t.manipulator_homed, gripper=t.manipulator_gripper,
            grip_detected=t.manipulator_grip_detected,
            moving=t.manipulator_moving, fault=t.manipulator_fault,
        )


class SimManipulator:
    """Mock backend: drives SimWorld directly (unit tests, no protocol)."""

    def __init__(self, world: SimWorld, clock: Clock | None = None) -> None:
        self.world = world
        self._clock = clock or FakeClock()

    def home(self) -> None:
        self.world.manip_home()

    def move_axis(self, axis: str, position_mm: float,
                  speed: float = 0.0) -> None:
        self.world.manip_move_axis(axis, position_mm, speed)

    def gripper_close(self) -> None:
        self.world.manip_set_gripper(True, 1.0, self._clock)

    def gripper_open(self) -> None:
        self.world.manip_set_gripper(False, 1.0, self._clock)

    def state(self) -> ManipulatorState:
        w = self.world
        return ManipulatorState(
            x_mm=w.manip_x_mm, z_mm=w.manip_z_mm,
            homed=w.manip_homed, gripper=w.manip_gripper,
            grip_detected=w.manip_grip_detected,
            moving=w.manip_moving, fault=False,
        )


class ManipulatorSequencer:
    """Async axis/gripper primitives with timeouts, backend-agnostic.

    A skill calls `tick()` every control cycle; each primitive moves to
    the next step once the hardware state confirms completion. This is
    what makes grab/place the same code on mock and real hardware.
    """

    def __init__(self, commander: ManipulatorCommander,
                 config: dict | None = None,
                 clock: Clock | None = None) -> None:
        self._cmd = commander
        self._cfg = {
            "home": 15.0, "move": 20.0, "gripper": 5.0,
            **(config or {}).get("timeouts", {}),
        }
        self._positions = {
            "pregrasp": {"x": 150, "z": 80},
            "grasp": {"x": 150, "z": 0},
            "place": {"x": 150, "z": 60},
            "retreat": {"x": 150, "z": 120},
            "home": {"x": 0, "z": 0},
            **(config or {}).get("positions", {}),
        }
        self._clock = clock or FakeClock()
        self._step_start = 0.0
        self._timeout = 0.0
        self._await_grip: bool | None = None   # None=axis move, True/False=grip

    # ------------------------------------------------------------- positions
    def position(self, name: str) -> tuple[float, float]:
        p = self._positions[name]
        return float(p["x"]), float(p["z"])

    # ------------------------------------------------------------- primitives
    def begin_home(self) -> None:
        self._await_grip = None
        self._cmd.home()
        self._arm(self._cfg["home"])

    def begin_move(self, position_name: str, speed: float = 0.0) -> None:
        x, z = self.position(position_name)
        self.begin_move_to(x, z, speed)

    def begin_move_to(self, x_mm: float, z_mm: float,
                      speed: float = 0.0) -> None:
        self._await_grip = None
        self._cmd.move_axis(AXIS_X, x_mm, speed)
        self._cmd.move_axis(AXIS_Z, z_mm, speed)
        self._arm(self._cfg["move"])

    def begin_gripper(self, close: bool) -> None:
        self._await_grip = bool(close)
        if close:
            self._cmd.gripper_close()
        else:
            self._cmd.gripper_open()
        self._arm(self._cfg["gripper"])

    def tick(self) -> str:
        """Returns 'RUNNING' | 'DONE' | 'TIMEOUT' | 'FAULT'.

        Done condition: axis steps finish when the axes stop moving;
        gripper steps finish when the grip sensor confirms the expected
        state (HOLDING+detected, or OPEN+empty).
        """
        state = self._cmd.state()
        if state.fault:
            return "FAULT"
        if self._clock.now() - self._step_start > self._timeout:
            return "TIMEOUT"

        if self._await_grip is None:
            return "DONE" if not state.moving else "RUNNING"

        if self._await_grip:
            done = (state.gripper is GripperState.HOLDING
                    and state.grip_detected)
        else:
            done = (state.gripper is GripperState.OPEN
                    and not state.grip_detected)
        return "DONE" if done else "RUNNING"

    # ------------------------------------------------------------- internal
    def _arm(self, timeout: float) -> None:
        self._step_start = self._clock.now()
        self._timeout = timeout
