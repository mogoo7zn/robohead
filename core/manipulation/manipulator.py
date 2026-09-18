"""Manipulator subsystem: lift axis + gripper (protocol v1.1 robot).

ManipulatorCommander is the hardware interface. Implementations:
  * McuManipulator (Pi): wraps McuClient (CMD_ACTION -> STM32)
  * SimManipulator (Mac/CI): reads/writes SimWorld directly

Skills (grab/place) only ever see the interface, so swapping the backend
changes nothing above it.

Note: protocol v1.1 has no X slide — the robot is chassis + lift + gripper.
move_axis("X", ...) is accepted (skills still specify x/z positions) but
forwarded as a no-op; z maps onto the lift (丝杆).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from core.model.enums import GripperState
from core.mock.sim_world import SimWorld
from core.protocol import (
    ACTION_LIFT_DOWN,
    ACTION_LIFT_UP,
    ERR_GRIPPER,
    ERR_LIFT,
)
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
    """Real backend: commands via CMD_ACTION, state via ROBOT_STATE.

    ROBOT_STATE carries no axis position, so commanded positions are
    mirrored locally; completion is detected from lift_state transitions
    (MOVING_* -> IDLE) reported at 10 Hz. Because the state report lags
    the command by up to one 10 Hz period, a short grace window after
    each lift command keeps `moving` true until the report catches up.
    """

    GRACE_S = 0.25   # > one 10 Hz ROBOT_STATE period

    def __init__(self, client, clock: Clock) -> None:
        self._client = client
        self._clock = clock
        self._cmd_x = 0.0
        self._cmd_z = 0.0
        self._homing = False
        self._homed = False
        self._last_lift_cmd = float("-inf")
        self._warned_no_x = False

    def home(self) -> None:
        self._homing = True
        self._homed = False
        self._cmd_z = 0.0
        self._last_lift_cmd = self._clock.now()
        self._client.send_action(ACTION_LIFT_DOWN, 0)   # bottom limit = home

    def move_axis(self, axis: str, position_mm: float,
                  speed: float = 0.0) -> None:
        if axis == AXIS_X:
            if not self._warned_no_x:
                log.warning("move_axis(X): robot has no X slide — ignored")
                self._warned_no_x = True
            self._cmd_x = position_mm
            return
        action = ACTION_LIFT_UP if position_mm > self._cmd_z else ACTION_LIFT_DOWN
        self._cmd_z = position_mm
        self._last_lift_cmd = self._clock.now()
        self._client.send_action(action, int(position_mm))

    def gripper_close(self) -> None:
        self._client.send_gripper(close=True)

    def gripper_open(self) -> None:
        self._client.send_gripper(close=False)

    def state(self) -> ManipulatorState:
        t = self._client.telemetry.state
        in_grace = (self._clock.now() - self._last_lift_cmd) < self.GRACE_S
        moving = t.lift_moving or in_grace
        if self._homing and not moving:
            self._homing = False
            self._homed = True
        return ManipulatorState(
            x_mm=self._cmd_x, z_mm=self._cmd_z,
            homed=self._homed,
            gripper=t.gripper,
            grip_detected=t.gripper is GripperState.HOLDING,
            moving=moving,
            fault=bool(t.error_code & (ERR_GRIPPER | ERR_LIFT)))


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
