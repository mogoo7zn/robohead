"""FakeSTM32 — a software stand-in for the real MCU.

Speaks the v1.1 binary protocol over a Transport, drives a SimWorld
(mecanum chassis plant + lift/gripper), and emits telemetry
(ODOM / LINE_SENSOR at 50 Hz, ROBOT_STATE at 10 Hz). Used on Mac / in CI
so the whole mission runs without any hardware.

Faithful to the real firmware's safety behavior (protocol doc §7):
  * no CMD_VEL for 200 ms -> target velocity zeroed, motor_state=TIMEOUT,
    error_code bit3 set; cleared when CMD_VEL resumes
  * CRC-invalid / wrong-version frames are dropped by the StreamParser

The real STM32 firmware (firmware/stm32) implements the same protocol on
the other side of the wire — swapping FakeSTM32 for the real MCU changes
nothing above the Transport.
"""
from __future__ import annotations

from core.hardware.transport import Transport
from core.mock.sim_world import SimWorld
from core.model.enums import GripperState
from core.protocol import (
    ACTION_GRIPPER_CLOSE,
    ACTION_GRIPPER_OPEN,
    ACTION_LIFT_DOWN,
    ACTION_LIFT_UP,
    ACTION_STOP_ALL,
    Frame,
    GRIPPER_CLOSED,
    GRIPPER_FAULT,
    GRIPPER_HOLDING,
    GRIPPER_OPENED,
    GRIPPER_UNKNOWN,
    LIFT_IDLE,
    LIFT_MOVING_DOWN,
    LIFT_MOVING_UP,
    MOTOR_IDLE,
    MOTOR_RUNNING,
    MOTOR_TIMEOUT,
    ERR_COMM_TIMEOUT,
    StreamParser,
    decode_payload,
    encode_frame,
    encode_payload,
    spec_by_id,
    spec_by_name,
)
from core.utils.clock import Clock
from core.utils.log import get_logger

log = get_logger("mock.fake_stm32")

_LIFT_TOP_MM = 200.0   # matches SimWorld axis_limits["Z"]

_GRIPPER_CODES = {
    GripperState.UNKNOWN: GRIPPER_UNKNOWN,
    GripperState.OPEN: GRIPPER_OPENED,
    GripperState.CLOSED: GRIPPER_CLOSED,
    GripperState.HOLDING: GRIPPER_HOLDING,
    GripperState.FAULT: GRIPPER_FAULT,
}


def _to_int16(value: float) -> int:
    return max(-32768, min(32767, round(value)))


def _to_int32(value: float) -> int:
    return max(-2147483648, min(2147483647, round(value)))


class FakeSTM32:
    """Protocol-level emulation of the STM32 firmware (protocol v1.1)."""

    def __init__(self, transport: Transport, world: SimWorld, clock: Clock,
                 telemetry_period: float = 0.02,
                 state_period: float = 0.10,
                 gripper_duration: float = 1.0,
                 watchdog_timeout: float = 0.2) -> None:
        self._transport = transport
        self._world = world
        self._clock = clock
        self._parser = StreamParser()
        self._seq = 0
        self._telemetry_period = telemetry_period   # ODOM / LINE_SENSOR 50 Hz
        self._state_period = state_period           # ROBOT_STATE 10 Hz
        self._last_telemetry = float("-inf")
        self._last_state = float("-inf")
        self._last_cmd_vel_rx = float("-inf")
        self._watchdog_timeout = watchdog_timeout
        self._gripper_duration = gripper_duration
        self.watchdog_tripped = False
        self.start_time = clock.now()

    # -------------------------------------------------------------- main loop
    def tick(self, dt: float) -> None:
        """Process inbound commands, advance simulation, emit telemetry."""
        now = self._clock.now()

        # 1. inbound frames
        data = self._transport.read(timeout=0.0)
        if data:
            for frame in self._parser.feed(data):
                self._handle_frame(frame, now)

        # 2. CMD_VEL watchdog (protocol doc §7)
        if self._last_cmd_vel_rx > float("-inf"):
            if now - self._last_cmd_vel_rx > self._watchdog_timeout:
                if not self.watchdog_tripped:
                    log.warning("watchdog trip: no CMD_VEL for %.0f ms",
                                (now - self._last_cmd_vel_rx) * 1000)
                self.watchdog_tripped = True
                self._world.velocity_cmd = (0.0, 0.0, 0.0)
                self._world.stopped = True
            else:
                self.watchdog_tripped = False

        # 3. simulate
        self._world.step(dt, self._clock)

        # 4. telemetry
        if now - self._last_telemetry >= self._telemetry_period:
            self._last_telemetry = now
            self._emit_odom()
            self._emit_line_sensor()
        if now - self._last_state >= self._state_period:
            self._last_state = now
            self._emit_robot_state(now)

    # -------------------------------------------------------------- commands
    def _send(self, msg_name: str, **values) -> None:
        payload = encode_payload(msg_name, **values)
        frame = Frame(seq=self._seq, msg_id=spec_by_name(msg_name).msg_id,
                      payload=payload)
        self._seq = (self._seq + 1) & 0xFF
        self._transport.send(encode_frame(frame))

    def _handle_frame(self, frame: Frame, now: float) -> None:
        spec = spec_by_id(frame.msg_id)
        if spec is None or spec.direction != "pi_to_mcu":
            return
        values = decode_payload(spec.name, frame.payload)

        if spec.name == "CMD_VEL":
            self._last_cmd_vel_rx = now
            # a valid CMD_VEL proves the link is alive: apply it even if
            # the watchdog had tripped (doc §7: 恢复收到 CMD_VEL 后清除)
            self.watchdog_tripped = False
            self._world.velocity_cmd = (
                values["vx_mm_s"] / 1000.0,
                values["vy_mm_s"] / 1000.0,
                values["wz_mrad_s"] / 1000.0)
            self._world.stopped = not any(
                abs(v) > 1e-9 for v in self._world.velocity_cmd)
        elif spec.name == "CMD_ACTION":
            self._handle_action(values, now)

    def _handle_action(self, values: dict, now: float) -> None:
        action, param = values["action_id"], values["param"]
        w = self._world
        if action == ACTION_GRIPPER_OPEN:
            w.manip_set_gripper(False, self._gripper_duration, self._clock)
        elif action == ACTION_GRIPPER_CLOSE:
            w.manip_set_gripper(True, self._gripper_duration, self._clock)
        elif action == ACTION_LIFT_UP:
            target = param if param > 0 else _LIFT_TOP_MM
            w.manip_move_axis("Z", target, 0.0)
        elif action == ACTION_LIFT_DOWN:
            target = param if param > 0 else 0.0
            w.manip_move_axis("Z", target, 0.0)
        elif action == ACTION_STOP_ALL:
            w.velocity_cmd = (0.0, 0.0, 0.0)
            w.stopped = True
            # freeze the lift at its current position
            w.manip_move_axis("Z", w.manip_z_mm, 0.0)
        else:
            log.debug("fake mcu ignores action %d", action)

    # -------------------------------------------------------------- telemetry
    def _emit_odom(self) -> None:
        w = self._world
        vx, vy, wz = w.velocity_cmd
        self._send("ODOM",
                   x_mm=_to_int32(w.odom_x * 1000.0),
                   y_mm=_to_int32(w.odom_y * 1000.0),
                   theta_mrad=_to_int32(w.odom_yaw * 1000.0),
                   vx_mm_s=_to_int16(vx * 1000.0),
                   vy_mm_s=_to_int16(vy * 1000.0),
                   wz_mrad_s=_to_int16(wz * 1000.0))

    def _emit_line_sensor(self) -> None:
        w = self._world
        self._send("LINE_SENSOR",
                   line_detected=1 if w.line_detected else 0,
                   offset_mm=_to_int16(w.line_error * 1000.0),
                   confidence=_to_int16(w.line_confidence * 100.0))

    def _emit_robot_state(self, now: float) -> None:
        w = self._world
        if self.watchdog_tripped:
            motor = MOTOR_TIMEOUT
        elif any(abs(v) > 1e-9 for v in w.velocity_cmd):
            motor = MOTOR_RUNNING
        else:
            motor = MOTOR_IDLE

        if w.manip_moving:
            lift = LIFT_MOVING_UP if w._manip_target_z > w.manip_z_mm \
                else LIFT_MOVING_DOWN
        else:
            lift = LIFT_IDLE

        error = ERR_COMM_TIMEOUT if self.watchdog_tripped else 0
        self._send("ROBOT_STATE",
                   motor_state=motor,
                   gripper_state=_GRIPPER_CODES[w.manip_gripper],
                   lift_state=lift,
                   error_code=error)
