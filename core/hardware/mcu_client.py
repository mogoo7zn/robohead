"""McuClient — the Raspberry Pi side of the STM32 link.

Responsibilities:
  * send commands (CMD_VEL, CMD_ACTION)
  * receive telemetry (ODOM, LINE_SENSOR, ROBOT_STATE)
  * SI <-> integer unit conversion (protocol doc v1.1 §8)
  * sequence numbers, link health for the high-level watchdog

It never runs motors itself: it only speaks the binary protocol.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

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
    StreamParser,
    decode_payload,
    encode_frame,
    encode_payload,
    spec_by_id,
)
from core.hardware.transport import Transport
from core.utils.clock import Clock
from core.utils.log import get_logger

log = get_logger("hardware.mcu_client")

INT16_MIN, INT16_MAX = -32768, 32767

_GRIPPER_STATES = {
    GRIPPER_UNKNOWN: GripperState.UNKNOWN,
    GRIPPER_OPENED: GripperState.OPEN,
    GRIPPER_CLOSED: GripperState.CLOSED,
    GRIPPER_HOLDING: GripperState.HOLDING,
    GRIPPER_FAULT: GripperState.FAULT,
}


@dataclass
class OdometrySample:
    """SI units: m, rad, m/s, rad/s (converted from mm/mrad integers)."""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    timestamp: float = 0.0


@dataclass
class LineSensorSample:
    line_detected: bool = False
    offset_m: float = 0.0          # lateral offset, right of travel dir +
    confidence: float = 0.0        # 0..1
    timestamp: float = 0.0


@dataclass
class RobotState:
    motor_state: int = 0
    gripper_state: int = 0
    lift_state: int = 0
    error_code: int = 0
    timestamp: float = 0.0

    @property
    def gripper(self) -> GripperState:
        return _GRIPPER_STATES.get(self.gripper_state, GripperState.UNKNOWN)

    @property
    def lift_moving(self) -> bool:
        return self.lift_state in (2, 3)   # MOVING_UP / MOVING_DOWN

    @property
    def comm_timeout(self) -> bool:
        return bool(self.error_code & 0x08)  # ERR_COMM_TIMEOUT


@dataclass
class McuTelemetry:
    odom: OdometrySample = field(default_factory=OdometrySample)
    line: LineSensorSample = field(default_factory=LineSensorSample)
    state: RobotState = field(default_factory=RobotState)


class McuClient:
    """Sends commands and parses telemetry over a Transport.

    CMD_VEL is periodic (50 Hz) per the protocol: it doubles as the
    heartbeat for the STM32's 200 ms watchdog, so the current target is
    re-sent even when idle (zeros). set_velocity() sends immediately and
    tick() fills the gaps whenever the caller's own rate is lower.
    """

    LINK_TIMEOUT = 0.5     # s without any MCU frame -> link dead (doc §7)
    CMD_VEL_PERIOD = 0.02  # s — 50 Hz (doc §5.1)

    def __init__(self, transport: Transport, clock: Clock) -> None:
        self._transport = transport
        self._clock = clock
        self._parser = StreamParser()
        self._seq = 0
        self.telemetry = McuTelemetry()
        self._handlers: dict[str, list[Callable[[dict], None]]] = {}
        self.last_rx_time = float("-inf")
        self._vel_target = (0.0, 0.0, 0.0)
        self._last_cmd_vel_sent = float("-inf")

    # -------------------------------------------------------------- commands
    def _send(self, msg_name: str, **values) -> None:
        payload = encode_payload(msg_name, **values)
        frame = Frame(seq=self._seq, msg_id=_id_of(msg_name), payload=payload)
        self._seq = (self._seq + 1) & 0xFF
        self._transport.send(encode_frame(frame))

    @staticmethod
    def _to_int16(value: float) -> int:
        return max(INT16_MIN, min(INT16_MAX, round(value * 1000.0)))

    def set_velocity(self, vx: float, vy: float, wz: float) -> None:
        """Set the body-twist target (SI units) and send it now."""
        self._vel_target = (vx, vy, wz)
        self._send_cmd_vel()

    def _send_cmd_vel(self) -> None:
        vx, vy, wz = self._vel_target
        self._send("CMD_VEL",
                   vx_mm_s=self._to_int16(vx),
                   vy_mm_s=self._to_int16(vy),
                   wz_mrad_s=self._to_int16(wz))
        self._last_cmd_vel_sent = self._clock.now()

    def send_action(self, action_id: int, param: int = 0) -> None:
        self._send("CMD_ACTION", action_id=action_id, param=int(param))

    def send_gripper(self, close: bool) -> None:
        self.send_action(ACTION_GRIPPER_CLOSE if close else ACTION_GRIPPER_OPEN)

    def send_stop_all(self) -> None:
        """Stop every mechanism and zero the chassis (CMD_ACTION 0x05)."""
        self._vel_target = (0.0, 0.0, 0.0)
        self.send_action(ACTION_STOP_ALL)

    # -------------------------------------------------------------- receive
    def on_message(self, msg_name: str, handler: Callable[[dict], None]) -> None:
        self._handlers.setdefault(msg_name, []).append(handler)

    def poll(self) -> None:
        """Read all pending bytes and dispatch complete frames."""
        data = self._transport.read(timeout=0.0)
        if not data:
            return
        now = self._clock.now()
        self.last_rx_time = now
        for frame in self._parser.feed(data):
            self._dispatch(frame, now)

    def _dispatch(self, frame: Frame, now: float) -> None:
        spec = spec_by_id(frame.msg_id)
        if spec is None:
            return
        try:
            values = decode_payload(spec.name, frame.payload)
        except ValueError as e:
            log.warning("bad payload for %s: %s", spec.name, e)
            return

        t = self.telemetry
        if spec.name == "ODOM":
            t.odom = OdometrySample(
                x=values["x_mm"] / 1000.0,
                y=values["y_mm"] / 1000.0,
                yaw=values["theta_mrad"] / 1000.0,
                vx=values["vx_mm_s"] / 1000.0,
                vy=values["vy_mm_s"] / 1000.0,
                wz=values["wz_mrad_s"] / 1000.0,
                timestamp=now)
        elif spec.name == "LINE_SENSOR":
            t.line = LineSensorSample(
                line_detected=bool(values["line_detected"]),
                offset_m=values["offset_mm"] / 1000.0,
                confidence=values["confidence"] / 100.0,
                timestamp=now)
        elif spec.name == "ROBOT_STATE":
            t.state = RobotState(
                motor_state=values["motor_state"],
                gripper_state=values["gripper_state"],
                lift_state=values["lift_state"],
                error_code=values["error_code"],
                timestamp=now)
            if values["error_code"]:
                log.warning("ROBOT_STATE error_code=%#04x "
                            "(motor=%d gripper=%d lift=%d)",
                            values["error_code"], values["motor_state"],
                            values["gripper_state"], values["lift_state"])
        else:
            log.debug("unhandled telemetry %s", spec.name)

        for handler in self._handlers.get(spec.name, []):
            handler(values)

    # -------------------------------------------------------------- health
    def tick(self) -> None:
        """Periodic maintenance: keep the CMD_VEL stream alive, then poll.

        There is no heartbeat message: CMD_VEL at 50 Hz *is* the heartbeat
        for the STM32, and ODOM at 50 Hz is the heartbeat for us (doc §7).
        """
        now = self._clock.now()
        if now - self._last_cmd_vel_sent >= self.CMD_VEL_PERIOD:
            self._send_cmd_vel()
        self.poll()

    @property
    def link_alive(self) -> bool:
        return self._clock.now() - self.last_rx_time < self.LINK_TIMEOUT

    @property
    def stats(self):
        return self._parser.stats


def _id_of(msg_name: str) -> int:
    from core.protocol.messages import MESSAGES
    return MESSAGES[msg_name].msg_id
