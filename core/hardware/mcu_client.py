"""McuClient — the Raspberry Pi side of the STM32 link.

Responsibilities:
  * send commands (velocity, line-follow, manipulator, heartbeat)
  * receive telemetry (odometry, IMU, line state, manipulator state, ...)
  * sequence numbers, link health for the high-level watchdog

It never runs motors itself: it only speaks the binary protocol.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from core.model.enums import GripperState
from core.protocol import (
    AXIS_ALL,
    Frame,
    GRIPPER_CLOSE,
    GRIPPER_OPEN,
    StreamParser,
    decode_payload,
    encode_frame,
    encode_payload,
    spec_by_id,
)
from core.protocol.messages import (
    LINE_CTRL_IDLE,
    LINE_CTRL_RUNNING,
)
from core.hardware.transport import Transport
from core.utils.clock import Clock
from core.utils.log import get_logger

log = get_logger("hardware.mcu_client")

_GRIPPER_STATES = {
    0: GripperState.UNKNOWN,
    1: GripperState.OPEN,
    2: GripperState.CLOSED,
    3: GripperState.HOLDING,
    4: GripperState.FAULT,
}


@dataclass
class OdometrySample:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    timestamp: float = 0.0


@dataclass
class ImuSample:
    yaw: float = 0.0
    gyro_z: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    timestamp: float = 0.0


@dataclass
class LineFollowStatus:
    line_detected: bool = False
    line_error: float = 0.0
    confidence: float = 0.0
    controller_state: int = LINE_CTRL_IDLE   # LINE_CTRL_*
    intersection_detected: bool = False
    fault: bool = False
    timestamp: float = 0.0

    @property
    def is_running(self) -> bool:
        return self.controller_state == LINE_CTRL_RUNNING


@dataclass
class McuTelemetry:
    heartbeat_age: float = float("inf")
    odom: OdometrySample = field(default_factory=OdometrySample)
    imu: ImuSample = field(default_factory=ImuSample)
    line: LineFollowStatus = field(default_factory=LineFollowStatus)
    manipulator_x_mm: float = 0.0
    manipulator_z_mm: float = 0.0
    manipulator_homed: bool = False
    manipulator_gripper: GripperState = GripperState.UNKNOWN
    manipulator_grip_detected: bool = False
    manipulator_moving: bool = False
    manipulator_fault: bool = False
    manipulator_fault_code: int = 0
    start_button: bool = False
    fault_code: int = 0
    fault_detail: int = 0
    fault_flags: int = 0


class McuClient:
    """Sends commands and parses telemetry over a Transport."""

    def __init__(self, transport: Transport, clock: Clock,
                 heartbeat_period: float = 0.05) -> None:
        self._transport = transport
        self._clock = clock
        self._parser = StreamParser()
        self._seq = 0
        self._heartbeat_period = heartbeat_period
        self._last_heartbeat_sent = float("-inf")
        self.telemetry = McuTelemetry()
        self._handlers: dict[str, list[Callable[[dict], None]]] = {}
        self.last_rx_time = float("-inf")

    # -------------------------------------------------------------- commands
    def _send(self, msg_name: str, **values) -> None:
        payload = encode_payload(msg_name, **values)
        frame = Frame(seq=self._seq, msg_id=_id_of(msg_name), payload=payload)
        self._seq = (self._seq + 1) & 0xFF
        self._transport.send(encode_frame(frame))

    def send_heartbeat(self) -> None:
        self._send("HEARTBEAT", uptime_ms=int(self._clock.now() * 1000) & 0xFFFFFFFF)

    def send_velocity(self, vx: float, vy: float, wz: float) -> None:
        self._send("CMD_VELOCITY", vx=vx, vy=vy, wz=wz)

    def send_stop(self, brake: bool = False) -> None:
        self._send("CMD_STOP", mode=1 if brake else 0)

    def send_line_follow_start(self, segment_id: int, target_speed: float) -> None:
        self._send("CMD_LINE_FOLLOW_START", segment_id=segment_id & 0xFFFF,
                   target_speed=target_speed)

    def send_line_follow_stop(self) -> None:
        self._send("CMD_LINE_FOLLOW_STOP")

    def send_line_follow_config(self, kp: float, ki: float, kd: float) -> None:
        self._send("CMD_LINE_FOLLOW_CONFIG", kp=kp, ki=ki, kd=kd)

    def send_linear_axis(self, axis: int, position_mm: float, speed: float = 0.0) -> None:
        self._send("CMD_LINEAR_AXIS", axis=axis, position_mm=position_mm, speed=speed)

    def send_gripper(self, close: bool) -> None:
        self._send("CMD_GRIPPER", command=GRIPPER_CLOSE if close else GRIPPER_OPEN)

    def send_home(self, axis: int = AXIS_ALL) -> None:
        self._send("CMD_HOME", axis=axis)

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
        if spec.name == "MCU_HEARTBEAT":
            t.heartbeat_age = 0.0
            t.fault_flags = values.get("fault_flags", 0)
        elif spec.name == "ODOMETRY":
            t.odom = OdometrySample(timestamp=now, **values)
        elif spec.name == "IMU":
            t.imu = ImuSample(timestamp=now, **values)
        elif spec.name == "LINE_STATE":
            t.line = LineFollowStatus(
                line_detected=bool(values["line_detected"]),
                line_error=values["line_error"],
                confidence=values["confidence"],
                controller_state=values["controller_state"],
                intersection_detected=bool(values["intersection_detected"]),
                fault=bool(values["fault"]),
                timestamp=now,
            )
        elif spec.name == "MANIPULATOR_STATE":
            t.manipulator_x_mm = values["x_mm"]
            t.manipulator_z_mm = values["z_mm"]
            t.manipulator_homed = bool(values["homed"])
            t.manipulator_gripper = _GRIPPER_STATES.get(
                values["gripper_state"], GripperState.UNKNOWN)
            t.manipulator_grip_detected = bool(values["grip_detected"])
            t.manipulator_moving = bool(values["moving"])
            t.manipulator_fault = bool(values["fault"])
            t.manipulator_fault_code = values["fault_code"]
        elif spec.name == "START_EVENT":
            t.start_button = bool(values["button_state"])
        elif spec.name == "FAULT":
            t.fault_code = values["fault_code"]
            t.fault_detail = values["detail"]
            log.error("MCU FAULT code=%s detail=%s", values["fault_code"], values["detail"])
        else:
            log.debug("unhandled telemetry %s", spec.name)

        for handler in self._handlers.get(spec.name, []):
            handler(values)

    # -------------------------------------------------------------- health
    def tick(self) -> None:
        """Periodic maintenance: heartbeat + poll."""
        now = self._clock.now()
        if now - self._last_heartbeat_sent >= self._heartbeat_period:
            self.send_heartbeat()
            self._last_heartbeat_sent = now
        self.poll()

    @property
    def link_alive(self) -> bool:
        return self._clock.now() - self.last_rx_time < 1.0

    @property
    def stats(self):
        return self._parser.stats


def _id_of(msg_name: str) -> int:
    from core.protocol.messages import MESSAGES
    return MESSAGES[msg_name].msg_id
