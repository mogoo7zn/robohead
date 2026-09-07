"""FakeSTM32 — a software stand-in for the real MCU.

Speaks the exact binary protocol over a Transport, drives a SimWorld, and
emits telemetry (odometry, IMU, line state, manipulator state, heartbeat,
start events). Used on Mac / in CI so the whole mission runs without any
hardware.

The real STM32 firmware (firmware/stm32) implements the same protocol on
the other side of the wire — swapping FakeSTM32 for the real MCU changes
nothing above the Transport.
"""
from __future__ import annotations

from core.hardware.transport import Transport
from core.mock.sim_world import SimWorld
from core.model.enums import GripperState
from core.protocol import (
    AXIS_X,
    AXIS_Z,
    Frame,
    StreamParser,
    decode_payload,
    encode_frame,
    encode_payload,
    spec_by_id,
    spec_by_name,
)
from core.protocol.messages import (
    GRIPPER_CLOSE,
    LINE_CTRL_IDLE,
    LINE_CTRL_LOST,
    LINE_CTRL_RUNNING,
)
from core.utils.clock import Clock
from core.utils.log import get_logger

log = get_logger("mock.fake_stm32")

_GRIPPER_CODES = {
    GripperState.UNKNOWN: 0,
    GripperState.OPEN: 1,
    GripperState.CLOSED: 2,
    GripperState.HOLDING: 3,
    GripperState.FAULT: 4,
}


class FakeSTM32:
    """Protocol-level emulation of the STM32 firmware."""

    def __init__(self, transport: Transport, world: SimWorld, clock: Clock,
                 telemetry_period: float = 0.02,
                 gripper_duration: float = 1.0,
                 watchdog_timeout: float = 0.2) -> None:
        self._transport = transport
        self._world = world
        self._clock = clock
        self._parser = StreamParser()
        self._seq = 0
        self._telemetry_period = telemetry_period
        self._last_telemetry = float("-inf")
        self._last_heartbeat_rx = float("-inf")
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
                if self._parser.stats.frames_ok:
                    pass

        # 2. MCU watchdog: Pi heartbeats must keep arriving
        if self._last_heartbeat_rx > float("-inf"):
            if now - self._last_heartbeat_rx > self._watchdog_timeout:
                if not self.watchdog_tripped:
                    log.warning("watchdog trip: no Pi heartbeat")
                self.watchdog_tripped = True
                self._world.velocity_cmd = (0.0, 0.0, 0.0)
                self._world.stopped = True
                self._world.line_follow_active = False
            else:
                self.watchdog_tripped = False

        # 3. simulate
        self._world.step(dt, self._clock)

        # 4. telemetry
        if now - self._last_telemetry >= self._telemetry_period:
            self._last_telemetry = now
            self._emit_telemetry(now)

    # -------------------------------------------------------------- commands
    def _send(self, msg_name: str, **values) -> None:
        payload = encode_payload(msg_name, **values)
        frame = Frame(seq=self._seq, msg_id=spec_by_name(msg_name).msg_id, payload=payload)
        self._seq = (self._seq + 1) & 0xFF
        self._transport.send(encode_frame(frame))

    def _handle_frame(self, frame: Frame, now: float) -> None:
        spec = spec_by_id(frame.msg_id)
        if spec is None:
            return
        if spec.direction != "pi_to_mcu":
            return
        values = decode_payload(spec.name, frame.payload)

        if spec.name == "HEARTBEAT":
            self._last_heartbeat_rx = now
        elif spec.name == "CMD_STOP":
            self._world.velocity_cmd = (0.0, 0.0, 0.0)
            self._world.stopped = True
            self._world.line_follow_active = False
        elif spec.name == "CMD_VELOCITY":
            if not self.watchdog_tripped:
                self._world.velocity_cmd = (values["vx"], values["vy"], values["wz"])
                self._world.stopped = False
                self._world.line_follow_active = False
        elif spec.name == "CMD_LINE_FOLLOW_START":
            if not self.watchdog_tripped:
                self._world.line_follow_active = True
                self._world.line_follow_speed = values["target_speed"]
                self._world.stopped = False
        elif spec.name == "CMD_LINE_FOLLOW_STOP":
            self._world.line_follow_active = False
            self._world.velocity_cmd = (0.0, 0.0, 0.0)
            self._world.stopped = True
        elif spec.name == "CMD_LINE_FOLLOW_CONFIG":
            self._world.line_pid = (values["kp"], values["ki"], values["kd"])
        elif spec.name == "CMD_LINEAR_AXIS":
            axis = "X" if values["axis"] == AXIS_X else "Z" if values["axis"] == AXIS_Z else "X"
            self._world.manip_move_axis(axis, values["position_mm"], values["speed"])
        elif spec.name == "CMD_GRIPPER":
            close = values["command"] == GRIPPER_CLOSE
            self._world.manip_set_gripper(close, self._gripper_duration, self._clock)
        elif spec.name == "CMD_HOME":
            self._world.manip_home()
        else:
            log.debug("fake mcu ignores %s", spec.name)

    # -------------------------------------------------------------- telemetry
    def _emit_telemetry(self, now: float) -> None:
        w = self._world
        uptime = int((now - self.start_time) * 1000) & 0xFFFFFFFF
        self._send("MCU_HEARTBEAT", uptime_ms=uptime, fault_flags=0)

        self._send("ODOMETRY",
                   x=w.odom_x, y=w.odom_y, yaw=w.odom_yaw,
                   vx=w.velocity_cmd[0], vy=w.velocity_cmd[1], wz=w.velocity_cmd[2])

        self._send("IMU",
                   yaw=w.imu_yaw, gyro_z=w.velocity_cmd[2],
                   accel_x=0.0, accel_y=0.0)

        if w.line_follow_active:
            if w.line_detected:
                ctrl = LINE_CTRL_RUNNING
            elif w.line_confidence <= 0.0:
                ctrl = LINE_CTRL_LOST
            else:
                ctrl = LINE_CTRL_RUNNING
        else:
            ctrl = LINE_CTRL_IDLE
        self._send("LINE_STATE",
                   line_detected=1 if w.line_detected else 0,
                   line_error=w.line_error,
                   confidence=w.line_confidence,
                   controller_state=ctrl,
                   intersection_detected=0,
                   fault=0)

        self._send("MANIPULATOR_STATE",
                   x_mm=w.manip_x_mm, z_mm=w.manip_z_mm,
                   homed=1 if w.manip_homed else 0,
                   gripper_state=_GRIPPER_CODES[w.manip_gripper],
                   grip_detected=1 if w.manip_grip_detected else 0,
                   moving=1 if w.manip_moving else 0,
                   fault=0, fault_code=0)

        if w.start_button:
            self._send("START_EVENT", button_state=1)
            w.start_button = False  # edge-triggered
