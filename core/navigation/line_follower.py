"""Pi-side line-following controller.

Protocol v1.1 has no line-follow command: the STM32 only reports LINE_SENSOR
detections, so the closed loop runs here on the Pi and outputs CMD_VEL —
the same twist the Navigator would send.

    Control law ("follow the carrot" using the lateral offset only):
when the line is detected, steer toward a virtual carrot at LOOKAHEAD
metres ahead on the line. Protocol §6.2: offset is positive when the
robot is right of the line, i.e. the line lies to the robot's LEFT
(body frame, x-forward y-left), so a positive offset requires a
positive (counter-clockwise) correction:

    wz = +2 * vx * offset / LOOKAHEAD^2

For small errors this is stable for any entry combination of lateral +
heading error (a pure P-law on offset is an undamped oscillator and
overshoots). When the line is lost, creep forward while steering back
toward the last reported offset (same sign convention).
"""
from __future__ import annotations

from core.hardware.mcu_client import LineSensorSample

DEFAULT_CONFIG = {
    "lookahead_m": 0.20,     # virtual carrot distance (m)
    "creep_speed": 0.08,     # m/s while line lost
    "lost_kp": 3.0,          # steering gain while lost
    "max_wz": 1.2,           # rad/s clamp
}


class LineFollower:
    def __init__(self, config: dict | None = None) -> None:
        self._cfg = {**DEFAULT_CONFIG, **(config or {})}
        self._active = False
        self._target_speed = 0.0
        self._last_offset: float | None = None

    @property
    def active(self) -> bool:
        return self._active

    def start(self, speed: float) -> None:
        self._target_speed = speed
        self._active = True
        # A stale offset from a previous segment (different line, taken
        # under a different heading) would steer the recovery the wrong
        # way — each segment starts with a clean slate.
        self._last_offset = None

    def stop(self) -> None:
        self._active = False
        self._target_speed = 0.0

    def update(self, line: LineSensorSample) -> tuple[float, float, float]:
        """One control step; returns the (vx, vy, wz) body twist to send."""
        if not self._active:
            return (0.0, 0.0, 0.0)
        max_wz = self._cfg["max_wz"]
        if line.line_detected:
            self._last_offset = line.offset_m
            vx = self._target_speed
            l2 = self._cfg["lookahead_m"] ** 2
            wz = 2.0 * vx * line.offset_m / l2
        elif self._last_offset is not None:
            # lost after seeing the line: creep and steer back toward
            # where it was last seen (same heading as when it was seen)
            vx = self._cfg["creep_speed"]
            wz = self._cfg["lost_kp"] * self._last_offset
        else:
            # never saw the line on this segment: the TURN aimed us at
            # the target node, so creep straight and let the path cross
            # the line (the approach is only ever centimetres off)
            vx = self._cfg["creep_speed"]
            wz = 0.0
        wz = max(-max_wz, min(max_wz, wz))
        return (vx, 0.0, wz)
