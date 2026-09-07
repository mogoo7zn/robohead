"""High-level Watchdog.

Checks the health of: MCU link, both cameras, line sensor, localization,
manipulator. Applies HardwareStatusEvent to WorldState and decides whether
the mission can continue (RECOVERY) or must stop (SAFE_STOP).
"""
from __future__ import annotations

from enum import Enum

from core.state.events import HardwareStatusEvent
from core.state.store import WorldStateStore
from core.utils.clock import Clock
from core.utils.log import get_logger

log = get_logger("mission.watchdog")


class WatchdogVerdict(Enum):
    OK = "OK"
    RECOVER = "RECOVER"       # degraded -> enter RECOVERY state
    SAFE_STOP = "SAFE_STOP"  # critical -> stop everything


class Watchdog:
    def __init__(self, store: WorldStateStore, clock: Clock, config: dict) -> None:
        self._store = store
        self._clock = clock
        cfg = config.get("watchdog", {})
        self._timeouts = {
            "mcu": cfg.get("high_level_timeout", 1.0),
            "camera": cfg.get("camera_timeout", 5.0),
            "localization": cfg.get("localization_timeout", 10.0),
            "imu": cfg.get("imu_timeout", 1.0),
            "odom": cfg.get("odom_timeout", 1.0),
        }
        # last-seen timestamps, filled by the runner / bridges
        self.last_mcu_rx: float = float("-inf")
        self.last_localization_camera: float = float("-inf")
        self.last_block_camera: float = float("-inf")
        self.last_localization: float = float("-inf")
        self.last_imu: float = float("-inf")
        self.last_odom: float = float("-inf")
        self.emergency_stop: bool = False

    def observe_mcu(self) -> None:
        self.last_mcu_rx = self._clock.now()

    def observe_localization_camera(self) -> None:
        self.last_localization_camera = self._clock.now()

    def observe_block_camera(self) -> None:
        self.last_block_camera = self._clock.now()

    def observe_localization(self) -> None:
        self.last_localization = self._clock.now()

    def observe_imu(self) -> None:
        self.last_imu = self._clock.now()

    def observe_odom(self) -> None:
        self.last_odom = self._clock.now()

    # -------------------------------------------------------------- check
    def _age(self, last: float) -> float:
        if last == float("-inf"):
            return float("inf")
        return self._clock.now() - last

    def tick(self) -> WatchdogVerdict:
        mcu_ok = self._age(self.last_mcu_rx) <= self._timeouts["mcu"]
        loc_cam_ok = self._age(self.last_localization_camera) <= self._timeouts["camera"]
        block_cam_ok = self._age(self.last_block_camera) <= self._timeouts["camera"]
        loc_ok = self._age(self.last_localization) <= self._timeouts["localization"]
        imu_ok = self._age(self.last_imu) <= self._timeouts["imu"]

        self._store.apply_event(HardwareStatusEvent(
            camera_localization_ok=loc_cam_ok,
            camera_block_ok=block_cam_ok,
            mcu_ok=mcu_ok,
            line_sensor_ok=mcu_ok,       # line sensor data arrives via MCU
            imu_ok=imu_ok,
            chassis_ok=mcu_ok,
            manipulator_ok=mcu_ok,
        ))

        verdict = WatchdogVerdict.OK
        if self.emergency_stop:
            verdict = WatchdogVerdict.SAFE_STOP
        elif not mcu_ok:
            verdict = WatchdogVerdict.SAFE_STOP   # no MCU -> cannot move safely
        elif not loc_ok or not loc_cam_ok:
            verdict = WatchdogVerdict.RECOVER      # localization trouble

        if verdict is not WatchdogVerdict.OK:
            log.warning("watchdog verdict=%s mcu=%s loc=%s loccam=%s blockcam=%s",
                        verdict.value, mcu_ok, loc_ok, loc_cam_ok, block_cam_ok)
        return verdict
