"""MissionStack — backend-agnostic assembly of the complete mission software.

Both entry points build their stack through this class:

  MockMissionRunner (Mac)              robogame_bringup (Raspberry Pi)
  ------------------------------------------------ ------------------
  FakeClock                             RealClock
  MemoryTransport pair + FakeSTM32       SerialTransport -> real STM32
  MockMarkerDetector (SimWorld)         camera / AprilTag backend
  MockBlockDetector (SimWorld)          camera / HSV backend

Everything above those leaves — protocol client, chassis bridge, manipulator
sequencer, skills, localization fusion, navigation, planner, watchdog and
the HFSM — is assembled here once and shared verbatim, so moving to real
hardware touches zero mission-layer code.
"""
from __future__ import annotations

from typing import Protocol

from core.hardware.mcu_client import McuClient
from core.hardware.transport import Transport, TransportError, create_transport
from core.localization.camera import CameraParams
from core.localization.localization import Localization
from core.localization.marker_map import MarkerMap
from core.localization.pose_estimator import MarkerObservation, PoseEstimator
from core.manipulation.manipulator import (
    ManipulatorSequencer,
    McuManipulator,
)
from core.mission.context import MissionContext
from core.mission.hfsm import DONE
from core.mission.match_manager import MatchManager
from core.mission.states import build_match_machine
from core.mission.watchdog import Watchdog
from core.model.pose import Pose2D
from core.navigation.chassis import McuChassis
from core.navigation.navigator import Navigator
from core.navigation.route_graph import RouteGraph
from core.skill.alignment import AlignmentController
from core.skill.grab import GrabSkill
from core.skill.place import PlaceSkill
from core.state.events import (
    FieldSupplyEvent,
    PoseUpdatedEvent,
    RouteNodeEvent,
)
from core.state.store import WorldStateStore
from core.strategy.planner import Planner
from core.utils.clock import Clock
from core.utils.config import load_yaml
from core.utils.log import get_logger

log = get_logger("mission.stack")


class MarkerDetectorLike(Protocol):
    def detect(self, timestamp: float) -> list[MarkerObservation]: ...


class BlockDetectorLike(Protocol):
    def detect(self, timestamp: float) -> list: ...


class MissionStack:
    """The full mission software with injectable leaf backends.

    Parameters
    ----------
    config_dir:      directory with the yaml files (mock_field, routes, ...)
    clock:           FakeClock (tests/mock) or RealClock (Pi)
    transport:       byte pipe to the MCU (SerialTransport on the robot);
                     if None, one is created from hardware.yaml
    marker_detector: localization-camera backend
    block_detector:  block-camera backend
    field_file:      field layout yaml (markers, towers, zones, start pose)
    initial_supply:  (orange, purple) free blocks at match start; if None
                     the field file's `supply:` section is used
    """

    def __init__(self, config_dir: str, clock: Clock,
                 transport: Transport | None = None,
                 marker_detector: MarkerDetectorLike | None = None,
                 block_detector: BlockDetectorLike | None = None,
                 field_file: str = "mock_field.yaml",
                 initial_supply: tuple[int, int] | None = None) -> None:
        self.clock = clock
        field_cfg = load_yaml(f"{config_dir}/{field_file}")
        routes_cfg = load_yaml(f"{config_dir}/mock_routes.yaml")
        strategy_cfg = load_yaml(f"{config_dir}/strategy.yaml")
        nav_cfg = load_yaml(f"{config_dir}/navigation.yaml")
        manip_cfg = load_yaml(f"{config_dir}/manipulation.yaml")
        perc_cfg = load_yaml(f"{config_dir}/perception.yaml")
        loc_cfg = load_yaml(f"{config_dir}/localization.yaml")
        hw_cfg = load_yaml(f"{config_dir}/hardware.yaml")

        # ---- route graph / field geometry --------------------------------
        self.graph = RouteGraph.from_config(routes_cfg)
        start = field_cfg["start_pose"]
        self.start_pose = Pose2D(float(start["x"]), float(start["y"]),
                                 float(start.get("yaw", 0.0)))
        self.towers = {name: (float(t["x"]), float(t["y"]))
                       for name, t in field_cfg.get("towers", {}).items()}

        # ---- protocol stack: Pi <-> MCU ----------------------------------
        if transport is None:
            transport = create_transport(hw_cfg.get("mcu", {}))
        self.transport = transport
        mcu_cfg = hw_cfg.get("mcu", {})
        self.client = McuClient(
            transport, clock,
            heartbeat_period=float(mcu_cfg.get("heartbeat_period", 0.05)))
        self.heartbeat_timeout = float(mcu_cfg.get("heartbeat_timeout", 0.2))

        # ---- hardware-bridge backends (over the protocol) ----------------
        self.chassis = McuChassis(self.client)
        manip_backend = McuManipulator(self.client)
        self.sequencer = ManipulatorSequencer(manip_backend, manip_cfg, clock)
        self.grab_skill = GrabSkill(self.sequencer, manip_cfg.get("grab"), clock)
        self.place_skill = PlaceSkill(self.sequencer, self.chassis,
                                      manip_cfg.get("place"), clock)
        self.alignment = AlignmentController(perc_cfg.get("alignment"), clock)

        # ---- perception backends (injected) ------------------------------
        self.marker_detector = marker_detector
        self.block_detector = block_detector

        # ---- localization fusion ------------------------------------------
        marker_map = MarkerMap.from_config(field_cfg)
        self.loc_camera = CameraParams.from_config(field_cfg["localization_camera"])
        self.localization = Localization(
            PoseEstimator(marker_map, self.loc_camera), loc_cfg, clock)

        # ---- navigation ----------------------------------------------------
        nav_params = {**nav_cfg.get("line_follow", {}),
                      **nav_cfg.get("navigator", {})}
        self.navigator = Navigator(self.graph, self.chassis, nav_params,
                                   clock=clock, on_node_event=self._on_route_node)

        # ---- mission services ---------------------------------------------
        self.store = WorldStateStore()
        self.match = MatchManager(self.store, clock, **{
            "total_time": strategy_cfg["match"]["total_time"],
            "normal_end_time": strategy_cfg["match"]["normal_end_time"],
            "safe_end_time": strategy_cfg["match"]["safe_end_time"]})
        self.planner = Planner(strategy_cfg)
        self.watchdog = Watchdog(self.store, clock, hw_cfg)

        self.ctx = MissionContext(
            store=self.store, clock=clock, match=self.match,
            planner=self.planner, watchdog=self.watchdog,
            navigator=self.navigator, localization=self.localization,
            marker_detector=self.marker_detector,
            block_detector=self.block_detector, alignment=self.alignment,
            grab_skill=self.grab_skill, place_skill=self.place_skill,
            chassis=self.chassis, start_pose=self.start_pose,
            towers=self.towers)
        self.machine = build_match_machine(self.ctx)

        # ---- initial field supply for the planner -------------------------
        supply = initial_supply
        if supply is None:
            sup_cfg = field_cfg.get("supply", {})
            supply = (int(sup_cfg.get("orange", 0)),
                      int(sup_cfg.get("purple", 0)))
        self.store.apply_event(FieldSupplyEvent(
            orange_remaining=supply[0], purple_remaining=supply[1]))

    # ------------------------------------------------------------------ node
    def _on_route_node(self, node: str, segment: str, marker: str) -> None:
        self.store.apply_event(RouteNodeEvent(
            current_node=node, current_segment=segment,
            expected_marker=marker, line_follow_active=True))

    # --------------------------------------------------------------- pipeline
    def tick_sensors(self,
                     marker_detections: list[MarkerObservation] | None = None
                     ) -> None:
        """Telemetry in -> sensor fusion -> watchdog health -> pose event.

        Called once per control period by both the mock runner and the ROS
        bringup node. `marker_detections` is whatever the localization camera
        produced since the last tick (None = no new frame this period).
        """
        self.client.tick()
        tele = self.client.telemetry

        self.localization.update_odometry(
            Pose2D(tele.odom.x, tele.odom.y, tele.odom.yaw))
        self.localization.update_imu_yaw(tele.imu.yaw)
        self.watchdog.observe_odom()
        self.watchdog.observe_imu()
        self.watchdog.observe_localization()
        if self.client.link_alive:
            self.watchdog.observe_mcu()
        if marker_detections is not None:
            self.localization.update_markers(marker_detections)
        # cameras are assumed streaming (mock continuously, ROS via callbacks)
        self.watchdog.observe_localization_camera()
        self.watchdog.observe_block_camera()
        self.store.apply_event(PoseUpdatedEvent(
            pose=self.localization.pose,
            confidence=self.localization.confidence))

        # start signal: start button -> match clock
        if tele.start_button and not self.match.started:
            self.match.start()
            log.info("match started at t=%.1fs", self.clock.now())

    def tick_mission(self) -> str | None:
        """Advance the match clock one step and tick the HFSM."""
        self.match.tick()
        return self.machine.tick()

    def tick(self,
             marker_detections: list[MarkerObservation] | None = None
             ) -> str | None:
        """One full control period: sensors + mission."""
        self.tick_sensors(marker_detections)
        return self.tick_mission()

    @property
    def finished(self) -> bool:
        return self.match.finished or self.machine.history[-1:] == ["SAFE_STOP"]

    # --------------------------------------------------------------- shutdown
    def shutdown(self) -> None:
        """Halt chassis and close the transport (idempotent-ish, best effort)."""
        try:
            self.ctx.stop_chassis()
        except Exception as exc:  # pragma: no cover - defensive on real hw
            log.warning("shutdown: chassis stop failed: %s", exc)
        try:
            self.transport.close()
        except Exception as exc:  # pragma: no cover - defensive on real hw
            log.warning("shutdown: transport close failed: %s", exc)


__all__ = ["MissionStack", "DONE", "TransportError",
           "MarkerDetectorLike", "BlockDetectorLike"]
