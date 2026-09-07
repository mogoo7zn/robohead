"""MockMissionRunner — a complete simulated match, zero hardware.

This is the flagship "runs on a Mac" entry point (`make mock`). It wires
the REAL software stack (assembled by MissionStack) around simulated
leaves:

  Mission layer      Planner / HFSM / WorldState            (real code)
  Skill layer        Alignment / Grab / Place / Navigator   (real code)
  Service layer      Localization / Mock perception         (real code)
  Hardware bridge    McuClient <-> MemoryTransport          (real code)
  Leaves             FakeSTM32 -> SimWorld                  (mock backend)

Swapping to the real robot replaces exactly the leaves: the Transport
(memory -> serial) and the two detectors (mock -> camera backends). The
assembly itself is shared with the ROS 2 bringup node via MissionStack.

Run:  python -m mock.runner [--realtime] [--max-time 420]
"""
from __future__ import annotations

import argparse
import logging
import math
import sys
from dataclasses import dataclass, field

from core.hardware.transport import MemoryTransport
from core.localization.camera import CameraParams
from core.localization.marker_map import MarkerMap
from core.mission.hfsm import DONE
from core.mission.stack import MissionStack
from core.mock.fake_stm32 import FakeSTM32
from core.mock.sim_world import LineSegment, SimBlock, SimWorld
from core.model.pose import Pose2D
from core.navigation.route_graph import RouteGraph
from core.perception.block_detector import MockBlockDetector
from core.perception.marker_detector import MockMarkerDetector
from core.utils.clock import FakeClock
from core.utils.config import load_yaml
from core.utils.log import get_logger

log = get_logger("mock.runner")


@dataclass
class MockMissionResult:
    """Everything a test or the CLI needs to judge the simulated match."""

    finished: bool = False                  # match reached a terminal state
    reason: str = ""
    sim_time: float = 0.0
    final_state: str = ""
    state_history: list[str] = field(default_factory=list)
    placed_total: int = 0
    tower_heights: dict = field(default_factory=dict)
    tower_purple_top: dict = field(default_factory=dict)
    inventory: tuple[int, int] = (0, 0)     # orange, purple still carried
    watchdog_tripped: bool = False
    timeouts: int = 0

    @property
    def ok(self) -> bool:
        return (self.finished and not self.watchdog_tripped
                and self.final_state in ("SAFE_STOP", "ENDGAME/SAFE_STOP"))


class MockMissionRunner:
    """Builds the full mock stack and ticks it until the match ends."""

    def __init__(self, config_dir: str = "config",
                 dt: float = 0.02,
                 start_button_delay: float = 5.0,
                 marker_period: float = 0.10,
                 realtime: bool = False,
                 log_level: int = logging.INFO) -> None:
        self.dt = dt
        self.start_button_delay = start_button_delay
        self.marker_period = marker_period
        self.realtime = realtime
        self.log_level = log_level
        self._started = False
        self.trace: list[str] = []          # 1 Hz state trace for debugging

        logging.getLogger("robogame").setLevel(log_level)

        self.clock = FakeClock(0.0)

        # ---- simulated vehicle --------------------------------------------
        field_cfg = load_yaml(f"{config_dir}/mock_field.yaml")
        manip_cfg = load_yaml(f"{config_dir}/manipulation.yaml")
        hw_cfg = load_yaml(f"{config_dir}/hardware.yaml")

        start = field_cfg["start_pose"]
        self.start_pose = Pose2D(float(start["x"]), float(start["y"]),
                                 float(start.get("yaw", 0.0)))
        self.world = SimWorld()
        self.world.reset(self.start_pose)
        # physical lines: one segment per directed route edge
        self.graph = RouteGraph.from_config(
            load_yaml(f"{config_dir}/mock_routes.yaml"))
        self.world.line_segments = []
        for node_id in self.graph.node_ids:
            a = self.graph.node(node_id)
            for edge in self.graph.edges_from(node_id):
                b = self.graph.node(edge.to_node)
                self.world.line_segments.append(
                    LineSegment(a.x, a.y, b.x, b.y))
        for blk in field_cfg.get("blocks", []):
            self.world.blocks.append(SimBlock(
                block_type=str(blk["type"]),
                x=float(blk["x"]), y=float(blk["y"]),
                yaw=float(blk.get("yaw", 0.0))))
        initial_supply = (
            sum(1 for b in self.world.blocks if b.block_type == "ORANGE"),
            sum(1 for b in self.world.blocks if b.block_type == "PURPLE"))

        # ---- protocol stack: Pi <-> memory wire <-> fake MCU --------------
        t_pi, t_mcu = MemoryTransport.create_pair()
        mcu_cfg = hw_cfg.get("mcu", {})
        self.fake_mcu = FakeSTM32(
            t_mcu, self.world, self.clock,
            telemetry_period=self.dt,
            gripper_duration=float(
                manip_cfg.get("gripper", {}).get("close_time", 1.0)),
            watchdog_timeout=float(mcu_cfg.get("heartbeat_timeout", 0.2)))

        # ---- perception mocks (render SimWorld through camera models) ----
        marker_map = MarkerMap.from_config(field_cfg)
        loc_camera = CameraParams.from_config(field_cfg["localization_camera"])
        blk_camera = CameraParams.from_config(field_cfg["block_camera"])
        block_size = float(field_cfg.get("block", {}).get("size", 0.07))
        marker_detector = MockMarkerDetector(
            marker_map, loc_camera,
            pose_provider=lambda: self.world.robot_pose)
        block_detector = MockBlockDetector(
            blk_camera,
            blocks_provider=lambda: self.world.blocks,
            pose_provider=lambda: self.world.robot_pose,
            block_size=block_size)

        # ---- the shared assembly (identical to the real robot) -----------
        self.stack = MissionStack(
            config_dir, self.clock, transport=t_pi,
            marker_detector=marker_detector, block_detector=block_detector,
            initial_supply=initial_supply)
        # expose commonly used services for tests / tracing
        self.store = self.stack.store
        self.match = self.stack.match
        self.planner = self.stack.planner
        self.navigator = self.stack.navigator
        self.localization = self.stack.localization
        self.client = self.stack.client
        self.machine = self.stack.machine
        self.towers = self.stack.towers

    @staticmethod
    def _start_pose(field_cfg: dict):
        from core.model.pose import Pose2D
        start = field_cfg["start_pose"]
        return Pose2D(float(start["x"]), float(start["y"]),
                     float(start.get("yaw", 0.0)))

    # ------------------------------------------------------------------- run
    def run(self, max_time: float = 420.0) -> MockMissionResult:
        log.info("=== RoboGame 2026 MOCK MISSION — dt=%.3fs, start button in "
                 "%.0fs ===", self.dt, self.start_button_delay)
        result = MockMissionResult()
        t = self.clock.now()
        next_marker_time = t
        next_start_button = t + self.start_button_delay

        while True:
            t = self.clock.advance(self.dt)
            if not self._started:
                self._started = True
                self.machine.start()

            # 1. MCU: apply pending commands, advance physics, emit telemetry
            self.fake_mcu.tick(self.dt)

            # 2. markers at camera fps, then the shared control pipeline
            markers = None
            if t >= next_marker_time:
                next_marker_time = t + self.marker_period
                markers = self.stack.marker_detector.detect(t)
            # start button rises inside SimWorld; telemetry carries it
            if t >= next_start_button:
                next_start_button = math.inf
                self.world.start_button = True
            outcome = self.stack.tick(markers)

            if int(t / self.dt) % int(1.0 / self.dt) == 0:   # ~1 Hz
                pose = self.world.robot_pose
                self.trace.append(
                    f"t={t:6.1f}|{self.machine.full_state_name:<26} "
                    f"nav={self.navigator.mode:<6} "
                    f"line={'Y' if self.world.line_detected else 'n'} "
                    f"pose=({pose.x:5.2f},{pose.y:5.2f},{pose.yaw:+5.2f}) "
                    f"loc=({self.localization.pose.x:5.2f},"
                    f"{self.localization.pose.y:5.2f},"
                    f"{self.localization.pose.yaw:+5.2f})")

            if outcome == DONE:
                result.reason = "hfsm finished"
                break
            if self.match.finished:
                result.reason = "match finished"
                break
            if t >= max_time:
                result.reason = f"sim exceeded max time {max_time:.0f}s"
                break
            if self.realtime:
                import time as _time
                _time.sleep(self.dt)

        return self._collect(result)

    # --------------------------------------------------------------- collect
    def _collect(self, result: MockMissionResult) -> MockMissionResult:
        snap = self.store.snapshot()
        result.finished = self.match.finished or result.reason == "hfsm finished"
        result.sim_time = self.clock.now()
        result.final_state = self.machine.full_state_name
        result.state_history = list(self.machine.history)
        result.placed_total = snap.building.placed_total
        result.tower_heights = dict(snap.building.tower_heights)
        result.tower_purple_top = dict(snap.building.tower_purple_top)
        result.inventory = (snap.inventory.orange_count,
                            snap.inventory.purple_count)
        result.watchdog_tripped = self.fake_mcu.watchdog_tripped
        result.timeouts = self.machine.timeout_count

        log.info("=== MOCK MISSION OVER: %s at t=%.1fs ===",
                 result.reason, result.sim_time)
        log.info("final state: %s | placed: %d %s | carried: O=%d P=%d "
                 "| mcu watchdog tripped: %s",
                 result.final_state, result.placed_total,
                 result.tower_heights, *result.inventory,
                 result.watchdog_tripped)
        for name, (x, y) in self.towers.items():
            log.info("tower %s at (%.2f, %.2f): height=%d purple_top=%s",
                     name, x, y, result.tower_heights.get(name, 0),
                     result.tower_purple_top.get(name, False))
        return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a complete simulated RoboGame match (no hardware)")
    parser.add_argument("--realtime", action="store_true",
                        help="pace the simulation to wall-clock time")
    parser.add_argument("--max-time", type=float, default=420.0,
                        help="simulation time budget (s)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="debug logging")
    args = parser.parse_args(argv)

    runner = MockMissionRunner(
        realtime=args.realtime,
        log_level=logging.DEBUG if args.verbose else logging.INFO)
    result = runner.run(max_time=args.max_time)
    return 0 if result.ok else 1


if __name__ == "__main__":
    sys.exit(main())
