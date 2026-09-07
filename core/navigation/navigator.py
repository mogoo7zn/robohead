"""Navigator — executes a RoutePlan segment by segment.

Per segment (edge):
  1. TURN  : rotate in place until facing the segment direction
  2. FOLLOW: STM32 line-follows the segment at the edge's speed
  3. ARRIVE: pose enters the node's arrival radius -> stop or continue

The Navigator never trusts the line controller for arrival: it monitors
the fused pose (localization) and decides arrival itself, issuing
CMD_LINE_FOLLOW_STOP — the same split used on the real robot
(MCU follows the line, Pi decides where the robot is).

Returns SkillStatus on every update; emits RouteNodeEvent data through
an optional callback so the store stays single-writer.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

from core.model.enums import SkillStatus
from core.model.pose import Pose2D, angle_diff
from core.navigation.chassis import ChassisCommander
from core.navigation.route_graph import RouteEdge, RouteGraph
from core.navigation.route_planner import RoutePlan, RoutePlanner
from core.utils.clock import Clock, FakeClock
from core.utils.log import get_logger

log = get_logger("navigation.navigator")

DEFAULT_CONFIG = {
    "default_speed": 0.35,
    "approach_speed": 0.15,
    "arrival_radius": 0.12,
    "marker_arrival_radius": 0.5,
    "segment_timeout": 120.0,
    "max_wz": 1.0,
    "kp_yaw": 1.5,
    "turn_tolerance": 0.10,
    "turn_timeout": 10.0,
}


@dataclass(frozen=True)
class NavigationStatus:
    status: SkillStatus
    current_node: str = ""
    current_segment: str = ""
    expected_marker: str = ""
    message: str = ""


class Navigator:
    """Follows a planned route; the only chassis commander user in missions."""

    def __init__(self, graph: RouteGraph, chassis: ChassisCommander,
                 config: dict | None = None,
                 clock: Clock | None = None,
                 on_node_event: Callable[[str, str, str], None] | None = None
                 ) -> None:
        self._graph = graph
        self._planner = RoutePlanner(graph)
        self._chassis = chassis
        self._cfg = {**DEFAULT_CONFIG, **(config or {})}
        self._clock = clock or FakeClock()
        self._on_node_event = on_node_event

        self._plan: RoutePlan | None = None
        self._edge_index = -1
        self._current_edge: RouteEdge | None = None
        self._mode = "IDLE"          # IDLE | TURN | FOLLOW | DONE
        self._segment_start_time = 0.0

    # ------------------------------------------------------------- lifecycle
    @property
    def status(self) -> SkillStatus:
        if self._mode == "DONE":
            return SkillStatus.SUCCESS
        if self._mode == "IDLE":
            return SkillStatus.CANCELLED if self._plan else SkillStatus.FAILURE
        return SkillStatus.RUNNING

    @property
    def current_edge(self) -> RouteEdge | None:
        return self._current_edge

    @property
    def mode(self) -> str:
        return self._mode

    def start(self, plan: RoutePlan) -> None:
        if plan.is_empty:
            self._plan = plan
            self._mode = "DONE"
            self._notify_node(plan.goal_node, "", "")
            return
        self._plan = plan
        self._edge_index = -1
        self._advance_edge()

    def start_to_node(self, pose: Pose2D, goal_node: str) -> None:
        self.start(self._planner.plan_to_node(pose, goal_node))

    def start_to_zone(self, pose: Pose2D, zone: str) -> None:
        self.start(self._planner.plan_to_zone(pose, zone))

    def cancel(self) -> None:
        self._chassis.line_follow_stop()
        self._chassis.stop()
        self._plan = None
        self._current_edge = None
        self._mode = "IDLE"

    # ------------------------------------------------------------- control
    def update(self, pose: Pose2D) -> NavigationStatus:
        if self._plan is None or self._mode == "IDLE":
            return NavigationStatus(SkillStatus.FAILURE, message="not started")
        if self._mode == "DONE":
            return NavigationStatus(SkillStatus.SUCCESS,
                                    current_node=self._plan.goal_node)

        assert self._current_edge is not None
        edge = self._current_edge
        now = self._clock.now()
        target_node = self._graph.node(edge.to_node)
        assert target_node is not None

        if self._mode == "TURN":
            if now - self._segment_start_time > self._cfg["turn_timeout"]:
                self.cancel()
                return NavigationStatus(
                    SkillStatus.TIMEOUT, message=f"turn timeout on {edge.from_node}")
            desired_yaw = math.atan2(target_node.y - pose.y,
                                     target_node.x - pose.x)
            err = angle_diff(desired_yaw, pose.yaw)
            if abs(err) <= self._cfg["turn_tolerance"]:
                self._begin_follow(edge)
            else:
                wz = max(-self._cfg["max_wz"],
                         min(self._cfg["max_wz"], self._cfg["kp_yaw"] * err))
                self._chassis.set_velocity(0.0, 0.0, wz)
                return self._running_status()

        if self._mode == "FOLLOW":
            if now - self._segment_start_time > self._cfg["segment_timeout"]:
                self.cancel()
                return NavigationStatus(
                    SkillStatus.TIMEOUT, message=f"segment timeout {edge.from_node}"
                                                 f"->{edge.to_node}")
            if pose.distance_to(target_node.pose) <= self._cfg["arrival_radius"]:
                return self._arrive()

        return self._running_status()

    # ------------------------------------------------------------- internal
    def _advance_edge(self) -> None:
        assert self._plan is not None
        self._edge_index += 1
        if self._edge_index >= len(self._plan.edges):
            self._mode = "DONE"
            self._notify_node(self._plan.goal_node, "", "")
            return
        self._current_edge = self._plan.edges[self._edge_index]
        self._mode = "TURN"
        self._segment_start_time = self._clock.now()

    def _begin_follow(self, edge: RouteEdge) -> None:
        speed = min(edge.speed, self._cfg["default_speed"])
        self._chassis.line_follow_start(edge.edge_id, speed)
        self._mode = "FOLLOW"
        self._segment_start_time = self._clock.now()

    def _arrive(self) -> NavigationStatus:
        assert self._plan is not None and self._current_edge is not None
        edge = self._current_edge
        node = self._graph.node(edge.to_node)
        assert node is not None

        self._notify_node(node.node_id, f"{edge.from_node}->{edge.to_node}",
                          node.expected_marker)

        if node.arrival_behavior.value == "CONTINUE" and \
                self._edge_index < len(self._plan.edges) - 1:
            self._advance_edge()
            return self._running_status()

        # final node (or STOP-like behavior): stop and finish
        self._chassis.line_follow_stop()
        self._chassis.stop()
        self._mode = "DONE"
        return NavigationStatus(SkillStatus.SUCCESS, current_node=node.node_id,
                                 current_segment=f"{edge.from_node}->{edge.to_node}",
                                 expected_marker=node.expected_marker)

    def _running_status(self) -> NavigationStatus:
        assert self._current_edge is not None and self._plan is not None
        node = self._graph.node(self._current_edge.to_node)
        return NavigationStatus(
            SkillStatus.RUNNING,
            current_node=self._current_edge.from_node,
            current_segment=f"{self._current_edge.from_node}"
                             f"->{self._current_edge.to_node}",
            expected_marker=node.expected_marker if node else "",
        )

    def _notify_node(self, node_id: str, segment: str, marker: str) -> None:
        if self._on_node_event is not None:
            self._on_node_event(node_id, segment, marker)
