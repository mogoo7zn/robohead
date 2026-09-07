"""RoutePlanner — decides which route nodes to traverse.

Pure planning (no hardware): given the current robot pose and a goal
(zone or node), produce the list of edges to follow. Re-planned after
every task — the whole match is never fixed in advance.
"""
from __future__ import annotations

from dataclasses import dataclass

from core.model.pose import Pose2D
from core.navigation.route_graph import RouteGraph


@dataclass(frozen=True)
class RoutePlan:
    """A sequence of edges from the current position to a goal node."""

    edges: tuple
    goal_node: str

    @property
    def node_sequence(self) -> list[str]:
        if not self.edges:
            return [self.goal_node]
        seq = [self.edges[0].from_node]
        seq.extend(e.to_node for e in self.edges)
        return seq

    @property
    def total_length(self) -> float:
        return sum(e.length for e in self.edges)

    @property
    def estimated_time(self) -> float:
        return sum(e.travel_time for e in self.edges)

    @property
    def is_empty(self) -> bool:
        return not self.edges


class RoutePlanner:
    """Plans edge sequences over the RouteGraph."""

    def __init__(self, graph: RouteGraph) -> None:
        self._graph = graph

    @property
    def graph(self) -> RouteGraph:
        return self._graph

    def plan_to_node(self, current: Pose2D, goal_node: str,
                     current_node: str | None = None) -> RoutePlan:
        """Plan from a pose (or the tracked node) to a specific node."""
        graph = self._graph
        if graph.node(goal_node) is None:
            raise ValueError(f"unknown goal node: {goal_node}")

        start = current_node or graph.nearest_node(current)
        if start == goal_node:
            return RoutePlan((), goal_node)
        edges = graph.shortest_path(start, goal_node)
        return RoutePlan(tuple(edges), goal_node)

    def plan_to_zone(self, current: Pose2D, zone: str,
                     current_node: str | None = None) -> RoutePlan:
        """Plan to the nearest node of a zone (MATERIAL / BUILD / ...)."""
        graph = self._graph
        candidates = graph.nodes_in_zone(zone)
        if not candidates:
            raise ValueError(f"no route nodes in zone: {zone}")

        start = current_node or graph.nearest_node(current)
        if any(n.node_id == start for n in candidates):
            return RoutePlan((), start)

        best_plan: RoutePlan | None = None
        best_cost: float | None = None
        for n in candidates:
            try:
                plan = self.plan_to_node(current, n.node_id, current_node=start)
            except Exception:
                continue
            cost = plan.estimated_time
            if best_cost is None or cost < best_cost:
                best_plan, best_cost = plan, cost
        if best_plan is None:
            raise ValueError(f"no route into zone: {zone}")
        return best_plan
