"""RouteGraph — the field's line network as a directed graph.

Nodes come from config (mock_routes.yaml / routes.yaml): each node has a
pose, a zone, an expected marker and an arrival behavior. Edges are
directed line segments with length and speed.

Segment IDs: each directed edge gets a stable integer id (config order),
used on the wire (CMD_LINE_FOLLOW_START.segment_id) and by the mock.
"""
from __future__ import annotations

import math
import heapq
from dataclasses import dataclass

from core.model.enums import ArrivalBehavior
from core.model.pose import Pose2D
from core.utils.config import is_filled, require


@dataclass(frozen=True)
class RouteNode:
    node_id: str
    pose: Pose2D
    zone: str = ""
    expected_marker: str = ""
    arrival_behavior: ArrivalBehavior = ArrivalBehavior.CONTINUE
    next_nodes: tuple[str, ...] = ()

    @property
    def x(self) -> float:
        return self.pose.x

    @property
    def y(self) -> float:
        return self.pose.y


@dataclass(frozen=True)
class RouteEdge:
    edge_id: int           # stable segment id for the wire protocol
    from_node: str
    to_node: str
    length: float
    speed: float

    @property
    def travel_time(self) -> float:
        """Cost for planning: time to traverse at cruise speed."""
        return self.length / max(self.speed, 1e-6)


_ZONE_ALIASES = {
    "START": "START", "ROUTE": "ROUTE", "MATERIAL": "MATERIAL",
    "BUILD": "BUILD",
}


class RouteGraphError(Exception):
    """Raised on malformed or disconnected route configs."""


class RouteGraph:
    """Immutable directed graph of line-follow segments."""

    def __init__(self, nodes: dict[str, RouteNode],
                 edges: list[RouteEdge]) -> None:
        self._nodes = nodes
        self._edges = edges
        self._out: dict[str, list[RouteEdge]] = {nid: [] for nid in nodes}
        self._in: dict[str, list[RouteEdge]] = {nid: [] for nid in nodes}
        for e in edges:
            if e.from_node not in nodes or e.to_node not in nodes:
                raise RouteGraphError(
                    f"edge {e.from_node}->{e.to_node} references unknown node")
            self._out[e.from_node].append(e)
            self._in[e.to_node].append(e)

    # ------------------------------------------------------------- factories
    @classmethod
    def from_config(cls, cfg: dict) -> "RouteGraph":
        node_entries = require(cfg, "route_nodes", "routes config")
        edge_entries = require(cfg, "route_edges", "routes config")

        nodes: dict[str, RouteNode] = {}
        for key, entry in node_entries.items():
            if not is_filled(key):
                raise RouteGraphError("route node key is TODO/empty")
            for field_name in ("x", "y", "yaw"):
                if not is_filled(require(entry, field_name, f"route_nodes.{key}")):
                    raise RouteGraphError(
                        f"route_nodes.{key}.{field_name} is TODO/empty")
            behavior = ArrivalBehavior(entry.get("arrival_behavior", "CONTINUE"))
            nodes[str(key)] = RouteNode(
                node_id=str(key),
                pose=Pose2D(float(entry["x"]), float(entry["y"]),
                            float(entry["yaw"])),
                zone=str(entry.get("zone", "")),
                expected_marker=str(entry.get("expected_marker", "")),
                arrival_behavior=behavior,
                next_nodes=tuple(str(n) for n in entry.get("next", [])),
            )
        if not nodes:
            raise RouteGraphError("route config has no nodes")

        edges: list[RouteEdge] = []
        for i, entry in enumerate(edge_entries):
            for field_name in ("from", "to", "length", "speed"):
                if not is_filled(require(entry, field_name, f"route_edges[{i}]")):
                    raise RouteGraphError(
                        f"route_edges[{i}].{field_name} is TODO/empty")
            edges.append(RouteEdge(
                edge_id=i,
                from_node=str(entry["from"]),
                to_node=str(entry["to"]),
                length=float(entry["length"]),
                speed=float(entry["speed"]),
            ))
        return cls(nodes, edges)

    # --------------------------------------------------------------- access
    def node(self, node_id: str) -> RouteNode | None:
        return self._nodes.get(node_id)

    @property
    def node_ids(self) -> list[str]:
        return list(self._nodes.keys())

    def edges_from(self, node_id: str) -> list[RouteEdge]:
        return list(self._out.get(node_id, []))

    def edges_to(self, node_id: str) -> list[RouteEdge]:
        return list(self._in.get(node_id, []))

    def edge(self, from_node: str, to_node: str) -> RouteEdge | None:
        for e in self._out.get(from_node, []):
            if e.to_node == to_node:
                return e
        return None

    def nodes_in_zone(self, zone: str) -> list[RouteNode]:
        zone = zone.upper()
        return [n for n in self._nodes.values()
                if _ZONE_ALIASES.get(n.zone.upper(), n.zone.upper()) == zone]

    def nearest_node(self, pose: Pose2D) -> str:
        """Closest node by Euclidean distance (with yaw tiebreak weight)."""
        best_id, best_cost = None, None
        for nid, n in self._nodes.items():
            cost = pose.distance_to(n.pose)
            if best_cost is None or cost < best_cost:
                best_id, best_cost = nid, cost
        assert best_id is not None
        return best_id

    # -------------------------------------------------------------- planning
    def shortest_path(self, start: str, goal: str) -> list[RouteEdge]:
        """Dijkstra over directed edges, cost = travel time."""
        if start not in self._nodes:
            raise RouteGraphError(f"unknown start node: {start}")
        if goal not in self._nodes:
            raise RouteGraphError(f"unknown goal node: {goal}")

        dist = {start: 0.0}
        prev: dict[str, RouteEdge] = {}
        heap: list[tuple[float, str]] = [(0.0, start)]
        visited: set[str] = set()

        while heap:
            d, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)
            if u == goal:
                break
            for e in self._out[u]:
                alt = d + e.travel_time
                if alt < dist.get(e.to_node, math.inf):
                    dist[e.to_node] = alt
                    prev[e.to_node] = e
                    heapq.heappush(heap, (alt, e.to_node))

        if goal not in visited and goal not in dist:
            raise RouteGraphError(f"no route from {start} to {goal}")

        path: list[RouteEdge] = []
        cur = goal
        while cur != start:
            e = prev.get(cur)
            if e is None:
                raise RouteGraphError(f"no route from {start} to {goal}")
            path.append(e)
            cur = e.from_node
        path.reverse()
        return path
