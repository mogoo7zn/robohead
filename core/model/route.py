"""Route graph data model (nodes / edges over the black-line network)."""
from __future__ import annotations

from dataclasses import dataclass, field

from core.model.enums import ArrivalBehavior
from core.model.pose import Pose2D


@dataclass
class RouteNode:
    """A named place on the field served by the line network."""
    name: str
    position: Pose2D = field(default_factory=Pose2D)
    expected_marker: str | None = None  # marker_id visible at this node
    zone: str = ""                       # e.g. START / MATERIAL / BUILD
    possible_next_nodes: list[str] = field(default_factory=list)
    arrival_behavior: ArrivalBehavior = ArrivalBehavior.CONTINUE

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "x": self.position.x,
            "y": self.position.y,
            "yaw": self.position.yaw,
            "expected_marker": self.expected_marker,
            "zone": self.zone,
            "possible_next_nodes": list(self.possible_next_nodes),
            "arrival_behavior": self.arrival_behavior.value,
        }


@dataclass
class RouteEdge:
    """A line-followable segment between two nodes."""
    from_node: str
    to_node: str
    length: float = 0.0           # metres (informational)
    default_speed: float = 0.3     # m/s line-follow target speed
    bidirectional: bool = True

    @property
    def id(self) -> str:
        return f"{self.from_node}->{self.to_node}"
