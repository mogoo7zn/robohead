"""RouteGraph + RoutePlanner: config loading, Dijkstra, zone planning."""
import pytest

from core.model.enums import ArrivalBehavior
from core.model.pose import Pose2D
from core.navigation.route_graph import RouteGraph, RouteGraphError
from core.navigation.route_planner import RoutePlanner
from core.utils.config import load_yaml


@pytest.fixture(scope="module")
def graph() -> RouteGraph:
    return RouteGraph.from_config(load_yaml("config/mock_routes.yaml"))


@pytest.fixture(scope="module")
def planner(graph) -> RoutePlanner:
    return RoutePlanner(graph)


# ----------------------------------------------------------------- loading
def test_loads_all_nodes(graph):
    assert set(graph.node_ids) == {"start", "mid_1", "junction",
                                   "material_zone", "build_gate", "build_zone"}


def test_node_fields(graph):
    n = graph.node("material_zone")
    assert n is not None
    assert n.zone == "MATERIAL"
    assert n.expected_marker == "marker_3"
    assert n.arrival_behavior is ArrivalBehavior.ENTER_MANIPULATION
    assert n.pose.x == pytest.approx(2.9)
    assert n.pose.yaw == pytest.approx(0.0)


def test_edge_ids_stable(graph):
    e = graph.edge("start", "mid_1")
    assert e is not None
    assert e.edge_id == 0
    assert e.length == pytest.approx(1.2)
    assert e.speed == pytest.approx(0.35)
    assert e.travel_time == pytest.approx(1.2 / 0.35)


def test_bidirectional_edges(graph):
    assert graph.edge("junction", "material_zone") is not None
    assert graph.edge("material_zone", "junction") is not None


def test_unknown_node_returns_none(graph):
    assert graph.node("nowhere") is None


def test_malformed_config_rejected():
    with pytest.raises(Exception):
        RouteGraph.from_config({"route_nodes": {}})          # no edges
    with pytest.raises(Exception):
        RouteGraph.from_config({
            "route_nodes": {"a": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
            "route_edges": [{"from": "a", "to": "ghost",       # dangling
                            "length": 1.0, "speed": 0.3}],
        })


def test_todo_values_rejected():
    with pytest.raises(RouteGraphError):
        RouteGraph.from_config({
            "route_nodes": {"a": {"x": "TODO", "y": 0.0, "yaw": 0.0}},
            "route_edges": [],
        })


# ----------------------------------------------------------------- Dijkstra
def test_shortest_path_direct(graph):
    path = graph.shortest_path("start", "mid_1")
    assert len(path) == 1
    assert path[0].from_node == "start"
    assert path[0].to_node == "mid_1"


def test_shortest_path_multi_hop(graph):
    path = graph.shortest_path("start", "material_zone")
    assert [e.from_node for e in path] == ["start", "mid_1", "junction"]
    assert path[-1].to_node == "material_zone"


def test_shortest_path_reverse(graph):
    path = graph.shortest_path("material_zone", "start")
    assert path[0].from_node == "material_zone"
    assert path[-1].to_node == "start"


def test_shortest_path_prefers_faster_route(graph):
    """start -> build_zone: via junction (long) vs via mid_1 (short)."""
    path = graph.shortest_path("start", "build_zone")
    seq = [e.from_node for e in path]
    # via mid_1: 1.2/0.35 + 0.7/0.25 + 0.5/0.2 = 3.43+2.8+2.5 = 8.73 s
    # via junction: 1.2+1.0 over 0.35 = 6.29 + 0.7/0.25 + 0.5/0.2 = 11.59 s
    # but there is no edge junction->build_gate? there is: mid_1->build_gate only
    assert seq == ["start", "mid_1", "build_gate"]


def test_shortest_path_same_node_empty(graph):
    assert graph.shortest_path("start", "start") == []


def test_unknown_nodes_raise(graph):
    with pytest.raises(RouteGraphError):
        graph.shortest_path("start", "ghost")
    with pytest.raises(RouteGraphError):
        graph.shortest_path("ghost", "start")


# ----------------------------------------------------------------- planner
def test_plan_to_node(planner):
    plan = planner.plan_to_node(Pose2D(0.3, 0.5, 0.0), "material_zone")
    assert plan.goal_node == "material_zone"
    assert not plan.is_empty
    assert plan.node_sequence[0] == "start"
    assert plan.node_sequence[-1] == "material_zone"
    assert plan.total_length == pytest.approx(1.2 + 1.0 + 0.4)
    assert plan.estimated_time > 0


def test_plan_to_same_node_is_empty(planner):
    plan = planner.plan_to_node(Pose2D(2.9, 0.5, 0.0), "material_zone")
    assert plan.is_empty
    assert plan.goal_node == "material_zone"
    assert plan.node_sequence == ["material_zone"]


def test_plan_to_zone(planner):
    plan = planner.plan_to_zone(Pose2D(0.3, 0.5, 0.0), "BUILD")
    assert plan.goal_node in ("build_zone",)
    assert plan.node_sequence[-1] == "build_zone"


def test_plan_to_zone_already_there(planner):
    plan = planner.plan_to_zone(Pose2D(2.9, 0.5, 0.0), "MATERIAL")
    assert plan.is_empty


def test_plan_to_unknown_zone_raises(planner):
    with pytest.raises(ValueError):
        planner.plan_to_zone(Pose2D(0.0, 0.0, 0.0), "NOSUCHZONE")


def test_plan_uses_current_node_hint(planner):
    # pose near start, but we KNOW we're at junction: plan must start there
    plan = planner.plan_to_node(Pose2D(0.3, 0.5, 0.0), "material_zone",
                                 current_node="junction")
    assert plan.node_sequence[0] == "junction"
    assert len(plan.edges) == 1


def test_nearest_node(graph):
    assert graph.nearest_node(Pose2D(0.32, 0.5, 0.0)) == "start"
    assert graph.nearest_node(Pose2D(1.55, 0.45, 0.0)) == "mid_1"
    assert graph.nearest_node(Pose2D(2.6, 1.25, 1.5)) == "build_gate"
