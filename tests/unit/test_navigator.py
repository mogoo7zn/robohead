"""Navigator: segment execution, turning, arrival, timeout, node events —
including a closed-loop run over the simulated line network."""
import math

import pytest

from core.mock.sim_world import LineSegment, SimWorld
from core.model.enums import SkillStatus
from core.model.pose import Pose2D
from core.navigation.chassis import MockChassis
from core.navigation.navigator import Navigator
from core.navigation.route_graph import RouteGraph
from core.navigation.route_planner import RoutePlanner
from core.utils.clock import FakeClock
from core.utils.config import load_yaml


@pytest.fixture(scope="module")
def graph() -> RouteGraph:
    return RouteGraph.from_config(load_yaml("config/mock_routes.yaml"))


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock(0.0)


@pytest.fixture
def world() -> SimWorld:
    ws = SimWorld()
    ws.robot_pose = Pose2D(0.3, 0.5, 0.0)
    return ws


@pytest.fixture
def chassis(world) -> MockChassis:
    return MockChassis(world)


@pytest.fixture
def events() -> list:
    return []


@pytest.fixture
def navigator(graph, chassis, clock, events) -> Navigator:
    return Navigator(graph, chassis, clock=clock,
                     on_node_event=lambda node, seg, marker:
                     events.append((node, seg, marker)))


def install_line_network(world: SimWorld, graph: RouteGraph) -> None:
    """Give SimWorld physical line segments for every graph edge."""
    world.line_segments = []
    for node_id in graph.node_ids:
        a = graph.node(node_id)
        for e in graph.edges_from(node_id):
            b = graph.node(e.to_node)
            world.line_segments.append(LineSegment(a.x, a.y, b.x, b.y))


# ------------------------------------------------------------- basic states
def test_not_started_fails(navigator):
    out = navigator.update(Pose2D(0.3, 0.5, 0.0))
    assert out.status is SkillStatus.FAILURE
    assert out.message == "not started"


def test_empty_plan_succeeds_immediately(navigator, graph, chassis, events):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(2.9, 0.5, 0.0),
                                            "material_zone")
    navigator.start(plan)
    out = navigator.update(Pose2D(2.9, 0.5, 0.0))
    assert out.status is SkillStatus.SUCCESS
    assert out.current_node == "material_zone"
    assert events == [("material_zone", "", "")]


def test_turn_before_follow(navigator, graph, chassis):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0), "mid_1")
    navigator.start(plan)
    assert navigator.mode == "TURN"

    out = navigator.update(Pose2D(0.3, 0.5, math.pi))    # facing backwards
    assert out.status is SkillStatus.RUNNING
    # chassis got a rotation command, not line follow
    assert chassis.commands[-1][0] == "velocity"
    assert chassis.commands[-1][3] != 0.0                  # wz nonzero


def test_turn_transitions_to_follow(navigator, graph, chassis):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0), "mid_1")
    navigator.start(plan)
    out = navigator.update(Pose2D(0.3, 0.5, 0.0))          # already facing
    assert navigator.mode == "FOLLOW"
    assert chassis.commands[-1][0] == "line_follow_start"
    assert out.current_segment == "start->mid_1"


def test_follow_speed_from_edge(graph, chassis, clock, events):
    navigator = Navigator(graph, chassis, clock=clock)
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0), "mid_1")
    navigator.start(plan)
    navigator.update(Pose2D(0.3, 0.5, 0.0))
    cmd = [c for c in chassis.commands if c[0] == "line_follow_start"][-1]
    assert cmd[2] == pytest.approx(0.35)                    # edge speed


def test_arrival_success(navigator, graph, chassis):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0), "mid_1")
    navigator.start(plan)
    navigator.update(Pose2D(0.3, 0.5, 0.0))                 # TURN -> FOLLOW
    out = navigator.update(Pose2D(1.45, 0.5, 0.0))          # within 0.12 m
    assert out.status is SkillStatus.SUCCESS
    assert out.current_node == "mid_1"
    assert chassis.commands[-1] == ("stop",)
    assert navigator.mode == "DONE"


def test_multi_segment_runs_through(navigator, graph, chassis, events):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0),
                                           "material_zone")
    navigator.start(plan)
    # drive the whole route by teleporting the pose along node positions
    poses = [Pose2D(0.3, 0.5, 0.0), Pose2D(1.5, 0.5, 0.0),
             Pose2D(2.5, 0.5, 0.0), Pose2D(2.9, 0.5, 0.0)]
    last = None
    for pose in poses:
        for _ in range(50):
            last = navigator.update(pose)
            if navigator.mode != "TURN":
                break
            pose = Pose2D(pose.x, pose.y, pose.yaw + 0.2)
    assert last is not None
    assert last.status is SkillStatus.SUCCESS
    assert last.current_node == "material_zone"
    node_ids = [n for n, _, _ in events]
    assert node_ids == ["mid_1", "junction", "material_zone"]


def test_node_events_carry_marker(navigator, graph, events):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0), "mid_1")
    navigator.start(plan)
    navigator.update(Pose2D(0.3, 0.5, 0.0))
    navigator.update(Pose2D(1.5, 0.5, 0.0))
    assert events == [("mid_1", "start->mid_1", "marker_6")]


def test_segment_timeout(navigator, graph, chassis, clock):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0), "mid_1")
    navigator.start(plan)
    navigator.update(Pose2D(0.3, 0.5, 0.0))                 # -> FOLLOW
    clock.advance(200.0)                                    # > 120 s timeout
    out = navigator.update(Pose2D(0.4, 0.5, 0.0))
    assert out.status is SkillStatus.TIMEOUT
    assert "segment timeout" in out.message


def test_turn_timeout(navigator, graph, chassis, clock):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0), "mid_1")
    navigator.start(plan)
    clock.advance(20.0)                                    # > turn_timeout 10
    out = navigator.update(Pose2D(0.3, 0.5, math.pi))
    assert out.status is SkillStatus.TIMEOUT
    assert "turn timeout" in out.message


def test_cancel_stops_chassis(navigator, graph, chassis):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0), "mid_1")
    navigator.start(plan)
    navigator.update(Pose2D(0.3, 0.5, 0.0))
    navigator.cancel()
    assert ("line_follow_stop",) in chassis.commands
    assert ("stop",) in chassis.commands
    assert navigator.mode == "IDLE"


def test_expected_marker_in_status(navigator, graph):
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0),
                                           "material_zone")
    navigator.start(plan)
    out = navigator.update(Pose2D(0.3, 0.5, 0.0))
    assert out.current_segment == "start->mid_1"


# ------------------------------------------------------------- closed loop
def test_closed_loop_line_following(graph, world, chassis, clock):
    """Robot physically follows the line network to the material zone."""
    install_line_network(world, graph)
    navigator = Navigator(graph, chassis, clock=clock)
    plan = RoutePlanner(graph).plan_to_node(Pose2D(0.3, 0.5, 0.0),
                                           "material_zone")
    navigator.start(plan)

    dt = 0.02
    final = None
    for _ in range(int(120.0 / dt)):                        # 120 s budget
        clock.advance(dt)
        final = navigator.update(world.robot_pose)
        if final.status is not SkillStatus.RUNNING:
            break
        world.step(dt, clock)

    assert final is not None
    assert final.status is SkillStatus.SUCCESS, final
    assert final.current_node == "material_zone"
    # robot physically arrived within the arrival radius of the node
    node = graph.node("material_zone")
    assert world.robot_pose.distance_to(node.pose) <= 0.15
    # and it actually moved the full distance
    assert world.robot_pose.x > 2.5
    assert not world.line_follow_active                    # chassis stopped
