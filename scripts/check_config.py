#!/usr/bin/env python3
"""Validate every yaml config against the shapes the code expects.

Run before every match and in CI (`make check-config`). Catches the
classic failures early: missing keys, leftover TODO placeholders,
markers referenced by routes but missing from the field, unreachable
nodes, and camera sections that would break CameraParams.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.localization.camera import CameraParams
from core.localization.marker_map import MarkerMap
from core.navigation.route_graph import RouteGraph
from core.utils.config import load_yaml

CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"

ERRORS: list[str] = []
WARNINGS: list[str] = []


def error(msg: str) -> None:
    ERRORS.append(msg)


def warn(msg: str) -> None:
    WARNINGS.append(msg)


def require(d: dict, key: str, ctx: str) -> None:
    if key not in d or d[key] in (None, "", "TODO"):
        error(f"{ctx}: missing/placeholder key '{key}'")


# ---------------------------------------------------------------- field files
def check_field(name: str, is_mock: bool) -> None:
    path = CONFIG_DIR / name
    if not path.exists():
        error(f"{name}: file not found")
        return
    cfg = load_yaml(str(path))

    require(cfg, "field", name)
    for key in ("length_x", "width_y"):
        if "field" in cfg:
            require(cfg["field"], key, f"{name}/field")

    # cameras -> CameraParams must construct
    for cam in ("localization_camera", "block_camera"):
        if cam not in cfg:
            error(f"{name}: missing '{cam}' section")
            continue
        try:
            CameraParams.from_config(cfg[cam])
        except Exception as exc:
            error(f"{name}/{cam}: {exc}")

    # markers -> MarkerMap (duplicate ids, TODO ids, geometry)
    if "markers" in cfg:
        try:
            MarkerMap.from_config(cfg)
        except Exception as exc:
            error(f"{name}/markers: {exc}")

    # towers: three, distinct positions
    towers = cfg.get("towers", {})
    if len(towers) != 3:
        warn(f"{name}: expected 3 towers, found {len(towers)}")

    # zones + start pose
    require(cfg, "start_pose", name)
    zones = cfg.get("zones", {})
    for z in ("start", "material", "build"):
        if z not in zones:
            error(f"{name}/zones: missing zone '{z}'")

    if not is_mock:
        if "supply" not in cfg:
            warn(f"{name}: no 'supply' section — planner will stay "
                 "optimistic about exhausted blocks")


# ---------------------------------------------------------------- route graph
def check_routes(name: str, field_name: str) -> None:
    path = CONFIG_DIR / name
    if not path.exists():
        error(f"{name}: file not found")
        return
    cfg = load_yaml(str(path))
    try:
        graph = RouteGraph.from_config(cfg)
    except Exception as exc:
        error(f"{name}: {exc}")
        return

    nodes = graph.node_ids
    # every node must be able to reach the material zone and back
    for target in ("material_zone", "build_zone", "start"):
        if target not in nodes:
            error(f"{name}: route node '{target}' missing")
            continue
        for node in nodes:
            if node == target:
                continue
            if graph.shortest_path(node, target) is None:
                error(f"{name}: '{target}' unreachable from '{node}'")

    # expected markers must exist in the field file
    field_path = CONFIG_DIR / field_name
    if field_path.exists():
        field_cfg = load_yaml(str(field_path))
        marker_ids = {m["id"] for m in field_cfg.get("markers", [])}
        for node in cfg.get("route_nodes", {}).values():
            m = node.get("expected_marker")
            if m and m not in marker_ids:
                error(f"{name}: expected_marker '{m}' not in {field_name}")

    # edge consistency
    for edge in cfg.get("route_edges", []):
        if edge.get("from") not in cfg.get("route_nodes", {}):
            error(f"{name}: edge from unknown node '{edge.get('from')}'")
        if edge.get("to") not in cfg.get("route_nodes", {}):
            error(f"{name}: edge to unknown node '{edge.get('to')}'")


# ---------------------------------------------------------------- strategy
def check_strategy() -> None:
    name = "strategy.yaml"
    cfg = load_yaml(str(CONFIG_DIR / name))
    require(cfg, "match", name)
    match = cfg.get("match", {})
    for key in ("total_time", "normal_end_time", "safe_end_time"):
        require(match, key, f"{name}/match")
    if match.get("normal_end_time", 0) >= match.get("total_time", 1):
        error(f"{name}: normal_end_time must be < total_time")
    if match.get("safe_end_time", 0) >= match.get("total_time", 1):
        error(f"{name}: safe_end_time must be < total_time")
    require(cfg, "inventory", name)
    require(cfg, "endgame", name)
    # utility weights sane
    util = cfg.get("utility", {})
    if util.get("time_cost_weight", 0) < 0:
        error(f"{name}: negative time_cost_weight")
    if util.get("risk_cost_weight", 0) < 0:
        error(f"{name}: negative risk_cost_weight")


# ---------------------------------------------------------------- hardware
def check_hardware() -> None:
    name = "hardware.yaml"
    cfg = load_yaml(str(CONFIG_DIR / name))
    require(cfg, "mcu", name)
    mcu = cfg.get("mcu", {})
    transport = mcu.get("transport")
    if transport not in ("memory", "serial"):
        error(f"{name}/mcu: transport must be memory|serial, got {transport}")
    if transport == "serial":
        serial_cfg = mcu.get("serial", {})
        device = serial_cfg.get("device")
        if not device or device == "TODO":
            error(f"{name}/mcu/serial: device is TODO — set it before "
                  "running on the robot")
    for key in ("heartbeat_period", "heartbeat_timeout"):
        require(mcu, key, f"{name}/mcu")


# ---------------------------------------------------------------- others
def check_simple(section: str, name: str) -> None:
    cfg = load_yaml(str(CONFIG_DIR / name))
    if section not in cfg:
        error(f"{name}: missing '{section}' section")


def main() -> int:
    check_field("mock_field.yaml", is_mock=True)
    check_field("real_field.yaml", is_mock=False)
    check_routes("mock_routes.yaml", "mock_field.yaml")
    check_strategy()
    check_hardware()
    check_simple("line_follow", "navigation.yaml")
    check_simple("navigator", "navigation.yaml")
    check_simple("grab", "manipulation.yaml")
    check_simple("place", "manipulation.yaml")
    check_simple("block_detector", "perception.yaml")
    check_simple("marker_detector", "perception.yaml")
    check_simple("alignment", "perception.yaml")
    check_simple("fusion", "localization.yaml")

    for w in WARNINGS:
        print(f"WARNING: {w}")
    if ERRORS:
        for e in ERRORS:
            print(f"ERROR: {e}")
        print(f"\n{len(ERRORS)} error(s), {len(WARNINGS)} warning(s)")
        return 1
    print(f"config OK ({len(WARNINGS)} warning(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
