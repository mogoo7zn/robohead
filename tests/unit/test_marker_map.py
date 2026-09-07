"""MarkerMap: config loading, validation, geometry conventions."""
import math

import numpy as np
import pytest

from core.localization.marker_map import MarkerDefinition, MarkerMap
from core.utils.config import load_yaml


@pytest.fixture
def mock_field_cfg() -> dict:
    return load_yaml("config/mock_field.yaml")


def test_from_config_loads_all_markers(mock_field_cfg):
    mmap = MarkerMap.from_config(mock_field_cfg)
    assert len(mmap) == 6
    assert "marker_1" in mmap
    assert set(mmap.ids) == {f"marker_{i}" for i in range(1, 7)}


def test_from_config_preserves_values(mock_field_cfg):
    mmap = MarkerMap.from_config(mock_field_cfg)
    m = mmap.get("marker_1")
    assert m is not None
    assert m.name == "start_area"
    assert m.x == pytest.approx(0.7)
    assert m.y == pytest.approx(0.62)
    assert m.yaw == pytest.approx(math.pi, abs=1e-4)
    assert m.z == pytest.approx(0.30)
    assert m.width == pytest.approx(0.15)


def test_unknown_id_returns_none(mock_field_cfg):
    mmap = MarkerMap.from_config(mock_field_cfg)
    assert mmap.get("marker_99") is None


def test_missing_markers_key_raises():
    with pytest.raises(Exception):
        MarkerMap.from_config({"field": {"length_x": 3.0}})


def test_todo_values_rejected():
    cfg = {"markers": [{"id": "m1", "x": "TODO", "y": 0.0, "yaw": 0.0,
                        "width": 0.15, "height": 0.15}]}
    with pytest.raises(ValueError):
        MarkerMap.from_config(cfg)


def test_duplicate_ids_rejected():
    entry = {"id": "m1", "x": 0.0, "y": 0.0, "yaw": 0.0,
             "width": 0.15, "height": 0.15}
    with pytest.raises(ValueError):
        MarkerMap.from_config({"markers": [entry, dict(entry)]})


def test_t_map_marker_face_normal_and_up():
    """Vertical marker: z axis = face normal at heading yaw, y axis = up."""
    m = MarkerDefinition(id="m", name="m", x=1.0, y=2.0, yaw=math.pi / 2,
                         width=0.2, height=0.2, z=0.3)
    t = m.t_map_marker
    # face normal -> heading yaw = +y direction
    assert np.allclose(t[:3, 2], [0.0, 1.0, 0.0])
    # image up -> map up
    assert np.allclose(t[:3, 1], [0.0, 0.0, 1.0])
    # right-handed basis
    assert np.allclose(t[:3, 0], np.cross(t[:3, 1], t[:3, 2]))
    # translation
    assert np.allclose(t[:3, 3], [1.0, 2.0, 0.3])


def test_t_map_marker_rotation_valid():
    for yaw in (0.0, 0.7, math.pi, -math.pi / 2, 3.0):
        m = MarkerDefinition(id="m", name="m", x=0.0, y=0.0, yaw=yaw,
                             width=0.1, height=0.1)
        r = m.t_map_marker[:3, :3]
        assert np.allclose(r @ r.T, np.eye(3), atol=1e-12)
        assert np.linalg.det(r) == pytest.approx(1.0)


def test_corners_order_and_size():
    m = MarkerDefinition(id="m", name="m", x=0, y=0, yaw=0,
                         width=0.2, height=0.1)
    corners = m.corners_3d
    assert corners.shape == (4, 3)
    # TL, TR, BR, BL in the marker plane z=0
    assert np.allclose(corners[0], [-0.1, 0.05, 0.0])
    assert np.allclose(corners[1], [0.1, 0.05, 0.0])
    assert np.allclose(corners[2], [0.1, -0.05, 0.0])
    assert np.allclose(corners[3], [-0.1, -0.05, 0.0])
