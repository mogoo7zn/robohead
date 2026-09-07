"""MarkerMap — the field's known visual markers and their poses.

Loaded from config (mock_field.yaml / field.yaml). This is the single
source of truth for the localization subsystem: the PoseEstimator maps
marker detections to robot poses through these definitions.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from core.localization.geometry import marker_corners_3d, se3
from core.utils.config import is_filled, require


@dataclass(frozen=True)
class MarkerDefinition:
    """One vertical field marker.

    Pose convention (map frame, z up):
      x, y   : marker center on the floor plane
      yaw    : heading of the marker's face normal (out of the face)
      z      : center height above the floor
    Marker image-up maps to map-up, image-right is the face normal rotated
    -90 deg — i.e. a robot looking at the face head-on sees an upright tag.
    """

    id: str
    name: str
    x: float
    y: float
    yaw: float
    width: float
    height: float
    z: float = 0.0

    @property
    def corners_3d(self) -> np.ndarray:
        """Corners in the marker frame (TL, TR, BR, BL)."""
        return marker_corners_3d(self.width, self.height)

    @property
    def t_map_marker(self) -> np.ndarray:
        """SE(3) pose of the marker center in the map frame.

        Equivalent to se3(x, y, z, roll=+pi/2, pitch=0, yaw=yaw+pi/2):
        marker z (face normal) -> heading `yaw`, marker y (image up) -> +z.
        """
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        t = se3(self.x, self.y, self.z,
                roll=math.pi / 2, pitch=0.0, yaw=self.yaw + math.pi / 2)
        # se3 RPY above already yields the basis below; recompute explicitly
        # to guard against RPY convention drift in geometry.py.
        t[:3, :3] = np.array([
            [-s, 0.0, c],
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
        ])
        return t


class MarkerMap:
    """Immutable set of field markers, addressable by id."""

    def __init__(self, markers: dict[str, MarkerDefinition]) -> None:
        self._markers = dict(markers)

    # ------------------------------------------------------------ factories
    @classmethod
    def from_config(cls, cfg: dict) -> "MarkerMap":
        """Build from the `markers:` list of a field config dict."""
        entries = require(cfg, "markers", "field config")
        if not isinstance(entries, list) or not entries:
            raise ValueError("field config 'markers' must be a non-empty list")
        markers: dict[str, MarkerDefinition] = {}
        for i, entry in enumerate(entries):
            ctx = f"markers[{i}]"
            marker_id = require(entry, "id", ctx)
            if not is_filled(marker_id):
                raise ValueError(f"{ctx}: marker id is TODO/empty")
            for key in ("x", "y", "yaw", "width", "height"):
                if not is_filled(require(entry, key, ctx)):
                    raise ValueError(f"{ctx}.{key} is TODO/empty")
            marker = MarkerDefinition(
                id=str(marker_id),
                name=str(entry.get("name", marker_id)),
                x=float(entry["x"]),
                y=float(entry["y"]),
                yaw=float(entry["yaw"]),
                width=float(entry["width"]),
                height=float(entry["height"]),
                z=float(entry.get("z", 0.0)),
            )
            if marker.id in markers:
                raise ValueError(f"duplicate marker id: {marker.id}")
            markers[marker.id] = marker
        return cls(markers)

    # -------------------------------------------------------------- access
    def get(self, marker_id: str) -> MarkerDefinition | None:
        return self._markers.get(marker_id)

    def __contains__(self, marker_id: str) -> bool:
        return marker_id in self._markers

    def __len__(self) -> int:
        return len(self._markers)

    @property
    def ids(self) -> list[str]:
        return list(self._markers.keys())

    def all(self) -> list[MarkerDefinition]:
        return list(self._markers.values())
