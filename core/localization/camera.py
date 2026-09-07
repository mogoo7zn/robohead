"""CameraParams — intrinsics + mounting extrinsics of the localization camera.

Frame convention (see geometry.py):
  base_link: x forward, y left, z up
  camera (optical): x right, y down, z forward

`t_base_cam` is the pose of the optical frame in base_link, i.e. it
transforms camera-frame points into base-frame points.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from core.localization.geometry import camera_matrix, se3
from core.utils.config import is_filled, require

# Default mounting: camera 15 cm ahead of base origin, 30 cm high, looking
# forward. roll/yaw map the optical frame onto REP-103: optical z (forward)
# -> base +x, optical y (down) -> base -z, optical x (right) -> base -y.
# `pitch` is a downward tilt about base +y (positive = looks down).
DEFAULT_T_BASE_CAM = se3(0.15, 0.0, 0.30,
                         roll=-math.pi / 2, pitch=0.0, yaw=-math.pi / 2)


def build_t_base_cam(x: float, y: float, z: float,
                     roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Mounting transform for a camera with a downward tilt.

    R = Ry(pitch) @ Rz(yaw) @ Rx(roll):
      * (roll, yaw) orient the optical frame (see DEFAULT_T_BASE_CAM),
      * pitch > 0 tilts the whole camera down about the base y axis —
        a plain RPY composition would rotate about the optical axis
        (an image roll) instead of tilting the view.
    """
    mount = se3(0.0, 0.0, 0.0, roll=roll, pitch=0.0, yaw=yaw)
    tilt = se3(0.0, 0.0, 0.0, roll=0.0, pitch=pitch, yaw=0.0)
    t = tilt @ mount
    t[0, 3], t[1, 3], t[2, 3] = x, y, z
    return t


@dataclass(frozen=True)
class CameraParams:
    fx: float
    fy: float
    cx: float
    cy: float
    t_base_cam: np.ndarray = field(default_factory=lambda: DEFAULT_T_BASE_CAM.copy())
    distortion: np.ndarray = field(default_factory=lambda: np.zeros(5))
    width: int = 1280
    height: int = 720

    @property
    def k(self) -> np.ndarray:
        return camera_matrix(self.fx, self.fy, self.cx, self.cy)

    @classmethod
    def from_config(cls, cfg: dict) -> "CameraParams":
        """Build from a localization_camera.yaml-style dict.

        Raises if any required intrinsic is still TODO — the competition
        robot must ship calibrated values.
        """
        intrinsic = require(cfg, "intrinsic", "camera config")
        for key in ("fx", "fy", "cx", "cy"):
            if not is_filled(require(intrinsic, key, "intrinsic")):
                raise ValueError(f"intrinsic.{key} is TODO/empty — calibrate first")
        dist = intrinsic.get("distortion") or [0.0] * 5

        extrinsic = cfg.get("extrinsic") or {}
        t_base_cam = build_t_base_cam(
            float(extrinsic.get("x", 0.15)),
            float(extrinsic.get("y", 0.0)),
            float(extrinsic.get("z", 0.30)),
            float(extrinsic.get("roll", -math.pi / 2)),
            float(extrinsic.get("pitch", 0.0)),
            float(extrinsic.get("yaw", -math.pi / 2)),
        )

        device = cfg.get("device") or {}
        resolution = device.get("resolution") or [1280, 720]

        return cls(
            fx=float(intrinsic["fx"]),
            fy=float(intrinsic["fy"]),
            cx=float(intrinsic["cx"]),
            cy=float(intrinsic["cy"]),
            t_base_cam=t_base_cam,
            distortion=np.asarray(dist, dtype=float),
            width=int(resolution[0]),
            height=int(resolution[1]),
        )
