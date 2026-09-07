"""SE(3)/SE(2) geometry for marker-based localization.

Frame conventions:
  map / base_link : x forward, y left, z up (REP-103 style)
  camera (optical) : x right, y down, z forward
  marker frame     : plane z=0, x right, y up, z out of the marker face

All 3D results are finally projected down to the planar Pose2D(x, y, yaw)
that the rest of the system consumes.
"""
from __future__ import annotations

import math

import numpy as np

from core.model.pose import Pose2D, normalize_angle


# ---------------------------------------------------------------- SE(3) utils
def se3(x: float = 0.0, y: float = 0.0, z: float = 0.0,
        roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> np.ndarray:
    """4x4 from translation + fixed-axis RPY (R = Rz(yaw) Ry(pitch) Rx(roll))."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def se3_from_pose2d(pose: Pose2D, z: float = 0.0) -> np.ndarray:
    return se3(pose.x, pose.y, z, 0.0, 0.0, pose.yaw)


def se3_compose(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return a @ b


def se3_inverse(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def pose2d_from_se3(T: np.ndarray) -> Pose2D:
    """Project a planar 3D transform down to (x, y, yaw)."""
    x, y = float(T[0, 3]), float(T[1, 3])
    yaw = math.atan2(float(T[1, 0]), float(T[0, 0]))
    return Pose2D(x, y, normalize_angle(yaw))


def rotation_yaw(R: np.ndarray) -> float:
    return normalize_angle(math.atan2(float(R[1, 0]), float(R[0, 0])))


# ---------------------------------------------------------------- projection
def camera_matrix(fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    return np.array([[fx, 0.0, cx],
                      [0.0, fy, cy],
                      [0.0, 0.0, 1.0]])


def project_points(K: np.ndarray, T_cam_obj: np.ndarray,
                    points_obj: np.ndarray) -> np.ndarray:
    """Project Nx3 object points through T_cam_obj and K -> Nx2 pixels."""
    R = T_cam_obj[:3, :3]
    t = T_cam_obj[:3, 3]
    pts_cam = (R @ points_obj.T).T + t
    if np.any(pts_cam[:, 2] <= 1e-6):
        return np.zeros((0, 2))  # behind camera -> not visible
    z = pts_cam[:, 2:3]
    uv = (K @ (pts_cam / z).T).T
    return uv[:, :2]


def marker_corners_3d(width: float, height: float) -> np.ndarray:
    """Corners in the marker frame, order TL, TR, BR, BL (y up)."""
    w, h = width / 2.0, height / 2.0
    return np.array([
        [-w, h, 0.0],    # top-left
        [w, h, 0.0],     # top-right
        [w, -h, 0.0],    # bottom-right
        [-w, -h, 0.0],   # bottom-left
    ])


# ---------------------------------------------------------------- PnP solving
def solve_pnp_homography(K: np.ndarray, object_points: np.ndarray,
                         image_points: np.ndarray) -> np.ndarray | None:
    """Planar pose from 4+ coplanar correspondences — pure numpy.

    Returns T_cam_obj or None if degenerate. Robust enough for mock/absent
    cv2; the cv2 backend below is preferred on the real robot.
    """
    if len(object_points) < 4 or len(image_points) < 4:
        return None

    H = _homography_dlt(object_points[:, :2], image_points)
    if H is None:
        return None

    Hn = np.linalg.inv(K) @ H
    h1, h2, h3 = Hn[:, 0], Hn[:, 1], Hn[:, 2]
    scale = 2.0 / (np.linalg.norm(h1) + np.linalg.norm(h2))
    r1, r2, t = h1 * scale, h2 * scale, h3 * scale
    r3 = np.cross(r1, r2)
    # homography scale sign: marker must be in front of the camera (z > 0)
    if t[2] < 0:
        r1, r2, r3, t = -r1, -r2, -r3, -t
    R = np.column_stack([r1, r2, r3])
    # project to the closest rotation matrix (handle non-orthogonality)
    U, _, Vt = np.linalg.svd(R)
    R = U @ np.diag([1.0, 1.0, np.linalg.det(U @ Vt)]) @ Vt
    if np.linalg.det(R) < 0:
        return None

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _homography_dlt(src: np.ndarray, dst: np.ndarray) -> np.ndarray | None:
    """4-point (or more) DLT homography."""
    A = []
    for (x, y), (u, v) in zip(src, dst):
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y, -u])
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y, -v])
    A = np.asarray(A, dtype=float)
    try:
        _, _, Vt = np.linalg.svd(A)
    except np.linalg.LinAlgError:
        return None
    H = Vt[-1].reshape(3, 3)
    if abs(H[2, 2]) < 1e-12:
        return None
    return H / H[2, 2]


def solve_pnp_cv2(K: np.ndarray, dist: np.ndarray,
                  object_points: np.ndarray,
                  image_points: np.ndarray) -> np.ndarray | None:
    """cv2.solvePnP backend (preferred when OpenCV is installed)."""
    try:
        import cv2
    except ImportError:
        return None
    ok, rvec, tvec = cv2.solvePnP(
        object_points.astype(np.float64), image_points.astype(np.float64),
        K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok or not np.all(np.isfinite(tvec)):
        ok, rvec, tvec = cv2.solvePnP(
            object_points.astype(np.float64), image_points.astype(np.float64),
            K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.all(np.isfinite(tvec)):
        return None
    R, _ = cv2.Rodrigues(rvec)
    if not np.all(np.isfinite(R)):
        return None
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()
    return T


def solve_pnp(K: np.ndarray, dist: np.ndarray,
              object_points: np.ndarray, image_points: np.ndarray) -> np.ndarray | None:
    """Best available backend: cv2 first, numpy homography fallback."""
    T = solve_pnp_cv2(K, dist, object_points, image_points)
    if T is not None:
        return T
    return solve_pnp_homography(K, object_points, image_points)


def solve_pnp_candidates(K: np.ndarray, dist: np.ndarray,
                         object_points: np.ndarray,
                         image_points: np.ndarray) -> list[np.ndarray]:
    """All plausible T_cam_obj solutions, best-first.

    Planar markers seen near-frontally have a two-fold ambiguity (the
    'planar flip'): two very different poses with almost identical
    reprojection error. Callers should disambiguate with an independent
    prior (e.g. the odometry pose) — see PoseEstimator.
    """
    candidates = _solve_pnp_candidates_cv2(K, dist, object_points, image_points)
    if candidates:
        return candidates
    T = solve_pnp_homography(K, object_points, image_points)
    return [T] if T is not None else []


def _solve_pnp_candidates_cv2(K: np.ndarray, dist: np.ndarray,
                             object_points: np.ndarray,
                             image_points: np.ndarray) -> list[np.ndarray]:
    try:
        import cv2
    except ImportError:
        return []
    try:
        _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            object_points.astype(np.float64), image_points.astype(np.float64),
            K, dist, flags=cv2.SOLVEPNP_IPPE)
    except Exception:
        return []
    out: list[np.ndarray] = []
    for rvec, tvec in zip(rvecs, tvecs):
        tvec = np.asarray(tvec).ravel()
        if not np.all(np.isfinite(tvec)):
            continue
        R, _ = cv2.Rodrigues(rvec)
        if not np.all(np.isfinite(R)):
            continue
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec
        out.append(T)
    return out
