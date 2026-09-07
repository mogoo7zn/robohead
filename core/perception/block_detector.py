"""BlockDetector — interface + mock and HSV backends for the block camera.

The block camera is dedicated to close-range work: detecting orange /
purple blocks and feeding the AlignmentController with pixel targets.
It is NEVER mixed with the localization camera (separate device, topic
and config file).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from core.localization.camera import CameraParams
from core.localization.geometry import se3_compose, se3_from_pose2d, se3_inverse
from core.model.enums import BlockType
from core.model.pose import Pose2D, angle_diff
from core.utils.log import get_logger

log = get_logger("perception.block_detector")


@dataclass(frozen=True)
class BlockDetection:
    """One block seen by the block camera."""

    block_type: BlockType
    u: float            # centroid pixel x
    v: float            # centroid pixel y
    area: float         # px^2
    angle: float        # rad, image-plane orientation (0 = square to camera)
    timestamp: float = 0.0
    distance: float = 0.0   # m, camera-to-block (mock/estimated)


class BlockDetector(Protocol):
    """Anything that yields block detections for one camera frame."""

    def detect(self, timestamp: float) -> list[BlockDetection]:
        ...


class MockBlockDetector:
    """Renders blocks from the simulated world through the block camera.

    Reads SimWorld blocks (ground truth) and the robot pose, projects
    each free block into the block camera image. Visible = in front of
    the camera, inside the image, within `max_distance`.
    """

    def __init__(self, camera: CameraParams,
                 blocks_provider=None,
                 pose_provider=None,
                 block_size: float = 0.07,
                 max_distance: float = 1.2,
                 pixel_noise: float = 0.0,
                 area_noise: float = 0.0,
                 seed: int | None = None) -> None:
        self._camera = camera
        self._blocks_provider = blocks_provider or (lambda: [])
        self._pose_provider = pose_provider or (lambda: Pose2D())
        self._block_size = block_size
        self._max_distance = max_distance
        self._pixel_noise = float(pixel_noise)
        self._area_noise = float(area_noise)
        self._rng = np.random.default_rng(seed)

    def bind(self, blocks_provider, pose_provider) -> None:
        """Attach SimWorld-backed callables (blocks list, robot pose)."""
        self._blocks_provider = blocks_provider
        self._pose_provider = pose_provider

    def detect(self, timestamp: float) -> list[BlockDetection]:
        robot_pose = self._pose_provider()
        t_map_base = se3_from_pose2d(robot_pose)
        t_cam_base = se3_inverse(self._camera.t_base_cam)
        t_cam_map = se3_compose(t_cam_base, se3_inverse(t_map_base))

        half = self._block_size / 2.0
        w, h = self._camera.width, self._camera.height
        detections: list[BlockDetection] = []
        for block in self._blocks_provider():
            if getattr(block, "grabbed", False) or getattr(block, "placed", False):
                continue
            # block center in map frame (z = half height above floor)
            p_map = np.array([block.x, block.y, half, 1.0])
            p_cam = t_cam_map @ p_map
            x, y, z = p_cam[0], p_cam[1], p_cam[2]
            if z <= 1e-6:
                continue
            distance = float(np.linalg.norm(p_cam[:3]))
            if distance > self._max_distance:
                continue
            u = self._camera.fx * x / z + self._camera.cx
            v = self._camera.fy * y / z + self._camera.cy
            margin = 2.0 * half  # allow partial visibility at the edges
            if not (-margin <= u <= w - 1 + margin and -margin <= v <= h - 1 + margin):
                continue

            # apparent square edge length in pixels
            edge_px = self._camera.fx * self._block_size / distance
            area = float(edge_px * edge_px)
            # block orientation in the robot frame (CCW positive): how the
            # block appears rotated in the image. The alignment controller
            # applies wz = +kp * angle to rotate the robot onto the block.
            angle = angle_diff(getattr(block, "yaw", 0.0), robot_pose.yaw)

            if self._pixel_noise > 0:
                u += float(self._rng.normal(0.0, self._pixel_noise))
                v += float(self._rng.normal(0.0, self._pixel_noise))
            if self._area_noise > 0:
                area *= float(self._rng.normal(1.0, self._area_noise))

            detections.append(BlockDetection(
                block_type=BlockType(block.block_type if isinstance(block.block_type, str)
                                     else block.block_type.value),
                u=float(u), v=float(v), area=area, angle=angle,
                timestamp=timestamp, distance=distance,
            ))
        return detections


class HsvBlockDetector:
    """Real backend: HSV thresholding on BGR frames from the block camera.

    On the Pi the ROS adapter pushes frames into detect_in_image(); the
    HSV ranges and area filters come from config/perception.yaml.
    """

    def __init__(self, config: dict, camera: CameraParams | None = None) -> None:
        self._config = config
        self._camera = camera
        self._last: list[BlockDetection] = []
        self._last_timestamp = 0.0

    def detect_in_image(self, image_bgr: np.ndarray, timestamp: float = 0.0
                        ) -> list[BlockDetection]:
        try:
            import cv2
        except ImportError:
            log.warning("cv2 unavailable — HsvBlockDetector needs OpenCV")
            return []

        cfg = self._config
        min_area = float(cfg.get("min_area", 300))
        max_area = float(cfg.get("max_area", 200000))
        max_blocks = int(cfg.get("max_blocks", 4))
        kernel = int(cfg.get("morphology_kernel", 5))

        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        out: list[BlockDetection] = []
        for block_type in (BlockType.ORANGE, BlockType.PURPLE):
            rng = cfg.get(block_type.value.lower())
            if not rng:
                continue
            lower = np.asarray(rng["lower"], dtype=np.uint8)
            upper = np.asarray(rng["upper"], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            if kernel > 1:
                k = np.ones((kernel, kernel), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            candidates = []
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if not (min_area <= area <= max_area):
                    continue
                (_, _), (bw, bh), rect_angle = cv2.minAreaRect(contour)
                m = cv2.moments(contour)
                if m["m00"] <= 0:
                    continue
                u = m["m10"] / m["m00"]
                v = m["m01"] / m["m00"]
                # normalize rectangle angle to rad, square blocks ~0/90 deg
                a = float(np.deg2rad(rect_angle))
                if bw < bh:  # minAreaRect angle convention
                    a += np.pi / 2.0
                candidates.append(BlockDetection(
                    block_type=block_type, u=u, v=v, area=area,
                    angle=a, timestamp=timestamp))
            candidates.sort(key=lambda d: d.area, reverse=True)
            out.extend(candidates[:max_blocks])

        self._last = out
        self._last_timestamp = timestamp
        return out

    def detect(self, timestamp: float) -> list[BlockDetection]:
        """Protocol-compatible read of the most recent frame result."""
        if timestamp != self._last_timestamp:
            return []
        return self._last
