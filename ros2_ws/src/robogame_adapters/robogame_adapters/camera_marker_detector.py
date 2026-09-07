"""CameraMarkerDetector — AprilTag backend for the localization camera.

Real-hardware counterpart of MockMarkerDetector: same
`detect(timestamp) -> list[MarkerObservation]` contract, so the mission
layer (localization fusion, RECOVERY) is unchanged.

The ROS bringup node pushes frames via on_frame(); detection runs lazily
inside detect() on the mission control thread, keeping camera callbacks
cheap.

Tag-id mapping: cv2 reports integer tag ids; MarkerMap uses string ids
("marker_1", ...). The default prefix mapping tag 7 -> "marker_7" covers
the standard layout; an explicit id_map overrides it for odd fields.
"""
from __future__ import annotations

import numpy as np

from core.localization.pose_estimator import MarkerObservation
from core.utils.log import get_logger
from robogame_adapters.latest_frame import LatestFrameStore

log = get_logger("adapters.marker_detector")


def _make_detector(dictionary: str):
    """cv2.aruco detector across OpenCV versions (4.7+ / legacy)."""
    import cv2
    import cv2.aruco as aruco

    names = {
        "apriltag_36h11": "DICT_APRILTAG_36H11",
        "apriltag_16h5": "DICT_APRILTAG_16H5",
        "aruco_4x4": "DICT_4X4_50",
        "aruco_6x6": "DICT_6X6_50",
    }
    attr = names.get(dictionary)
    if attr is None:
        raise ValueError(f"unknown marker dictionary: {dictionary}")

    dict_cls = getattr(aruco, attr, None)
    if dict_cls is None:
        raise ValueError(f"cv2 build lacks dictionary {attr}")
    if hasattr(aruco, "ArucoDetector"):          # OpenCV >= 4.7
        marker_dict = (dict_cls() if callable(dict_cls)
                      else aruco.getPredefinedDictionary(dict_cls))
        params = aruco.DetectorParameters()
        return aruco.ArucoDetector(marker_dict, params)
    # legacy API (OpenCV < 4.7)
    return dict_cls() if callable(dict_cls) else \
        aruco.getPredefinedDictionary(dict_cls)


def _detect_tags(detector, image_bgr):
    """Return {tag_id: corners(4,2) TL,TR,BR,BL} on one BGR frame."""
    corners, ids, _ = detector.detectMarkers(image_bgr)
    out = {}
    if ids is None:
        return out
    for tag_id, quad in zip(ids.flatten(), corners):
        out[int(tag_id)] = np.asarray(quad, dtype=float).reshape(4, 2)
    return out


class CameraMarkerDetector:
    """AprilTag/ArUco detector satisfying the MarkerDetector protocol."""

    def __init__(self, config: dict | None = None) -> None:
        cfg = config or {}
        self._store = LatestFrameStore(
            max_age=float(cfg.get("max_age", 0.5)))
        self._id_prefix = str(cfg.get("id_prefix", "marker_"))
        self._id_map = {int(k): str(v)
                        for k, v in (cfg.get("id_map") or {}).items()}
        # same keys as config/perception.yaml -> marker_detector
        backend = str(cfg.get("backend", "apriltag"))
        if "dictionary" in cfg:                    # explicit override
            self._dictionary = str(cfg["dictionary"])
        elif backend == "aruco":
            self._dictionary = f"aruco_{cfg.get('aruco_dict', '4x4')}"
        else:                                      # backend == apriltag
            self._dictionary = f"apriltag_{cfg.get('apriltag_family', '36h11')}"
        self._detector = None                     # lazy cv2 import

    # ------------------------------------------------------------- ROS side
    def on_frame(self, image_bgr, timestamp: float) -> None:
        self._store.push(image_bgr, timestamp)

    # ---------------------------------------------------- protocol side
    def detect(self, timestamp: float) -> list[MarkerObservation]:
        frame = self._store.take(timestamp)
        if frame is None:
            return []
        image, stamp = frame
        try:
            if self._detector is None:
                self._detector = _make_detector(self._dictionary)
            tags = _detect_tags(self._detector, image)
        except Exception as exc:          # cv2 missing / bad frame
            log.warning("marker detection failed: %s", exc)
            return []

        observations = []
        for tag_id, corners in tags.items():
            marker_id = self._id_map.get(tag_id,
                                         f"{self._id_prefix}{tag_id}")
            observations.append(MarkerObservation(
                marker_id=marker_id, corners_px=corners,
                timestamp=stamp))
        if observations:
            log.debug("markers seen: %s",
                      [o.marker_id for o in observations])
        return observations
