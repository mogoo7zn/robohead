"""Latest-frame store shared by the camera adapters.

ROS camera callbacks run asynchronously to the mission control loop.
The adapters keep the newest frame; `detect()` consumes it once (returns
its detections, then [] until the next frame arrives). This gives the
mission loop "camera fps" behaviour identical to the mock runner without
any timestamp coupling: markers are fused once per frame, and the
alignment controller keeps its last target between frames.
"""
from __future__ import annotations

import threading


class LatestFrameStore:
    """Thread-safe single-slot frame mailbox (newest frame wins)."""

    def __init__(self, max_age: float = 0.5) -> None:
        self._lock = threading.Lock()
        self._frame = None            # (ndarray, timestamp)
        self._consumed = True
        self._max_age = float(max_age)

    def push(self, image, timestamp: float) -> None:
        """Camera callback: replace whatever frame was waiting."""
        with self._lock:
            self._frame = (image, float(timestamp))
            self._consumed = False

    def take(self, now: float):
        """Control loop: return (image, timestamp) of the newest unread
        frame, or None if nothing new / the frame is stale."""
        with self._lock:
            if self._frame is None or self._consumed:
                return None
            image, stamp = self._frame
            self._consumed = True
            if now - stamp > self._max_age:
                return None            # stale frame: treat as no camera
            return image, stamp

    @property
    def last_timestamp(self) -> float:
        with self._lock:
            return self._frame[1] if self._frame else 0.0
