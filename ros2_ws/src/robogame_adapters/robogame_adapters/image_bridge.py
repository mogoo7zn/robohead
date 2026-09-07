"""sensor_msgs/Image -> numpy, without cv_bridge.

Works with any message exposing height/width/encoding/data (the ROS 2
python message is a plain object), so this imports cleanly on Mac too.
"""
from __future__ import annotations

import numpy as np


class ImageBridgeError(ValueError):
    pass


def image_to_ndarray(msg) -> np.ndarray:
    """Convert a sensor_msgs/Image (bgr8/rgb8/mono8) to a numpy array.

    BGR is the convention used by the HSV block detector (OpenCV), so
    rgb8 frames are swapped to bgr here; callers never care.
    """
    height, width = int(msg.height), int(msg.width)
    encoding = str(msg.encoding).lower()
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if encoding in ("bgr8", "rgb8"):
        img = data.reshape(height, width, 3)
        if encoding == "rgb8":
            img = img[:, :, ::-1]          # -> BGR
        return np.ascontiguousarray(img)
    if encoding in ("mono8", "8uc1"):
        return data.reshape(height, width)
    if encoding in ("bgra8", "rgba8"):
        img = data.reshape(height, width, 4)
        return np.ascontiguousarray(img[:, :, :3])   # drop alpha -> BGR-ish
    raise ImageBridgeError(f"unsupported image encoding: {msg.encoding}")
