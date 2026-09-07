"""Block (building cube) observation model from the block camera."""
from __future__ import annotations

from dataclasses import dataclass

from core.model.enums import BlockType


@dataclass
class BlockObservation:
    """A detected block in the block camera image.

    Note: this is *relative* image-space perception used for the final
    tens-of-centimetres alignment. It is NOT a global map position.
    """
    block_type: BlockType
    center_u: float
    center_v: float
    area: float
    angle: float  # radians, orientation of the block in image
    confidence: float = 1.0
    timestamp: float = 0.0

    @property
    def is_valid(self) -> bool:
        return self.area > 0.0 and self.confidence > 0.0
