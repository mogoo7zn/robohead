"""Manipulator (cross-slide + gripper) data model."""
from __future__ import annotations

from dataclasses import dataclass

from core.model.enums import GripperState


class AxisId:
    """Cross-slide logical axes. If the real mechanics differ, adapt in
    config / STM32 mapping — never in Mission logic."""
    X = "X"
    Z = "Z"


@dataclass
class ManipulatorState:
    x_position_mm: float = 0.0
    z_position_mm: float = 0.0
    homed: bool = False
    gripper_state: GripperState = GripperState.UNKNOWN
    grip_detected: bool = False
    moving: bool = False
    fault: bool = False
    fault_code: int = 0

    @property
    def ready(self) -> bool:
        return self.homed and not self.fault
