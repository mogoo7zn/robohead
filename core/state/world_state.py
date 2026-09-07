"""WorldState — the robot's single, unified view of the world.

Single-writer principle: no module ever mutates WorldState directly.
Producers emit events; WorldStateStore.apply_event() is the only writer.
Planner / HFSM only read snapshot().
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace

from core.model.enums import (
    BlockType,
    GripperState,
    MatchPhase,
    SkillStatus,
    TaskType,
)
from core.model.pose import Pose2D


@dataclass(frozen=True)
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    localization_confidence: float = 0.0

    @property
    def pose(self) -> Pose2D:
        return Pose2D(self.x, self.y, self.yaw)


@dataclass(frozen=True)
class RouteState:
    current_node: str = ""
    current_segment: str = ""
    expected_marker: str = ""
    line_follow_active: bool = False


@dataclass(frozen=True)
class MatchState:
    elapsed_time: float = 0.0
    remaining_time: float = 0.0
    phase: MatchPhase = MatchPhase.NORMAL
    started: bool = False
    finished: bool = False


@dataclass(frozen=True)
class InventoryState:
    orange_count: int = 0
    purple_count: int = 0

    @property
    def total(self) -> int:
        return self.orange_count + self.purple_count

    def can_grab(self, block_type: BlockType) -> bool:
        """Competition constraint: total <= 3 and purple <= 1, checked
        before every grab."""
        if self.total >= 3:
            return False
        if block_type is BlockType.PURPLE and self.purple_count >= 1:
            return False
        return True


@dataclass(frozen=True)
class PerceptionState:
    last_marker: str = ""
    last_marker_time: float = 0.0
    last_block: str = ""
    last_block_time: float = 0.0


@dataclass(frozen=True)
class SupplyState:
    """Estimated free blocks left on the field, per colour.

    None = unknown (default): the planner stays optimistic and keeps
    offering acquisition. A known 0 makes the planner skip that colour
    instead of driving to an empty material zone forever.
    """
    orange_remaining: int | None = None
    purple_remaining: int | None = None


@dataclass(frozen=True)
class MissionState:
    current_task: TaskType | None = None
    current_skill: str = ""
    hfsm_state: str = ""
    retry_count: int = 0
    last_result: SkillStatus | None = None


@dataclass(frozen=True)
class HardwareState:
    camera_localization_ok: bool = False
    camera_block_ok: bool = False
    mcu_ok: bool = False
    line_sensor_ok: bool = False
    imu_ok: bool = False
    chassis_ok: bool = False
    manipulator_ok: bool = False
    emergency_stop: bool = False

    def critical_ok(self) -> bool:
        return (self.mcu_ok and self.chassis_ok and not self.emergency_stop)


@dataclass(frozen=True)
class ManipulatorWorldState:
    axis_x_position: float = 0.0
    axis_z_position: float = 0.0
    homed: bool = False
    gripper_state: GripperState = GripperState.UNKNOWN
    grip_detected: bool = False
    fault: bool = False


@dataclass(frozen=True)
class BuildingState:
    """Simple 3-tower bookkeeping: heights in blocks + purple-on-top flags."""
    tower_heights: dict = field(default_factory=lambda: {"A": 0, "B": 0, "C": 0})
    tower_purple_top: dict = field(default_factory=lambda: {"A": False, "B": False, "C": False})
    placed_total: int = 0

    def as_dict(self) -> dict:
        return {
            "heights": dict(self.tower_heights),
            "purple_top": dict(self.tower_purple_top),
            "placed_total": self.placed_total,
        }


@dataclass(frozen=True)
class WorldState:
    robot: RobotState = field(default_factory=RobotState)
    route: RouteState = field(default_factory=RouteState)
    match: MatchState = field(default_factory=MatchState)
    inventory: InventoryState = field(default_factory=InventoryState)
    supply: SupplyState = field(default_factory=SupplyState)
    perception: PerceptionState = field(default_factory=PerceptionState)
    mission: MissionState = field(default_factory=MissionState)
    hardware: HardwareState = field(default_factory=HardwareState)
    manipulator: ManipulatorWorldState = field(default_factory=ManipulatorWorldState)
    building: BuildingState = field(default_factory=BuildingState)

    def with_(self, **sections) -> "WorldState":
        """Immutable copy with some sections replaced (store internal use)."""
        return replace(self, **sections)
