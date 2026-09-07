"""Events: the only way the outside world can change WorldState.

Producers (Localization, Perception, Hardware, Mission) create these
dataclasses and hand them to WorldStateStore.apply_event(). They never
touch WorldState fields directly.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from core.model.enums import (
    BlockType,
    GripperState,
    MatchPhase,
    SkillStatus,
    TaskType,
)
from core.model.pose import Pose2D, Velocity2D


@dataclass(frozen=True)
class WorldEvent:
    """Base marker class for all events."""


# ---------------------------------------------------------------- robot / localization

@dataclass(frozen=True)
class PoseUpdatedEvent(WorldEvent):
    """Localization output: fused global pose + confidence."""
    pose: Pose2D = field(default_factory=Pose2D)
    velocity: Velocity2D = field(default_factory=Velocity2D)
    confidence: float = 0.0
    source: str = "localization"      # localization | initial | relocalize


@dataclass(frozen=True)
class MarkerSeenEvent(WorldEvent):
    marker_id: str = ""
    timestamp: float = 0.0
    consistency_ok: bool = True


# ---------------------------------------------------------------- match

@dataclass(frozen=True)
class MatchStartedEvent(WorldEvent):
    start_time: float = 0.0


@dataclass(frozen=True)
class MatchTickEvent(WorldEvent):
    elapsed_time: float = 0.0
    remaining_time: float = 0.0
    phase: MatchPhase = MatchPhase.NORMAL


@dataclass(frozen=True)
class MatchFinishedEvent(WorldEvent):
    reason: str = ""


# ---------------------------------------------------------------- inventory / building

@dataclass(frozen=True)
class FieldSupplyEvent(WorldEvent):
    """Set the estimated free blocks per colour on the field.

    Emitted at bringup from the rulebook/config; kept up to date by
    grab/release events. None leaves a colour as 'unknown'.
    """
    orange_remaining: int | None = None
    purple_remaining: int | None = None


@dataclass(frozen=True)
class BlockGrabbedEvent(WorldEvent):
    block_type: BlockType = BlockType.ORANGE


@dataclass(frozen=True)
class BlockPlacedEvent(WorldEvent):
    tower: str = "A"
    block_type: BlockType = BlockType.ORANGE
    stable: bool = True


@dataclass(frozen=True)
class BlockReleasedEvent(WorldEvent):
    """A carried block was dropped / lost (e.g. placement failure)."""
    block_type: BlockType = BlockType.ORANGE


# ---------------------------------------------------------------- route / navigation

@dataclass(frozen=True)
class RouteNodeEvent(WorldEvent):
    current_node: str = ""
    current_segment: str = ""
    expected_marker: str = ""
    line_follow_active: bool = False


# ---------------------------------------------------------------- perception

@dataclass(frozen=True)
class BlockSeenEvent(WorldEvent):
    block_type: BlockType = BlockType.ORANGE
    timestamp: float = 0.0


# ---------------------------------------------------------------- mission

@dataclass(frozen=True)
class MissionEvent(WorldEvent):
    current_task: TaskType | None = None
    current_skill: str = ""
    hfsm_state: str = ""
    retry_count: int = 0
    last_result: SkillStatus | None = None


# ---------------------------------------------------------------- hardware

@dataclass(frozen=True)
class HardwareStatusEvent(WorldEvent):
    camera_localization_ok: bool | None = None
    camera_block_ok: bool | None = None
    mcu_ok: bool | None = None
    line_sensor_ok: bool | None = None
    imu_ok: bool | None = None
    chassis_ok: bool | None = None
    manipulator_ok: bool | None = None
    emergency_stop: bool | None = None


@dataclass(frozen=True)
class ManipulatorEvent(WorldEvent):
    axis_x_position: float | None = None
    axis_z_position: float | None = None
    homed: bool | None = None
    gripper_state: GripperState | None = None
    grip_detected: bool | None = None
    fault: bool | None = None
