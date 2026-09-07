"""MissionContext — everything a mission state may touch.

Built once by the runner (mock on Mac, bringup node on the Pi). States
never import hardware modules; they only see this context, so the whole
mission layer is backend-agnostic by construction.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from core.model.enums import BlockType, SkillStatus, TaskType
from core.model.pose import Pose2D
from core.state.events import (
    BlockGrabbedEvent,
    BlockPlacedEvent,
    BlockReleasedEvent,
    MissionEvent,
)
from core.state.store import WorldStateStore
from core.utils.clock import Clock
from core.utils.log import get_logger

log = get_logger("mission.states")


@dataclass
class MissionContext:
    """Service bundle injected into every HFSM state."""

    store: WorldStateStore = field(default_factory=WorldStateStore)
    clock: Clock = field(default=None)          # type: ignore[assignment]
    match: Any = None                           # MatchManager
    planner: Any = None                         # strategy Planner
    watchdog: Any = None                         # mission Watchdog
    navigator: Any = None                       # navigation Navigator
    localization: Any = None                    # localization Localization
    marker_detector: Any = None                 # perception MarkerDetector
    block_detector: Any = None                  # perception BlockDetector
    alignment: Any = None                        # skill AlignmentController
    grab_skill: Any = None                       # skill GrabSkill
    place_skill: Any = None                     # skill PlaceSkill
    chassis: Any = None                          # navigation ChassisCommander
    start_pose: Pose2D = field(default_factory=Pose2D)
    towers: dict = field(default_factory=dict)  # name -> {"x": .., "y": ..}
    max_task_retries: int = 2

    # per-task runtime
    task_block_type: BlockType = BlockType.ORANGE
    current_tower: str = "A"
    task_retries: int = 0
    current_task: TaskType | None = None

    # ------------------------------------------------------------- helpers
    def emit_mission(self, skill: str = "", state: str = "",
                     result: SkillStatus | None = None) -> None:
        self.store.apply_event(MissionEvent(
            current_task=self.current_task,
            current_skill=skill,
            hfsm_state=state,
            retry_count=self.task_retries,
            last_result=result,
        ))

    def stop_chassis(self) -> None:
        if self.chassis is not None:
            self.chassis.line_follow_stop()
            self.chassis.stop()

    def choose_tower(self) -> str:
        """Shortest tower first — simple, deterministic, upgradeable."""
        snap = self.store.snapshot()
        heights = snap.building.tower_heights
        return min(heights, key=lambda t: (heights[t], t))

    def apply_grabbed(self, block_type: BlockType) -> None:
        self.store.apply_event(BlockGrabbedEvent(block_type=block_type))

    def apply_placed(self, tower: str, block_type: BlockType,
                     stable: bool) -> None:
        self.store.apply_event(BlockPlacedEvent(
            tower=tower, block_type=block_type, stable=stable))

    def apply_released(self, block_type: BlockType) -> None:
        self.store.apply_event(BlockReleasedEvent(block_type=block_type))
