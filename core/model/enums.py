"""Shared enums for the whole framework. Single source of truth."""
from __future__ import annotations

from enum import Enum


class BlockType(str, Enum):
    """Building blocks carried by the robot (colour of walls / roof)."""
    ORANGE = "ORANGE"
    PURPLE = "PURPLE"


class SkillStatus(str, Enum):
    """Unified result status of every Skill / action in the system.

    All long-running operations return SkillResult with one of these
    statuses; ad-hoc booleans (is_done / finished / ok1 / ok2) are banned.
    """
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    TIMEOUT = "TIMEOUT"
    CANCELLED = "CANCELLED"


class MatchPhase(str, Enum):
    """Strategy phase driven by remaining match time (360 s total)."""
    NORMAL = "NORMAL"
    SAFE = "SAFE"
    ENDGAME = "ENDGAME"


class ArrivalBehavior(str, Enum):
    """What to do when the robot arrives at a RouteNode."""
    CONTINUE = "CONTINUE"
    STOP = "STOP"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    ENTER_ALIGNMENT = "ENTER_ALIGNMENT"
    ENTER_MANIPULATION = "ENTER_MANIPULATION"


class AlignmentState(str, Enum):
    """AlignmentController sub-states."""
    SEARCH = "SEARCH"
    TRACK = "TRACK"
    ALIGN = "ALIGN"
    ALIGNED = "ALIGNED"
    TARGET_LOST = "TARGET_LOST"
    FAILURE = "FAILURE"


class GripperState(str, Enum):
    UNKNOWN = "UNKNOWN"
    OPEN = "OPEN"
    CLOSED = "CLOSED"
    HOLDING = "HOLDING"
    FAULT = "FAULT"


class TaskType(str, Enum):
    """Planner candidates."""
    ACQUIRE_ORANGE = "ACQUIRE_ORANGE"
    ACQUIRE_PURPLE = "ACQUIRE_PURPLE"
    BUILD = "BUILD"
    ENDGAME = "ENDGAME"


class RunMode(str, Enum):
    """System run modes: mock (fully virtual), test (partial hardware),
    competition (full match). No scattered DEBUG flags."""
    MOCK = "mock"
    TEST = "test"
    COMPETITION = "competition"
