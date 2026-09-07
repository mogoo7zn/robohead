"""WorldStateStore — the single owner/writer of WorldState."""
from __future__ import annotations

from dataclasses import replace as dc_replace
from typing import Callable

from core.model.enums import BlockType
from core.state.events import (
    BlockGrabbedEvent,
    BlockPlacedEvent,
    BlockReleasedEvent,
    BlockSeenEvent,
    FieldSupplyEvent,
    HardwareStatusEvent,
    ManipulatorEvent,
    MarkerSeenEvent,
    MatchFinishedEvent,
    MatchStartedEvent,
    MatchTickEvent,
    MissionEvent,
    PoseUpdatedEvent,
    RouteNodeEvent,
    WorldEvent,
)
from core.state.world_state import WorldState
from core.utils.log import get_logger

log = get_logger("state.store")


class WorldStateStore:
    """Owns WorldState. Modules apply events; readers get immutable snapshots."""

    def __init__(self) -> None:
        self._state = WorldState()
        self._listeners: list[Callable[[WorldState], None]] = []

    # ------------------------------------------------------------- writer
    def apply_event(self, event: WorldEvent) -> WorldState:
        state = self._state

        if isinstance(event, PoseUpdatedEvent):
            state = state.with_(robot=dc_replace(
                state.robot,
                x=event.pose.x, y=event.pose.y, yaw=event.pose.yaw,
                vx=event.velocity.vx, vy=event.velocity.vy, wz=event.velocity.wz,
                localization_confidence=event.confidence,
            ))

        elif isinstance(event, MarkerSeenEvent):
            state = state.with_(perception=dc_replace(
                state.perception,
                last_marker=event.marker_id,
                last_marker_time=event.timestamp,
            ))

        elif isinstance(event, MatchStartedEvent):
            state = state.with_(match=dc_replace(state.match, started=True, finished=False))

        elif isinstance(event, MatchTickEvent):
            state = state.with_(match=dc_replace(
                state.match,
                elapsed_time=event.elapsed_time,
                remaining_time=event.remaining_time,
                phase=event.phase,
            ))

        elif isinstance(event, MatchFinishedEvent):
            state = state.with_(match=dc_replace(state.match, finished=True))

        elif isinstance(event, FieldSupplyEvent):
            state = state.with_(supply=dc_replace(
                state.supply,
                orange_remaining=event.orange_remaining,
                purple_remaining=event.purple_remaining,
            ))

        elif isinstance(event, BlockGrabbedEvent):
            inv = state.inventory
            if event.block_type is not None:
                orange = inv.orange_count + (1 if event.block_type.value == "ORANGE" else 0)
                purple = inv.purple_count + (1 if event.block_type.value == "PURPLE" else 0)
                state = state.with_(inventory=dc_replace(
                    inv, orange_count=orange, purple_count=purple))
                # a grabbed block leaves the free-field supply
                supply = state.supply
                if event.block_type is BlockType.ORANGE:
                    if supply.orange_remaining is not None:
                        state = state.with_(supply=dc_replace(
                            supply,
                            orange_remaining=max(0, supply.orange_remaining - 1)))
                elif event.block_type is BlockType.PURPLE:
                    if supply.purple_remaining is not None:
                        state = state.with_(supply=dc_replace(
                            supply,
                            purple_remaining=max(0, supply.purple_remaining - 1)))

        elif isinstance(event, BlockPlacedEvent):
            inv = state.inventory
            orange = inv.orange_count - (1 if event.block_type is BlockType.ORANGE else 0)
            purple = inv.purple_count - (1 if event.block_type is BlockType.PURPLE else 0)
            building = state.building
            heights = dict(building.tower_heights)
            purple_top = dict(building.tower_purple_top)
            if event.tower in heights:
                heights[event.tower] = heights[event.tower] + 1
                purple_top[event.tower] = event.block_type is BlockType.PURPLE
            state = state.with_(
                inventory=dc_replace(
                    inv,
                    orange_count=max(0, orange),
                    purple_count=max(0, purple),
                ),
                building=dc_replace(
                    building,
                    tower_heights=heights,
                    tower_purple_top=purple_top,
                    placed_total=building.placed_total + 1,
                ),
            )

        elif isinstance(event, BlockReleasedEvent):
            inv = state.inventory
            orange = inv.orange_count - (1 if event.block_type.value == "ORANGE" else 0)
            purple = inv.purple_count - (1 if event.block_type.value == "PURPLE" else 0)
            state = state.with_(inventory=dc_replace(
                inv,
                orange_count=max(0, orange),
                purple_count=max(0, purple),
            ))
            # a released block is back on the field
            supply = state.supply
            if event.block_type is BlockType.ORANGE:
                if supply.orange_remaining is not None:
                    state = state.with_(supply=dc_replace(
                        supply,
                        orange_remaining=supply.orange_remaining + 1))
            elif event.block_type is BlockType.PURPLE:
                if supply.purple_remaining is not None:
                    state = state.with_(supply=dc_replace(
                        supply,
                        purple_remaining=supply.purple_remaining + 1))

        elif isinstance(event, RouteNodeEvent):
            state = state.with_(route=dc_replace(
                state.route,
                current_node=event.current_node,
                current_segment=event.current_segment,
                expected_marker=event.expected_marker,
                line_follow_active=event.line_follow_active,
            ))

        elif isinstance(event, BlockSeenEvent):
            state = state.with_(perception=dc_replace(
                state.perception,
                last_block=event.block_type.value,
                last_block_time=event.timestamp,
            ))

        elif isinstance(event, MissionEvent):
            state = state.with_(mission=dc_replace(
                state.mission,
                current_task=event.current_task,
                current_skill=event.current_skill,
                hfsm_state=event.hfsm_state,
                retry_count=event.retry_count,
                last_result=event.last_result,
            ))

        elif isinstance(event, HardwareStatusEvent):
            hw = state.hardware
            fields = event.__dict__
            changes = {k: v for k, v in fields.items() if v is not None}
            if changes:
                state = state.with_(hardware=dc_replace(hw, **changes))

        elif isinstance(event, ManipulatorEvent):
            man = state.manipulator
            changes = {k: v for k, v in event.__dict__.items() if v is not None}
            if changes:
                state = state.with_(manipulator=dc_replace(man, **changes))

        else:
            log.warning("unhandled event type: %s", type(event).__name__)
            return self._state

        self._state = state
        for listener in self._listeners:
            listener(self._state)
        return self._state

    # ------------------------------------------------------------- reader
    def snapshot(self) -> WorldState:
        return self._state

    # ------------------------------------------------------------- observers
    def subscribe(self, listener: Callable[[WorldState], None]) -> None:
        self._listeners.append(listener)
