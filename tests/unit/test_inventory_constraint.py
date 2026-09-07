"""Competition constraints: inventory limits and tower bookkeeping."""
import pytest

from core.model.enums import BlockType
from core.state.events import (
    BlockGrabbedEvent,
    BlockPlacedEvent,
    BlockReleasedEvent,
)
from core.state.store import WorldStateStore


def test_grab_increments_counts():
    store = WorldStateStore()
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.PURPLE))
    snap = store.snapshot()
    assert snap.inventory.orange_count == 2
    assert snap.inventory.purple_count == 1
    assert snap.inventory.total == 3


def test_can_grab_enforces_total_limit():
    store = WorldStateStore()
    for _ in range(3):
        store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    inv = store.snapshot().inventory
    assert inv.can_grab(BlockType.ORANGE) is False
    assert inv.can_grab(BlockType.PURPLE) is False


def test_can_grab_enforces_purple_limit():
    store = WorldStateStore()
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.PURPLE))
    inv = store.snapshot().inventory
    assert inv.can_grab(BlockType.PURPLE) is False   # purple <= 1
    assert inv.can_grab(BlockType.ORANGE) is True    # still room (2/3)


def test_place_decrements_and_builds_tower():
    store = WorldStateStore()
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    store.apply_event(BlockPlacedEvent(tower="A", block_type=BlockType.ORANGE))
    snap = store.snapshot()
    assert snap.inventory.orange_count == 1
    assert snap.building.tower_heights["A"] == 1
    assert snap.building.placed_total == 1
    assert snap.building.tower_purple_top["A"] is False


def test_place_purple_marks_roof():
    store = WorldStateStore()
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.PURPLE))
    store.apply_event(BlockPlacedEvent(tower="B", block_type=BlockType.PURPLE))
    snap = store.snapshot()
    assert snap.building.tower_heights["B"] == 1
    assert snap.building.tower_purple_top["B"] is True
    assert snap.inventory.purple_count == 0


def test_release_drops_carried_block():
    store = WorldStateStore()
    store.apply_event(BlockGrabbedEvent(block_type=BlockType.ORANGE))
    store.apply_event(BlockReleasedEvent(block_type=BlockType.ORANGE))
    assert store.snapshot().inventory.orange_count == 0
