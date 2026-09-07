"""End-to-end mock mission: the full software stack over simulated hardware.

Runs MockMissionRunner (the same code path as `make mock`) with the FakeClock
— no wall-clock waiting — and asserts the mission-level outcomes:

  * the robot grabs all 6 field blocks (4 orange + 2 purple),
  * builds two complete 3-layer towers with purple roofs,
  * never trips the MCU watchdog / never enters RECOVERY,
  * parks in SAFE_STOP as soon as the field is exhausted.
"""
import pytest

from mock.runner import MockMissionRunner


@pytest.fixture(scope="module")
def result() -> "MockMissionResult":  # noqa: F821
    runner = MockMissionRunner()
    return runner.run(max_time=420.0)


def test_mission_reaches_terminal_state(result):
    assert result.finished
    assert result.final_state == "SAFE_STOP"


def test_all_field_blocks_placed(result):
    # 6 blocks start on the field; all must end up on towers
    assert result.placed_total == 6
    assert result.inventory == (0, 0)          # nothing left carried


def test_two_full_towers_with_purple_roofs(result):
    heights = result.tower_heights
    roofs = result.tower_purple_top
    full_towers = [t for t, h in heights.items() if h >= 3]
    assert len(full_towers) == 2                # A and B
    for t in full_towers:
        assert roofs.get(t) is True             # purple on top
    assert sum(heights.values()) == 6


def test_hardware_never_faulted(result):
    assert not result.watchdog_tripped


def test_no_recovery_in_history(result):
    assert "RECOVERY" not in result.state_history


def test_field_exhaustion_ends_mission(result):
    """The planner must switch to ENDGAME once no blocks remain — the
    mission ends well before the match clock runs out."""
    assert result.reason == "match finished"
    assert result.sim_time < 300.0
