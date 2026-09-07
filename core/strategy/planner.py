"""Strategy Planner — Rule + Utility score (first version by design).

    utility = expected_score - time_cost * weight - risk_cost * weight

Candidates: ACQUIRE_ORANGE, ACQUIRE_PURPLE, BUILD, ENDGAME.
All parameters come from config/strategy.yaml.
"""
from __future__ import annotations

from dataclasses import dataclass

from core.model.enums import MatchPhase, TaskType
from core.state.world_state import WorldState
from core.utils.log import get_logger

log = get_logger("strategy.planner")


@dataclass
class TaskDecision:
    task: TaskType
    utility: float
    reason: str


class Planner:
    """Rule-based utility planner over a WorldState snapshot."""

    def __init__(self, config: dict) -> None:
        self._cfg = config
        util = config.get("utility", {})
        self._score = util.get("expected_score", {})
        self._time_weight = util.get("time_cost_weight", 0.05)
        self._risk_weight = util.get("risk_cost_weight", 0.4)
        est = config.get("task_estimates", {})
        self._estimates = est
        risk = config.get("risk", {})
        self._risk = risk
        inv_cfg = config.get("inventory", {})
        self._max_total = inv_cfg.get("max_total", 3)
        self._max_purple = inv_cfg.get("max_purple", 1)
        # purple is only scoreable as the roof of a full tower: grab it last
        self._purple_as_roof = bool(inv_cfg.get("purple_as_roof", False))
        endgame = config.get("endgame", {})
        self._min_remaining = endgame.get("min_remaining_for_new_task", 45.0)

    # -------------------------------------------------------------- scoring
    def _utility(self, expected_score: float, duration: float, risk: float) -> float:
        return expected_score - duration * self._time_weight - risk * self._risk_weight

    def decide(self, world: WorldState) -> TaskDecision:
        remaining = world.match.remaining_time
        phase = world.match.phase
        inv = world.inventory

        # --- hard rules -------------------------------------------------
        # ENDGAME: never start a task that cannot finish in time left
        if remaining < self._min_remaining:
            return TaskDecision(TaskType.ENDGAME, 0.0,
                                f"remaining {remaining:.0f}s too short for a new task")

        # Cannot carry more blocks
        if inv.total >= self._max_total:
            return TaskDecision(TaskType.BUILD, self._utility(
                self._score.get("build_layer", 25.0),
                self._estimates.get("build", 55.0),
                self._risk.get("build", 0.15)),
                "inventory full -> build")

        # If carrying purple, prefer placing it (it can only be on top)
        if inv.purple_count >= self._max_purple:
            return TaskDecision(TaskType.BUILD, self._utility(
                self._score.get("build_layer", 25.0),
                self._estimates.get("build", 55.0),
                self._risk.get("build", 0.15)),
                "carrying purple -> build it on top")

        # --- utility comparison ----------------------------------------
        candidates = []
        supply = world.supply

        # Field supply: skip acquisition of a colour known to be exhausted
        # (None = unknown -> stay optimistic and keep offering it).
        if supply.orange_remaining != 0:
            candidates.append(TaskDecision(
                TaskType.ACQUIRE_ORANGE,
                self._utility(self._score.get("orange_grab", 10.0),
                              self._estimates.get("acquire_orange", 60.0),
                              self._risk.get("acquire_orange", 0.10)),
                "grab orange block"))

        # Purple only if we can still carry it — and, with purple_as_roof,
        # only when it would complete a full load (purple scores as the roof)
        can_purple = (inv.purple_count < self._max_purple
                      and inv.total < self._max_total
                      and supply.purple_remaining != 0)
        if can_purple and self._purple_as_roof:
            can_purple = inv.total >= self._max_total - 1
        if can_purple:
            candidates.append(TaskDecision(
                TaskType.ACQUIRE_PURPLE,
                self._utility(self._score.get("purple_grab", 14.0),
                              self._estimates.get("acquire_purple", 70.0),
                              self._risk.get("acquire_purple", 0.25)),
                "grab purple roof block"))

        # Building is attractive when carrying blocks — unless we play the
        # purple-roof strategy: then we top the load up to max_total first
        # (orange, orange, purple) so the roof block actually roofs a tower.
        if inv.total > 0 and not (self._purple_as_roof
                                  and inv.total < self._max_total):
            build_util = self._utility(
                self._score.get("build_layer", 25.0) * inv.total,
                self._estimates.get("build", 55.0),
                self._risk.get("build", 0.15))
            candidates.append(TaskDecision(TaskType.BUILD, build_util,
                                          f"place {inv.total} carried block(s)"))

        # SAFE phase: reduce risk — prefer building over acquiring
        if phase is MatchPhase.SAFE and inv.total == 0:
            # still allow acquisition but with higher risk penalty
            for c in candidates:
                if c.task is not TaskType.BUILD:
                    c.utility -= self._risk_weight * 5.0

        # Field awareness: an acquisition that keeps failing (block type
        # exhausted on the field) loses utility so the planner switches
        # to what is actually available instead of retrying forever.
        failed_task = world.mission.current_task
        if failed_task is not None and world.mission.retry_count > 0:
            penalty = 8.0 * world.mission.retry_count
            for c in candidates:
                if c.task is failed_task:
                    c.utility -= penalty
                    c.reason += f" (retry {world.mission.retry_count})"

        # Nothing acquirable and nothing carried -> match is over for us
        if not candidates:
            return TaskDecision(TaskType.ENDGAME, 0.0,
                                "no blocks left on the field")

        best = max(candidates, key=lambda c: c.utility)
        log.debug("planner decision: %s (%.2f) — %s",
                  best.task.value, best.utility, best.reason)
        return best
