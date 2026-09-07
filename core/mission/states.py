"""Match-level HFSM states (top machine + task sub-machine).

Top machine (design doc §4):
  BOOT -> SELF_CHECK -> WAIT_START -> INITIAL_LOCALIZE -> MISSION
  MISSION -> RECOVERY -> MISSION; MISSION/RECOVERY -> SAFE_STOP
  MISSION -> ENDGAME -> SAFE_STOP -> DONE

Task sub-machine (design doc §5/§6/§7):
  PLAN -> ACQUIRE (NAV -> ALIGN -> GRAB) | BUILD (NAV -> PLACE) -> PLAN
  failures -> TASK_FAILED (retry, then ENDGAME)
"""
from __future__ import annotations

from core.model.enums import BlockType, SkillStatus, TaskType
from core.model.pose import Pose2D
from core.mission.context import MissionContext
from core.mission.hfsm import DONE, CompositeState, Machine, State
from core.mission.watchdog import WatchdogVerdict
from core.utils.log import get_logger

log = get_logger("mission.states")


# ================================================================= top level
class BootState(State):
    name = "BOOT"
    timeout = 10.0

    def on_entry(self) -> None:
        log.info("BOOT: robogame mission starting")
        self.ctx.stop_chassis()

    def update(self) -> str | None:
        return "SELF_CHECK"

    def on_timeout(self) -> str:
        return "SAFE_STOP"


class SelfCheckState(State):
    name = "SELF_CHECK"
    timeout = 15.0

    def on_entry(self) -> None:
        log.info("SELF_CHECK: hardware health check")

    def update(self) -> str | None:
        verdict = (self.ctx.watchdog.tick()
                   if self.ctx.watchdog is not None else WatchdogVerdict.OK)
        if verdict is WatchdogVerdict.SAFE_STOP:
            return "SAFE_STOP"
        return "WAIT_START"

    def on_timeout(self) -> str:
        return "SAFE_STOP"


class WaitStartState(State):
    name = "WAIT_START"
    timeout = None

    def update(self) -> str | None:
        if self.ctx.match.started:
            log.info("WAIT_START: start signal received")
            return "INITIAL_LOCALIZE"
        return None


class InitialLocalizeState(State):
    name = "INITIAL_LOCALIZE"
    timeout = 10.0

    def on_entry(self) -> None:
        log.info("INITIAL_LOCALIZE: resetting pose to start position")
        odom = Pose2D(self.ctx.start_pose.x, self.ctx.start_pose.y,
                     self.ctx.start_pose.yaw)
        if self.ctx.localization is not None:
            self.ctx.localization.reset(self.ctx.start_pose, odom)

    def update(self) -> str | None:
        loc = self.ctx.localization
        if loc is None:
            return "MISSION"
        now = self.ctx.clock.now()
        if self.ctx.marker_detector is not None:
            loc.update_markers(self.ctx.marker_detector.detect(now))
        if loc.confidence >= 0.5 or loc.markers_fresh:
            log.info("INITIAL_LOCALIZE: fixed, confidence %.2f",
                     loc.confidence)
            return "MISSION"
        return None

    def on_timeout(self) -> str:
        log.warning("INITIAL_LOCALIZE: no marker fix — proceeding on odometry")
        return "MISSION"


class MissionComposite(CompositeState):
    """MISSION: runs the task sub-machine until it reports DONE.

    Health monitoring runs here (design doc §4: MISSION -> RECOVERY ->
    MISSION; MISSION/RECOVERY -> SAFE_STOP): the watchdog verdict gates
    every task tick, so hardware faults are caught mid-task, not just in
    SELF_CHECK.
    """

    name = "MISSION"
    timeout = None

    def update(self) -> str | None:
        verdict = (self.ctx.watchdog.tick()
                   if self.ctx.watchdog is not None else WatchdogVerdict.OK)
        if verdict is WatchdogVerdict.SAFE_STOP:
            log.error("MISSION: watchdog SAFE_STOP -> halting")
            return "SAFE_STOP"
        if verdict is WatchdogVerdict.RECOVER:
            log.warning("MISSION: watchdog RECOVER -> RECOVERY")
            return "RECOVERY"
        result = self.submachine.tick()
        if result == DONE:
            return self.on_submachine_done()
        return None

    def on_submachine_done(self) -> str | None:
        return "ENDGAME"


class RecoveryState(State):
    name = "RECOVERY"
    timeout = 20.0

    def on_entry(self) -> None:
        log.warning("RECOVERY: stopping, re-localizing")
        self.ctx.stop_chassis()
        self.ctx.task_retries = 0

    def update(self) -> str | None:
        loc = self.ctx.localization
        now = self.ctx.clock.now()
        if loc is not None and self.ctx.marker_detector is not None:
            loc.update_markers(self.ctx.marker_detector.detect(now))
            if loc.confidence >= 0.5:
                log.info("RECOVERY: recovered, confidence %.2f", loc.confidence)
                return "MISSION"
        return None

    def on_timeout(self) -> str:
        return "SAFE_STOP"


class EndgameState(State):
    name = "ENDGAME"
    timeout = 10.0

    def on_entry(self) -> None:
        log.info("ENDGAME: parking safely")
        self.ctx.stop_chassis()

    def update(self) -> str | None:
        return "SAFE_STOP"


class SafeStopState(State):
    name = "SAFE_STOP"
    timeout = None

    def on_entry(self) -> None:
        log.warning("SAFE_STOP: all halt")
        self.ctx.stop_chassis()
        if self.ctx.match is not None and self.ctx.match.started \
                and not self.ctx.match.finished:
            self.ctx.match.finish(reason="safe stop")
        self.ctx.emit_mission(state="SAFE_STOP")

    def update(self) -> str | None:
        return DONE


# ================================================================= task level
class PlanState(State):
    name = "PLAN"
    timeout = 5.0

    def on_entry(self) -> None:
        decision = self.ctx.planner.decide(self.ctx.store.snapshot())
        # NOTE: retries are NOT reset here — they only reset on task
        # success, so a failing task exhausts its budget even if the
        # planner briefly alternates between candidates.
        self.ctx.current_task = decision.task
        log.info("PLAN: %s (%.2f) — %s", decision.task.value,
                 decision.utility, decision.reason)
        self.ctx.emit_mission(skill="PLAN", state="PLAN")

    def update(self) -> str | None:
        task = self.ctx.current_task
        if task is TaskType.ACQUIRE_ORANGE:
            self.ctx.task_block_type = BlockType.ORANGE
            return "ACQUIRE_NAV"
        if task is TaskType.ACQUIRE_PURPLE:
            self.ctx.task_block_type = BlockType.PURPLE
            return "ACQUIRE_NAV"
        if task is TaskType.BUILD:
            return "BUILD_NAV"
        # ENDGAME: bank any carried blocks first (they score once placed),
        # then shut the mission down.
        if self.ctx.store.snapshot().inventory.total > 0:
            log.info("PLAN: ENDGAME with cargo — placing carried blocks")
            return "BUILD_NAV"
        return DONE        # -> mission composite -> top ENDGAME


class AcquireNavState(State):
    name = "ACQUIRE_NAV"
    timeout = 120.0

    def on_entry(self) -> None:
        log.info("ACQUIRE_NAV: navigating to MATERIAL zone")
        self.ctx.navigator.start_to_zone(self._pose(), "MATERIAL")
        self.ctx.emit_mission(skill="NAV_TO_MATERIAL", state="ACQUIRE_NAV")

    def update(self) -> str | None:
        status = self.ctx.navigator.update(self._pose())
        if status.status is SkillStatus.RUNNING:
            return None
        if status.status is SkillStatus.SUCCESS:
            return "ACQUIRE_ALIGN"
        log.warning("ACQUIRE_NAV failed: %s (%s)", status.status.value,
                    status.message)
        return "TASK_FAILED"

    def on_timeout(self) -> str:
        self.ctx.navigator.cancel()
        return "TASK_FAILED"

    def _pose(self) -> Pose2D:
        return self.ctx.localization.pose if self.ctx.localization \
            else self.ctx.store.snapshot().robot.pose


class AcquireAlignState(State):
    name = "ACQUIRE_ALIGN"
    timeout = 30.0

    def on_entry(self) -> None:
        log.info("ACQUIRE_ALIGN: visual servoing onto %s block",
                 self.ctx.task_block_type)
        self.ctx.alignment.start(self.ctx.task_block_type)
        self.ctx.emit_mission(skill="ALIGN_BLOCK", state="ACQUIRE_ALIGN")

    def update(self) -> str | None:
        now = self.ctx.clock.now()
        detections = self.ctx.block_detector.detect(now)
        out = self.ctx.alignment.update(detections)
        if out.status is SkillStatus.RUNNING:
            self.ctx.chassis.set_velocity(out.velocity.vx, out.velocity.vy,
                                          out.velocity.wz)
            return None
        if out.status is SkillStatus.SUCCESS:
            self.ctx.chassis.stop()
            return "ACQUIRE_GRAB"
        self.ctx.chassis.stop()
        log.warning("ACQUIRE_ALIGN failed: %s", out.state.value)
        return "TASK_FAILED"

    def on_exit(self) -> None:
        self.ctx.chassis.stop()

    def on_timeout(self) -> str:
        self.ctx.chassis.stop()
        return "TASK_FAILED"


class AcquireGrabState(State):
    name = "ACQUIRE_GRAB"
    timeout = 60.0

    def on_entry(self) -> None:
        log.info("ACQUIRE_GRAB: executing grab sequence")
        self.ctx.grab_skill.start()
        self.ctx.emit_mission(skill="GRAB_BLOCK", state="ACQUIRE_GRAB")

    def update(self) -> str | None:
        result = self.ctx.grab_skill.update()
        if result.status is SkillStatus.RUNNING:
            return None
        if result.status is SkillStatus.SUCCESS:
            self.ctx.apply_grabbed(self.ctx.task_block_type)
            self.ctx.task_retries = 0       # task succeeded — fresh budget
            log.info("ACQUIRE_GRAB: verified, inventory updated")
            return "PLAN"
        log.warning("ACQUIRE_GRAB failed: %s", result.message)
        return "TASK_FAILED"

    def on_timeout(self) -> str:
        return "TASK_FAILED"


class BuildNavState(State):
    name = "BUILD_NAV"
    timeout = 120.0

    def on_entry(self) -> None:
        log.info("BUILD_NAV: navigating to BUILD zone")
        self.ctx.navigator.start_to_zone(self._pose(), "BUILD")
        self.ctx.emit_mission(skill="NAV_TO_BUILD", state="BUILD_NAV")

    def update(self) -> str | None:
        status = self.ctx.navigator.update(self._pose())
        if status.status is SkillStatus.RUNNING:
            return None
        if status.status is SkillStatus.SUCCESS:
            return "BUILD_PLACE"
        log.warning("BUILD_NAV failed: %s (%s)", status.status.value,
                    status.message)
        return "TASK_FAILED"

    def on_timeout(self) -> str:
        self.ctx.navigator.cancel()
        return "TASK_FAILED"

    def _pose(self) -> Pose2D:
        return self.ctx.localization.pose if self.ctx.localization \
            else self.ctx.store.snapshot().robot.pose


class BuildPlaceState(State):
    name = "BUILD_PLACE"
    timeout = 90.0
    _continue_load: bool = False   # class default; instance flag set on re-entry

    def on_entry(self) -> None:
        snap = self.ctx.store.snapshot()
        if self._continue_load:
            # Multi-block load: keep stacking on the same tower so the
            # purple block lands as the roof, not on a neighbour tower.
            tower = self.ctx.current_tower
            self._continue_load = False
        else:
            tower = self.ctx.choose_tower()
        self.ctx.current_tower = tower
        layer = snap.building.tower_heights[tower]
        held = snap.inventory
        block_type = (BlockType.ORANGE if held.orange_count > 0
                      else BlockType.PURPLE)
        log.info("BUILD_PLACE: tower %s layer %d (%s)", tower, layer,
                 block_type)
        self.ctx.place_skill.start(layer=layer)
        self.ctx.emit_mission(skill="PLACE_BLOCK", state="BUILD_PLACE")

    def update(self) -> str | None:
        result = self.ctx.place_skill.update()
        if result.status is SkillStatus.RUNNING:
            return None
        snap = self.ctx.store.snapshot()
        held = snap.inventory
        # orange first, purple last: the roof block must top the tower
        block_type = (BlockType.ORANGE if held.orange_count > 0
                      else BlockType.PURPLE)
        if result.status is SkillStatus.SUCCESS:
            self.ctx.apply_placed(self.ctx.current_tower, block_type,
                                  stable=result.stable)
            log.info("BUILD_PLACE: placed on %s (stable=%s)",
                     self.ctx.current_tower, result.stable)
            if snap.inventory.total > 1:
                self._continue_load = True   # same tower, next layer
                return "BUILD_PLACE"         # re-enter: place the next block
            self.ctx.task_retries = 0       # task succeeded — fresh budget
            return "PLAN"
        log.warning("BUILD_PLACE failed: %s", result.message)
        self.ctx.apply_released(block_type)
        return "TASK_FAILED"

    def on_timeout(self) -> str:
        return "TASK_FAILED"


class TaskFailedState(State):
    name = "TASK_FAILED"
    timeout = 2.0

    def on_entry(self) -> None:
        self.ctx.task_retries += 1
        log.warning("TASK_FAILED: retry %d/%d", self.ctx.task_retries,
                    self.ctx.max_task_retries)
        self.ctx.stop_chassis()
        self.ctx.emit_mission(skill="RETRY", state="TASK_FAILED")

    def update(self) -> str | None:
        if self.ctx.task_retries > self.ctx.max_task_retries:
            log.error("TASK_FAILED: retries exhausted -> ENDGAME")
            return DONE
        return "PLAN"

    def on_timeout(self) -> str:
        return "PLAN"


# ================================================================= builders
def build_match_machine(ctx: MissionContext) -> Machine:
    """Top-level competition lifecycle machine."""
    m = Machine("MATCH", ctx, ctx.clock, initial="BOOT")
    m.add_state(BootState(m))
    m.add_state(SelfCheckState(m))
    m.add_state(WaitStartState(m))
    m.add_state(InitialLocalizeState(m))
    m.add_state(MissionComposite(m, build_task_machine(ctx)))
    m.add_state(RecoveryState(m))
    m.add_state(EndgameState(m))
    m.add_state(SafeStopState(m))
    return m


def build_task_machine(ctx: MissionContext) -> Machine:
    """MISSION sub-machine: the normal task loop."""
    m = Machine("TASKS", ctx, ctx.clock, initial="PLAN")
    m.add_state(PlanState(m))
    m.add_state(AcquireNavState(m))
    m.add_state(AcquireAlignState(m))
    m.add_state(AcquireGrabState(m))
    m.add_state(BuildNavState(m))
    m.add_state(BuildPlaceState(m))
    m.add_state(TaskFailedState(m))
    return m
