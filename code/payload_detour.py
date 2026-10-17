"""Interruptible payload detour driven by measured field walls.

The detour no longer uses a fused pose, a map origin, a global line or a
``Pose2D`` endpoint. It locks one *same physical wall* (the road wall the leg is
patrolling), records the drive-axle-centre perpendicular distance ``d0`` to it,
first advances 5 cm along the patrol heading using a front/rear wall, and then
performs the side moves and turns against the road wall's measured normal
distance and measured body bearing:

  standard      patrol forward 0.05 m -> left90 -> forward 0.43 m -> release
                -> reverse to d0 -> right90
  right_mount   patrol forward 0.05 m -> left90 -> forward 0.32 m -> left90 -> release -> right90
                -> reverse to d0 -> right90

The side-drop sequences follow ``docs/06_PAYLOAD_AND_ALLOCATION.md``; the
5 cm patrol advance is the operator's added offset. Nothing here reads a
fused pose, and no move ever substitutes an elapsed time or an
integrated command for a measured distance.
"""

from dataclasses import dataclass
import math
import sys
import time

from components.payload_servos import open_payload
from components.wall_navigation import (
    LEFT_BEARING_RAD,
    MoveNormalToWall,
    REAR_BEARING_RAD,
    RIGHT_BEARING_RAD,
    WallActionLimits,
    can_follow_wall,
    can_move_normal,
    drop_turn,
)
from components.wall_observer import WallLine, WallTracker
from mission_control import step_wall_runtime as _step
from robocup_runtime import RobocupMissionState

PERIOD_S = 0.05
# Retained for the retired fused call sites; the wall path uses WallActionLimits.
DEFAULT_APPROACH_M = 0.43
PRE_TURN_FORWARD_M = 0.05


@dataclass(frozen=True)
class DetourSettings:
    payload_slot: int = 4  # Slot usage comes from build_payload_plan(); 4 is yellow.
    approach_m: float = 0.43
    release_hold_s: float = 0.5
    position_tolerance_m: float = 0.01
    patrol_slow_speed_m_s: float = 0.08
    payload_slots: tuple[int, ...] = ()
    behavior: str = "standard"

    @property
    def release_slots(self):
        return self.payload_slots or (self.payload_slot,)

    def __post_init__(self):
        if (isinstance(self.payload_slot, bool) or not isinstance(self.payload_slot, int)
                or self.payload_slot not in range(5)):
            raise ValueError('payload cargo slot must be 0..4')
        if (any(isinstance(slot, bool) or not isinstance(slot, int) or slot not in range(5)
                for slot in self.release_slots) or len(set(self.release_slots)) != len(self.release_slots)):
            raise ValueError('payload cargo slots must be unique integers in 0..4')
        if self.behavior not in {"standard", "right_mount"}:
            raise ValueError("unknown payload detour behavior")
        if self.behavior == "right_mount" and self.release_slots != (4,):
            raise ValueError("right_mount is reserved for slot 4 only")
        if self.behavior == "standard" and 4 in self.release_slots:
            raise ValueError("slot 4 requires right_mount without other slots")
        for name in ("approach_m", "release_hold_s", "position_tolerance_m",
                     "patrol_slow_speed_m_s"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class WallDropReference:
    """The one reference wall a detour works against."""

    reference_id: str
    d0_base_m: float
    bearing_rad: float
    normal_xy: tuple[float, float]


def run_payload_detour(runtime, settings, *, guard=lambda: None,
                       check_vision=lambda: None, clock=time.monotonic, sleep=time.sleep,
                       tracker=None, acquire_reference=None, limits=None,
                       simulate_release=False, on_release=None, **legacy):
    """Wall-referenced detour. ``acquire_reference`` locks the tracked wall.

    ``acquire_reference(runtime, tracker)`` must return a
    :class:`WallDropReference` for the wall the leg is currently patrolling, or
    ``None`` when it cannot be seen. It is called once, before the first turn,
    and then never again: ``d0`` is frozen for the whole detour so the return
    distance cannot chase the current measurement.

    ``legacy`` swallows the retired ``travel_drive`` / ``fine_align`` arguments
    so an old call site fails loudly on behaviour instead of silently keeping a
    fused feedback source.
    """
    if legacy:
        raise TypeError("fused detour arguments are retired: %s" % ", ".join(sorted(legacy)))
    if tracker is None:
        raise ValueError("payload detour requires the tracked wall object")
    if acquire_reference is None:
        raise ValueError("payload detour requires a wall reference source")
    if limits is None:
        limits = getattr(runtime, "limits", None) or WallActionLimits.from_config(runtime.config)
    release_error = None
    commanded: list[int] = []

    def check():
        guard()
        check_vision()

    def current_wall() -> WallLine | None:
        return tracker.wall

    def run_action(action, stage):
        check()
        runtime.action = action
        runtime.record_event("payload_detour_stage_start", stage=stage, action=action.name,
                             behavior=settings.behavior, feedback_source="wall_scan")
        try:
            while True:
                check()
                # Use the tick's own report. Reading ``action.outcome`` after the
                # tick would conflate the outcome this tick produced with one
                # left over from an earlier call, and a single transient
                # "waiting for a fresh wall" result would look like a failure.
                step = _step(runtime)
                if step.action_state == "done":
                    runtime.action = None
                    outcome = action.outcome
                    runtime.record_event("payload_detour_stage_done", stage=stage,
                                         action=action.name,
                                         reason=None if outcome is None else outcome.reason,
                                         behavior=settings.behavior)
                    return True
                if step.action_state == "error":
                    runtime.action = None
                    raise RuntimeError(f"{stage}: {action.name} failed: {step.error}")
                if action.finished and action.outcome is not None and not action.outcome.done:
                    runtime.action = None
                    raise RuntimeError(f"{stage}: {action.name} failed: {action.outcome.reason}")
                sleep(PERIOD_S)
        except BaseException:
            runtime.action = None
            raise

    def hold(duration):
        deadline = clock() + duration
        runtime.record_event("payload_release_hold_start", duration_s=duration,
                             behavior=settings.behavior)
        while clock() < deadline:
            check()
            # The stopped hold keeps the drive fed and observes guard/STOP; it
            # needs no pose and no wall observation.
            _step(runtime)
            sleep(min(PERIOD_S, max(0.0, deadline - clock())))
        check()
        runtime.record_event("payload_release_hold_done", behavior=settings.behavior)

    try:
        check()
        runtime.drive.stop()
        if runtime.mission.state is RobocupMissionState.TARGET_OPERATION:
            runtime.mission.on_payload_action_done()
        reference = acquire_reference(runtime, tracker)
        if reference is None:
            runtime.record_event("payload_detour_skipped", reason="road reference unavailable")
            return None
        d0 = reference.d0_base_m
        runtime.record_event("payload_detour_reference", reference_id=reference.reference_id,
                             d0_base_m=d0, bearing_rad=reference.bearing_rad,
                             behavior=settings.behavior, feedback_source="wall_scan")
        runtime.mission.on_payload_action_start()

        # A side wall alone cannot measure travel along the patrol direction.
        # Lock a separate front/rear wall for this short move; never replace
        # the road tracker or estimate these 5 cm from commanded speed/time.
        _step(runtime)
        snapshot = runtime.observer.latest()
        if snapshot is None or not snapshot.is_fresh(float(clock())):
            raise RuntimeError("pre-turn advance requires a fresh wall scan")
        candidates = [wall for wall in snapshot.lines
                      if abs(wall.normal_xy[0]) >= math.cos(math.radians(30))
                      and abs(sum(a * b for a, b in zip(wall.normal_xy, reference.normal_xy)))
                      <= math.sin(math.radians(20))]
        if not candidates:
            raise RuntimeError("pre-turn advance requires a visible front or rear wall")
        forward_wall = max(candidates, key=lambda wall:
                           (wall.normal_xy[0] > 0, wall.span_m, -wall.rms_m))
        forward_tracker = WallTracker()
        forward_tracker.set_reference(forward_wall)
        forward_target = forward_wall.rho_base_m - PRE_TURN_FORWARD_M * forward_wall.normal_xy[0]
        runtime.record_event("payload_pre_turn_advance", distance_m=PRE_TURN_FORWARD_M,
                             reference_bearing_rad=forward_wall.bearing_rad,
                             target_base_m=forward_target, feedback_source="wall_scan")
        advance = MoveNormalToWall(target_distance_m=forward_target, direction=1,
                                  reference_bearing_rad=forward_wall.bearing_rad,
                                  desired_bearing_rad=forward_wall.bearing_rad,
                                  limits=limits, tracker=forward_tracker, clock=clock,
                                  terminal_range_m=0.015, tolerance_m=0.003)
        run_action(advance, "forward_on_patrol_5cm")
        snapshot = runtime.observer.latest()
        road = tracker.track(snapshot.lines, expected_bearing_rad=reference.bearing_rad,
                             expected_distance_m=tracker.last_rho_m)
        if road is None:
            raise RuntimeError("road wall lost after the pre-turn advance")

        left = drop_turn(road.bearing_rad, REAR_BEARING_RAD, limits=limits,
                         tracker=tracker, turn_direction=1, clock=clock,
                         handoff=lambda wall: can_move_normal(wall, REAR_BEARING_RAD, limits))
        if not run_action(left, "left_90deg"):
            raise RuntimeError("left_90deg did not complete")

        approach_target = d0 + settings.approach_m
        approach = MoveNormalToWall(target_distance_m=approach_target, direction=1,
                                    reference_bearing_rad=tracker.wall.bearing_rad,
                                    desired_bearing_rad=REAR_BEARING_RAD,
                                    limits=limits, tracker=tracker, clock=clock)
        if not run_action(approach, "forward_to_drop"):
            raise RuntimeError("forward_to_drop did not complete")

        if settings.behavior == "right_mount":
            second_left = drop_turn(tracker.wall.bearing_rad, LEFT_BEARING_RAD,
                                    limits=limits, tracker=tracker, turn_direction=1,
                                    clock=clock)
            if not run_action(second_left, "right_mount_left_90deg"):
                raise RuntimeError("right_mount_left_90deg did not complete")

        for slot in settings.release_slots:
            check()
            if simulate_release:
                runtime.record_event("payload_release_skipped", actuator="payload_servo",
                                     slot=slot, cargo_count=1, behavior=settings.behavior,
                                     reason="explicit payload simulation; physical output disabled")
            else:
                runtime.record_event("payload_release_start", actuator="payload_servo",
                                     slot=slot, cargo_count=1, behavior=settings.behavior)
                try:
                    released = open_payload(slot)
                except Exception as exc:
                    runtime.record_event("payload_release_failed", slot=slot, reason=str(exc), continuing=True)
                    continue
                if released is False:
                    runtime.record_event("payload_release_failed", slot=slot,
                                         reason="PWM command returned False", continuing=True)
                    continue
                commanded.append(slot)
                # Book the ledger *now*, before the hold and before the turn back
                # to the road: the slot is open from this instant, so a later
                # failure must not put it back into pending and let the same
                # colour be serviced twice.
                if on_release is not None:
                    try:
                        on_release(slot)
                    except Exception as exc:
                        runtime.record_event("payload_drop_log_failed", slot=slot, reason=str(exc))
            hold(settings.release_hold_s)
            if not simulate_release:
                runtime.record_event("payload_release_done", actuator="payload_servo",
                                     cargo_count=1, slot=slot, behavior=settings.behavior,
                                     reason="PWM command written; no mechanical feedback")

        if settings.behavior == "right_mount":
            back = drop_turn(tracker.wall.bearing_rad, REAR_BEARING_RAD, limits=limits,
                             tracker=tracker, turn_direction=-1, clock=clock,
                             handoff=lambda wall: can_move_normal(wall, REAR_BEARING_RAD, limits))
            if not run_action(back, "right_mount_right_90deg"):
                raise RuntimeError("right_mount_right_90deg did not complete")

        return_wall = tracker.wall
        if return_wall is None:
            raise RuntimeError("reference wall lost before the return")
        ret = MoveNormalToWall(target_distance_m=d0, direction=-1,
                               reference_bearing_rad=return_wall.bearing_rad,
                               desired_bearing_rad=REAR_BEARING_RAD,
                               limits=limits, tracker=tracker, clock=clock)
        if not run_action(ret, "return_from_drop"):
            raise RuntimeError("return_from_drop did not complete")

        restore_wall = tracker.wall
        if restore_wall is None:
            raise RuntimeError("reference wall lost before restoring the road heading")
        restore = drop_turn(restore_wall.bearing_rad, RIGHT_BEARING_RAD, limits=limits,
                            tracker=tracker, turn_direction=-1, clock=clock,
                            handoff=lambda wall: can_follow_wall(wall, d0, limits))
        if not run_action(restore, "right_90deg"):
            raise RuntimeError("right_90deg did not complete")
        runtime.mission.on_payload_action_done()
        runtime.record_event("payload_detour_done", actuator="payload_servo",
                             behavior=settings.behavior, reference_id=reference.reference_id,
                             commanded_slots=tuple(commanded), feedback_source="wall_scan")
        return tuple(commanded)
    except BaseException as exc:
        runtime.record_event("payload_detour_failed",
                             reason=f"{type(exc).__name__}: {exc}",
                             behavior=settings.behavior, commanded_slots=tuple(commanded))
        raise
    finally:
        primary_error = sys.exc_info()[0] is not None
        runtime.action = None
        cleanup_error = None
        try:
            runtime.drive.stop()
        except Exception as exc:
            cleanup_error = exc
            runtime.record_event("payload_detour_cleanup_failed", reason=str(exc))
        # Cleanup opens every slot for safety, and is logged as cleanup -- never
        # as five successful target releases.
        if not simulate_release and (primary_error or cleanup_error):
            try:
                if open_payload() is False:
                    raise RuntimeError('cleanup payload OPEN failed: returned False')
                runtime.record_event("cleanup_open", actuator="payload_servo", slots="all",
                                     reason="detour failure or cleanup")
            except Exception as exc:
                runtime.record_event("payload_detour_cleanup_failed", reason=str(exc))
                if not primary_error:
                    cleanup_error = cleanup_error or exc
