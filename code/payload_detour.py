"""Interruptible left-side payload detour using the existing runtime motion API."""

from dataclasses import dataclass, replace
import math
import sys
import time

from core.frames import normalize_angle_rad
from components.basic_motion_controller import MotionActionState
from components.payload_servos import open_payload
from mission_control import step_runtime as _step, usable_pose as _usable_pose
from robocup_runtime import RobocupMissionState

PERIOD_S = 0.05


@dataclass(frozen=True)
class DetourSettings:
    payload_slot: int = 0  # 0: yellow cargo; 1: task-board cargo (one release each).
    approach_m: float = 0.43
    release_hold_s: float = 0.5
    position_tolerance_m: float = 0.005
    patrol_slow_speed_m_s: float = 0.08

    def __post_init__(self):
        if (isinstance(self.payload_slot, bool) or not isinstance(self.payload_slot, int)
                or self.payload_slot not in (0, 1)):
            raise ValueError('payload cargo slot must be 0 or 1')
        for name in ("approach_m", "release_hold_s", "position_tolerance_m",
                     "patrol_slow_speed_m_s"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")


def run_payload_detour(runtime, settings, *, guard=lambda: None,
                       check_vision=lambda: None, clock=time.monotonic, sleep=time.sleep,
                       travel_drive=None, fine_align=None, simulate_release=False):
    """Left90 at the centred stop, approach, release, return, restore road yaw.

    Without ``fine_align`` the approach uses settings.approach_m. With it, the
    callback owns a fused side-axis approach and stopped distance verification.
    Return starts at the latest fused pose and reverses parallel to side_yaw
    into the pivot's inner cross-section before the single inward turn.

    Caller owns runtime lifecycle and resumes its original segment.
    Every motion/hold tick checks cancellation, vision and runtime safety.
    Absolute turn headings avoid accumulating left/right turn tolerances.
    Translation, headings and saved return endpoints share the fused pose frame.
    """
    original = runtime.motion.navigation
    entry_drive = runtime.motion.drive
    hold_failure = None
    on_side_line = False
    side_yaw = None

    def check():
        guard()
        check_vision()

    def fused_pose(result):
        if not _usable_pose(result):
            raise RuntimeError("payload fused pose unavailable")
        return result.estimate.pose

    def motion(label, method, *args, on_tick=None, **kwargs):
        check()
        if on_side_line:
            if method != "track_global_line":
                raise ValueError("side approach/return permits only straight fused lines")
            delta = (args[1][0]-args[0][0], args[1][1]-args[0][1])
            cross = delta[0]*math.sin(side_yaw)-delta[1]*math.cos(side_yaw)
            if abs(cross) > 1e-6:
                raise ValueError("side approach/return must remain parallel to side_yaw")
        runtime.motion.stop()
        if runtime.mission.state is RobocupMissionState.TARGET_OPERATION:
            runtime.mission.on_payload_action_done()
        getattr(runtime.motion, method)(*args, **kwargs)
        runtime.record_event("payload_detour_stage_start", stage=label, args=args, kwargs=kwargs,
                             pose_reference="fused")
        while True:
            check()
            result = _step(runtime)
            if _usable_pose(result) and on_tick is not None and on_tick(result.estimate.pose):
                runtime.motion.stop()
                runtime.drive.stop()
                settled = _step(runtime)
                runtime.record_event("payload_detour_stage_done", stage=label,
                                     pose=settled.estimate.pose, early_stop=True,
                                     pose_reference="fused")
                return fused_pose(settled)
            if runtime.motion.state is MotionActionState.SUCCEEDED:
                runtime.drive.stop()
                runtime.record_event("payload_detour_stage_done", stage=label,
                                     pose=result.estimate.pose, pose_reference="fused")
                return fused_pose(result)
            if runtime.motion.state not in {MotionActionState.RUNNING, MotionActionState.POSE_LOST}:
                raise RuntimeError(f"{label}: motion failed: {runtime.motion.state.value}")
            sleep(PERIOD_S)

    def hold(duration):
        nonlocal hold_failure
        try:
            deadline = clock() + duration
            while clock() < deadline:
                check()
                _step(runtime)  # Motion cancelled: maintain zero output and live fusion.
                sleep(min(PERIOD_S, max(0, deadline - clock())))
            check()
        except BaseException as exc:
            hold_failure = exc
            raise

    def record_drop_pose(A, B, side_yaw):
        check()
        result = _step(runtime)  # Motion cancelled: maintain zero output.
        pose = fused_pose(result)
        ux, uy = math.cos(side_yaw), math.sin(side_yaw)
        span = (B[0]-A[0])*ux + (B[1]-A[1])*uy
        raw = (pose.x_m-A[0])*ux + (pose.y_m-A[1])*uy
        cross = ux*(pose.y_m-A[1])-uy*(pose.x_m-A[0])
        runtime.record_event("payload_drop_pose", pose_reference="fused",
                             raw_progress_m=raw,cross_track_m=cross,
                             along_track_error_m=raw-span,
                             yaw_error_rad=normalize_angle_rad(pose.yaw_rad-side_yaw),
                             target_yaw_rad=side_yaw)

    try:
        runtime.motion.navigation = replace(original, position_tolerance_m=min(
            original.position_tolerance_m, settings.position_tolerance_m))
        check()
        runtime.motion.stop()
        entry = _step(runtime)
        road_pose = fused_pose(entry)
        road_yaw = road_pose.yaw_rad
        runtime.record_event("payload_detour_road_pose", pose=road_pose, pose_reference="fused")
        runtime.motion.drive = travel_drive if travel_drive is not None else entry_drive
        side_yaw = road_yaw + math.pi / 2
        side_pose = motion("left_90deg", "rotate_to", side_yaw)
        on_side_line = True
        A2 = (side_pose.x_m, side_pose.y_m)
        runtime.record_event("payload_detour_pivot", pose=side_pose, side_yaw=side_yaw,
                             road_yaw=road_yaw, pose_reference="fused")
        if fine_align is None:
            B2 = (A2[0] + settings.approach_m * math.cos(side_yaw),
                  A2[1] + settings.approach_m * math.sin(side_yaw))
            motion("forward_to_drop", "track_global_line", A2, B2)
        else:
            # The callback only drives through this fused motion closure; it
            # returns the fused pose the payload is aligned at.
            final_pose = fine_align(side_pose, road_yaw, side_yaw, motion)
            B2 = (final_pose.x_m, final_pose.y_m)
        runtime.motion.stop()
        runtime.drive.stop()
        record_drop_pose(A2, B2, side_yaw)
        check()
        if simulate_release:
            runtime.record_event("payload_release_skipped", actuator="payload_servo",
                                 slot=settings.payload_slot, cargo_count=1,
                                 reason="car full-flow test; physical output disabled")
        else:
            runtime.record_event("payload_release_start", actuator="payload_servo",
                                 slot=settings.payload_slot, cargo_count=1)
            released = open_payload(settings.payload_slot)
        hold(settings.release_hold_s)
        if not simulate_release:
            runtime.record_event("payload_release_skipped" if released is False else "payload_release_done",
                                 actuator="payload_servo", cargo_count=1, slot=settings.payload_slot,
                                 reason="PWM unconfigured" if released is False else "command sent")
        # Use the latest fusion after the release hold, then reverse along the
        # saved side axis. Even a small cross-track residual cannot turn this
        # into a diagonal path to the historical XY pivot.
        current = fused_pose(_step(runtime))
        B2 = (current.x_m, current.y_m)
        ux, uy = math.cos(side_yaw), math.sin(side_yaw)
        reach = (B2[0]-A2[0])*ux + (B2[1]-A2[1])*uy
        inside = (B2[0]-reach*ux, B2[1]-reach*uy)
        if reach > settings.position_tolerance_m:
            motion("reverse_from_drop" if fine_align is None else "return_from_drop",
                   "track_global_line", B2, inside, reverse=True)
        else:
            runtime.record_event("payload_detour_stage_skipped", stage="return_from_drop",
                                 reason="already at the turn pose", pose_reference="fused")
        on_side_line = False
        returned = motion("right_90deg", "rotate_to", road_yaw)
        runtime.record_event("payload_detour_done", actuator="dual_servo", pose=returned,
                             pose_reference="fused")
        return returned
    except BaseException as exc:
        runtime.record_event("payload_detour_failed", reason=f"{type(exc).__name__}: {exc}")
        runtime.mission.request_safe_stop(str(exc))
        raise
    finally:
        primary_error = sys.exc_info()[0] is not None
        cleanup_error = None
        for stop in (runtime.drive.stop, runtime.motion.stop):
            try:
                stop()
            except Exception as exc:
                cleanup_error = cleanup_error or exc
                runtime.record_event("payload_detour_cleanup_failed", reason=str(exc))
                if not primary_error:
                    runtime.mission.request_safe_stop(str(exc))
        runtime.motion.navigation = original
        runtime.motion.drive = entry_drive
        if not simulate_release and (primary_error or cleanup_error):
            try:
                open_payload()
            except Exception as exc:
                runtime.record_event("payload_detour_cleanup_failed", reason=str(exc))
                if not primary_error:
                    cleanup_error = cleanup_error or exc
                    runtime.mission.request_safe_stop(str(exc))
        if cleanup_error is not None and not primary_error:
            raise cleanup_error
