"""Interruptible left-side payload detour using the existing runtime motion API."""

from dataclasses import dataclass, replace
import math
import sys
import time

from core.frames import normalize_angle_rad
from components.basic_motion_controller import MotionActionState
from components.payload_task import drop_payload, payload_channel
from mission_control import step_runtime as _step, usable_pose as _usable_pose
from robocup_runtime import RobocupMissionState

PERIOD_S = 0.05


@dataclass(frozen=True)
class DetourSettings:
    payload_slot: int = 1  # 1=右前CH2，2=中间CH3，3=左前CH4；CH1无电磁铁。
    advance_m: float = 0.07
    approach_m: float = 0.47
    release_hold_s: float = 0.5
    position_tolerance_m: float = 0.005
    verify_relay: bool = True
    patrol_slow_speed_m_s: float = 0.08

    def __post_init__(self):
        if (isinstance(self.payload_slot, bool) or not isinstance(self.payload_slot, int)
                or self.payload_slot not in (1, 2, 3)):
            raise ValueError("payload slot must be 1, 2 or 3")
        for name in ("advance_m", "approach_m", "release_hold_s", "position_tolerance_m",
                     "patrol_slow_speed_m_s"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")


def run_payload_detour(runtime, settings, *, guard=lambda: None,
                       check_vision=lambda: None, clock=time.monotonic, sleep=time.sleep,
                       travel_drive=None):
    """Advance 7cm, left90, forward47cm, release, reverse47cm, restore road yaw.

    Caller owns runtime/relay lifecycle and resumes its original segment.
    Every motion/hold tick checks cancellation, vision and runtime safety.
    Absolute turn headings avoid accumulating left/right turn tolerances.
    Translation, headings and saved return endpoints share the fused pose frame.
    """
    relay = runtime.relay
    original = runtime.motion.navigation
    entry_drive = runtime.motion.drive
    hold_failure = None

    def check():
        guard()
        check_vision()

    def fused_pose(result):
        if not _usable_pose(result):
            raise RuntimeError("payload fused pose unavailable")
        return result.estimate.pose

    def motion(label, method, *args, **kwargs):
        check()
        runtime.motion.stop()
        if runtime.mission.state is RobocupMissionState.TARGET_OPERATION:
            runtime.mission.on_payload_action_done()
        getattr(runtime.motion, method)(*args, **kwargs)
        runtime.record_event("payload_detour_stage_start", stage=label, args=args, kwargs=kwargs,
                             pose_reference="fused")
        while True:
            check()
            result = _step(runtime)
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
        ux, uy = (B[0]-A[0])/settings.approach_m, (B[1]-A[1])/settings.approach_m
        raw = (pose.x_m-A[0])*ux + (pose.y_m-A[1])*uy
        cross = ux*(pose.y_m-A[1])-uy*(pose.x_m-A[0])
        runtime.record_event("payload_drop_pose", pose_reference="fused",
                             raw_progress_m=raw,cross_track_m=cross,
                             along_track_error_m=raw-settings.approach_m,
                             yaw_error_rad=normalize_angle_rad(pose.yaw_rad-side_yaw),
                             target_yaw_rad=side_yaw)

    try:
        channel = payload_channel(settings.payload_slot)
        if relay is None or not relay.connected or relay.channel_count < channel:
            raise RuntimeError("payload relay unavailable or selected channel is missing")
        runtime.motion.navigation = replace(original, position_tolerance_m=min(
            original.position_tolerance_m, settings.position_tolerance_m))
        check()
        runtime.motion.stop()
        entry = _step(runtime)
        road_pose = fused_pose(entry)
        road_yaw = road_pose.yaw_rad
        start = (road_pose.x_m, road_pose.y_m)
        road_end = (start[0] + settings.advance_m * math.cos(road_yaw),
                    start[1] + settings.advance_m * math.sin(road_yaw))
        motion("advance_7cm", "track_global_line", start, road_end)
        runtime.record_event("payload_detour_road_pose", pose=road_pose, pose_reference="fused")
        runtime.motion.drive = travel_drive if travel_drive is not None else entry_drive
        side_yaw = road_yaw + math.pi / 2
        side_pose = motion("left_90deg", "rotate_to", side_yaw)
        A2 = (side_pose.x_m, side_pose.y_m)
        B2 = (A2[0] + settings.approach_m * math.cos(side_yaw),
              A2[1] + settings.approach_m * math.sin(side_yaw))
        motion("forward_47cm", "track_global_line", A2, B2)
        runtime.motion.stop()
        runtime.drive.stop()
        record_drop_pose(A2, B2, side_yaw)
        check()
        runtime.record_event("payload_release_start", slot=settings.payload_slot, channel=channel)
        released = drop_payload(relay, settings.payload_slot, hold_s=settings.release_hold_s,
                                verify=settings.verify_relay, sleep=hold)
        if hold_failure is not None:
            raise hold_failure
        if not released:
            raise RuntimeError("payload release or relay deactivation failed")
        runtime.record_event("payload_release_done", slot=settings.payload_slot, channel=channel)
        motion("reverse_47cm", "track_global_line", B2, A2, reverse=True)
        returned = motion("right_90deg", "rotate_to", road_yaw)
        runtime.record_event("payload_detour_done", slot=settings.payload_slot, pose=returned,
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
        # On success only the selected magnet is released; preserve other loads.
        # On failure disconnect all contacts; runtime close retries cleanup.
        if (primary_error or cleanup_error) and relay is not None and relay.connected:
            try:
                if relay.all_off(verify=settings.verify_relay) is False:
                    raise RuntimeError("payload relay all_off was not confirmed")
            except Exception as exc:
                runtime.record_event("payload_detour_cleanup_failed", reason=str(exc))
                if not primary_error:
                    runtime.mission.request_safe_stop(str(exc))
        if cleanup_error is not None and not primary_error:
            raise cleanup_error
