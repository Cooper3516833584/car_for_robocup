"""Interruptible left-side payload detour using the existing runtime motion API."""

from dataclasses import dataclass, replace
import math
import sys
import time

from components.basic_motion_controller import MotionActionState
from components.payload_task import drop_payload, payload_channel
from competition_task import _step
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
    """
    relay = runtime.relay
    original = runtime.motion.navigation
    entry_drive = runtime.motion.drive
    hold_failure = None

    def check():
        guard()
        check_vision()

    def motion(label, method, *args, **kwargs):
        check()
        runtime.motion.stop()
        if runtime.mission.state is RobocupMissionState.TARGET_OPERATION:
            runtime.mission.on_payload_action_done()
        if method == "drive_distance":
            kwargs["lateral_tolerance_m"] = original.position_tolerance_m
        getattr(runtime.motion, method)(*args, **kwargs)
        runtime.record_event("payload_detour_stage_start", stage=label, args=args, kwargs=kwargs)
        while True:
            check()
            result = _step(runtime)
            if runtime.motion.state is MotionActionState.SUCCEEDED:
                runtime.drive.stop()
                runtime.record_event("payload_detour_stage_done", stage=label,
                                     pose=result.estimate.pose)
                return result.estimate.pose
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

    try:
        channel = payload_channel(settings.payload_slot)
        if relay is None or not relay.connected or relay.channel_count < channel:
            raise RuntimeError("payload relay unavailable or selected channel is missing")
        runtime.motion.navigation = replace(original, position_tolerance_m=min(
            original.position_tolerance_m, settings.position_tolerance_m))
        # Replace the patrol command directly; do not pause before the extra 7cm.
        road_pose = motion("advance_7cm", "drive_distance", settings.advance_m)
        road_yaw = road_pose.yaw_rad
        runtime.record_event("payload_detour_road_pose", pose=road_pose)
        # The visible-target cap belongs to patrol and the extra 7cm only.
        runtime.motion.drive = travel_drive if travel_drive is not None else entry_drive
        side_pose = motion("left_90deg", "rotate_to", road_yaw + math.pi / 2)
        runtime.drive.stop()
        hold(0.20)  # Keep fusion live while the completed turn settles.
        check()
        side_pose = _step(runtime).estimate.pose
        if side_pose is None:
            raise RuntimeError("payload turn settling pose unavailable")
        side_yaw = side_pose.yaw_rad
        motion("forward_47cm", "drive_distance", settings.approach_m, heading_yaw_rad=side_yaw)
        runtime.motion.stop()
        runtime.drive.stop()
        check()
        runtime.record_event("payload_release_start", slot=settings.payload_slot, channel=channel)
        released = drop_payload(relay, settings.payload_slot, hold_s=settings.release_hold_s,
                                verify=settings.verify_relay, sleep=hold)
        if hold_failure is not None:
            raise hold_failure
        if not released:
            raise RuntimeError("payload release or relay deactivation failed")
        runtime.record_event("payload_release_done", slot=settings.payload_slot, channel=channel)
        motion("reverse_47cm", "drive_distance", -settings.approach_m, heading_yaw_rad=side_yaw)
        returned = motion("right_90deg", "rotate_to", road_yaw)
        runtime.record_event("payload_detour_done", slot=settings.payload_slot, pose=returned)
        return returned
    except BaseException as exc:
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
                runtime.mission.request_safe_stop(str(exc))
        if cleanup_error is not None and not primary_error:
            raise cleanup_error
