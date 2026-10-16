"""Composition root and safe event loop for the RoboCup differential platform."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
import logging
import math
import queue
import threading
import time
from typing import TYPE_CHECKING, Callable, Iterable

if TYPE_CHECKING:
    from components.task_board_reader import TaskCounts
    from components.wall_observer import WallSnapshot

from components.differential_drive import DifferentialDrive
from components.basic_motion_controller import BasicMotionController, MotionActionState, MotionActionType, MotionOutput, MotionPhase
from components.differential_navigation import DifferentialNavigator, NavigationOutput
from components.diagnostics_log import JsonlEventLogger
from components.navigation_common import NavigationGoal
from components.pose_fusion import FusedPoseEstimate, PoseFusion, PoseFusionState
from components.pose_log_replay import PoseLogEvent, read_pose_events
from components.radar_driver import D500_TIMESTAMP_MODULUS_MS
from components.radar_pose_adapter import RadarPoseAdapter
from components.sensor_clock import DeviceClockMapper
from components.t265_driver import FakeT265PoseSource, RealSenseT265PoseSource, T265RawPose
from components.t265_pose_adapter import T265PoseAdapter
from config.v2_factory import (
    build_basic_motion_controller,
    build_differential_drive,
    build_differential_navigator,
    build_pose_fusion,
)
from config.v2_loader import load_v2_config
from config.v2_models import DifferentialRobotConfig
from config.v2_runtime import RuntimeConstraints, RuntimeMode, runtime_constraints, validate_runtime_readiness
from core.frames import compose_pose2d, inverse_pose2d
from core.types import Pose2D, PoseQuality, Twist2D

LOG = logging.getLogger("robocup-runtime")

# Two distinct freshness concepts, deliberately not one number:
#
#   * ``fusion.d500_max_age_s`` bounds how old a raw D500 measurement may be to
#     count as a fresh measurement / local fallback.  It is short (0.50 s) and
#     a D500 revolution period is ~0.14 s, so it is a sensor-level property.
#   * the map anchor established by an *accepted* absolute observation is a
#     different thing: it stays meaningful while T265 keeps propagating the
#     pose, even if D500 sees nothing for a while.  Judging the anchor with the
#     0.50 s sensor timeout would drop an established anchor roughly every
#     revolution gap and make global-ready flap.
#
# The anchor window is a code constant rather than a new TOML knob, and it is
# intentionally conservative: longer than any plausible single missed scan,
# short enough to stop trusting the map if absolute corrections stop entirely.
D500_ANCHOR_STALE_AFTER_S = 5.0


class RuntimeReadinessError(RuntimeError):
    """The configured robot is not ready for the requested runtime mode."""


class D500LocalizationMode(Enum):
    GLOBAL = "global"
    LOCAL_ONLY = "local_only"
    LOST = "lost"


class RobocupMissionState(Enum):
    INIT = "init"
    WAIT_FOR_LOCALIZATION = "wait_for_localization"
    READY = "ready"
    NAVIGATING = "navigating"
    TARGET_OPERATION = "target_operation"
    RETURNING = "returning"
    FINISHED = "finished"
    SAFE_STOP = "safe_stop"
    ERROR = "error"


@dataclass(frozen=True, slots=True)
class RuntimeStep:
    now_s: float
    estimate: FusedPoseEstimate
    mission_state: RobocupMissionState
    command: Twist2D
    navigation: NavigationOutput | None
    error: str | None = None
    motion: MotionOutput | None = None


@dataclass(frozen=True, slots=True)
class FakeD500PoseSample:
    x_cm: float
    y_cm: float
    yaw_cw_deg: float
    timestamp_s: float


@dataclass(frozen=True, slots=True)
class D500PoseObservation:
    """D500 local/map poses and absolute-wall acceptance as separate facts."""

    local_pose: Pose2D | None
    map_pose: Pose2D | None
    mode: D500LocalizationMode
    map_alignment_established: bool
    map_pose_valid: bool
    absolute_observation_available: bool
    absolute_observation_accepted: bool
    confidence: float | None
    timestamp_s: float


class FakeD500PoseSource:
    """Small deterministic legacy-pose source for dry-run and unit tests."""

    def __init__(self, samples: Iterable[FakeD500PoseSample] = ()) -> None:
        self.samples = tuple(samples)
        self._index = 0
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True
        self.stopped = False

    def read(self) -> FakeD500PoseSample | None:
        if not self.started:
            raise RuntimeError("fake D500 source is not running")
        if self._index >= len(self.samples):
            return None
        sample = self.samples[self._index]
        self._index += 1
        return sample

    def stop(self) -> None:
        self.started = False
        self.stopped = True


class RobocupMission:
    """Competition task-state skeleton, separate from planning and hardware."""

    def __init__(self, navigator: DifferentialNavigator, profile_name: str = "default", motion: BasicMotionController | None = None) -> None:
        self.navigator = navigator
        self.motion = motion or BasicMotionController(navigator, navigator.navigation, navigator.drive)
        self.profile_name = profile_name
        self.state = RobocupMissionState.INIT
        self._goal_pending = False
        self.last_error: str | None = None
        self.task_counts: TaskCounts | None = None

    def set_task_counts(self, counts: TaskCounts) -> None:
        from components.task_board_reader import TaskCounts

        if not isinstance(counts, TaskCounts):
            raise TypeError("counts must be validated TaskCounts")
        if self.task_counts is not None:
            raise RuntimeError("task board result already set")
        if self.state in {RobocupMissionState.FINISHED, RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR}:
            raise RuntimeError(f"cannot set task counts while mission is {self.state.value}")
        self.task_counts = counts

    def set_navigation_goal(self, goal: NavigationGoal) -> None:
        if self.state in {RobocupMissionState.FINISHED, RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR}:
            raise RuntimeError(f"cannot set a goal while mission is {self.state.value}")
        if goal.yaw_rad is None:
            self.motion.navigate_to(goal.x_m, goal.y_m)
        else:
            self.motion.navigate_to_pose(goal.x_m, goal.y_m, goal.yaw_rad)
        self._goal_pending = True
        if self.state is RobocupMissionState.READY:
            self.state = RobocupMissionState.NAVIGATING

    def on_localization(self, estimate: FusedPoseEstimate) -> None:
        if self.state in {RobocupMissionState.FINISHED, RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR}:
            return
        if estimate.pose is None or estimate.state in {PoseFusionState.LOST, PoseFusionState.UNANCHORED}:
            if self.state in {RobocupMissionState.NAVIGATING, RobocupMissionState.RETURNING}:
                # Runtime applies the configured loss timeout and immediately
                # requests zero output while retaining this mission state.
                pass
            else:
                self.state = RobocupMissionState.WAIT_FOR_LOCALIZATION
            return
        if self.state in {RobocupMissionState.INIT, RobocupMissionState.WAIT_FOR_LOCALIZATION}:
            self.state = RobocupMissionState.NAVIGATING if self._goal_pending else RobocupMissionState.READY

    def on_motion_done(self) -> None:
        if self.state in {RobocupMissionState.NAVIGATING, RobocupMissionState.RETURNING}:
            self._goal_pending = False
            self.state = RobocupMissionState.TARGET_OPERATION

    def on_goal_reached(self) -> None:
        self.on_motion_done()

    def on_target_detected(self) -> None:
        if self.state is RobocupMissionState.NAVIGATING:
            self.motion.stop()
            self._goal_pending = False
            self.state = RobocupMissionState.TARGET_OPERATION

    def on_payload_action_done(self) -> None:
        if self.state is RobocupMissionState.TARGET_OPERATION:
            self.state = RobocupMissionState.READY

    def set_return_goal(self, goal: NavigationGoal) -> None:
        if goal.yaw_rad is None:
            self.motion.navigate_to(goal.x_m, goal.y_m)
        else:
            self.motion.navigate_to_pose(goal.x_m, goal.y_m, goal.yaw_rad)
        self._goal_pending = True
        self.state = RobocupMissionState.RETURNING

    def finish(self) -> None:
        self.motion.stop()
        self.state = RobocupMissionState.FINISHED

    def request_safe_stop(self, reason: str = "operator requested safe stop") -> None:
        self.motion.safe_stop(reason)
        self.last_error = reason
        self.state = RobocupMissionState.SAFE_STOP

    def request_error(self, reason: str) -> None:
        self.motion.safe_stop(reason)
        self.last_error = reason
        self.state = RobocupMissionState.ERROR


class RobocupRuntime:
    """Own sources, fusion, mission, navigation and drive shutdown order."""

    def __init__(
        self,
        config: DifferentialRobotConfig,
        mode: RuntimeMode,
        *,
        drive: DifferentialDrive,
        d500_source,
        t265_source,
        d500_fake: bool,
        t265_adapter: T265PoseAdapter,
        radar_adapter: RadarPoseAdapter,
        fusion: PoseFusion,
        navigator: DifferentialNavigator,
        motion: BasicMotionController | None = None,
        clock: Callable[[], float],
        mission_profile: str = "default",
        constraints: RuntimeConstraints | None = None,
        event_logger: JsonlEventLogger | None = None,
        replay_events: tuple[PoseLogEvent, ...] = (),
        slam_bridge=None,
    ) -> None:
        self.config = config
        self.mode = mode
        self.drive = drive
        self.slam_bridge = slam_bridge
        self._last_slam_anchor_timestamp_s: float | None = None
        self._last_slam_metrics_s: float | None = None
        self._last_slam_metrics: dict | None = None
        self.d500_source = d500_source
        self.t265_source = t265_source
        self.d500_fake = d500_fake
        self.t265_adapter = t265_adapter
        self.radar_adapter = radar_adapter
        self.fusion = fusion
        self.navigator = navigator
        self.motion = motion or BasicMotionController(navigator, config.navigation, config.drive,
                                                      track_width_m=config.geometry.drive_track_width_m)
        self.mission = RobocupMission(navigator, mission_profile, self.motion)
        self.clock = clock
        self.constraints = constraints or runtime_constraints(config, mode)
        self.event_logger = event_logger
        self._replay_events = replay_events
        self._replay_index = 0
        self._start_time_s: float | None = None
        self._current_step_s: float | None = None
        self._last_safety_state: RobocupMissionState | None = None
        self._pose_lost_since_s: float | None = None
        self._last_watchdog_stop_count = drive.watchdog_stop_count
        self._d500_events: queue.SimpleQueue[D500PoseObservation] = queue.SimpleQueue()
        self.d500_abs_accept_count = 0
        self.d500_abs_reject_low_confidence = 0
        self.d500_abs_reject_position_gate = 0
        self.d500_abs_reject_yaw_gate = 0
        self._last_d500_absolute_update_s: float | None = None
        self._last_d500_map_pose_update_s: float | None = None
        self._d500_global_alignment_trusted = False
        # Measured on the car, not assumed: see D500_TIMESTAMP_MODULUS_MS.
        self._d500_clock = DeviceClockMapper(
            modulus_ms=D500_TIMESTAMP_MODULUS_MS
        )
        self._d500_global_pending = False
        self._slam_action_started_with_fresh_anchor = False
        self._started = False
        self._closed = False
        self._error: str | None = None

    @property
    def is_running(self) -> bool:
        return self._started and not self._closed

    def start(self) -> "RobocupRuntime":
        if self._started:
            raise RuntimeError("RoboCup runtime is already started")
        started_sources: list[object] = []
        try:
            if self.d500_source is not None:
                self.d500_source.start()
                started_sources.append(self.d500_source)
            if self.t265_source is not None:
                self.t265_source.start()
                started_sources.append(self.t265_source)
            if self.slam_bridge is not None:
                self.slam_bridge.start()
                started_sources.append(self.slam_bridge)
            if self.mode in {RuntimeMode.DRY_RUN, RuntimeMode.REPLAY}:
                self.drive.start()
            self._start_time_s = float(self.clock())
            self._started = True
            self._emit("runtime_started", mode=self.mode.value, priority=True)
            return self
        except BaseException:
            for source in reversed(started_sources):
                self._stop_source(source)
            self._safe_stop_drive()
            raise

    def step(self, *, now_s: float | None = None) -> RuntimeStep:
        if not self._started or self._closed:
            raise RuntimeError("RoboCup runtime is not running")
        live_time = now_s is None
        now = float(self.clock() if now_s is None else now_s)
        if not math.isfinite(now):
            raise ValueError("runtime clock must be finite monotonic time")
        if self.mode is RuntimeMode.DRY_RUN and self.motion.state is MotionActionState.IDLE and self.mission.state in {
            RobocupMissionState.INIT, RobocupMissionState.WAIT_FOR_LOCALIZATION, RobocupMissionState.READY,
        }:
            # Supply the synthetic acceptance goal only if no task action was submitted.
            self.mission.set_navigation_goal(NavigationGoal(0.8, 0.0))
        self._current_step_s = now
        watchdog_stops = self.drive.watchdog_stop_count
        navigation_output: NavigationOutput | None = None
        motion_output: MotionOutput | None = None
        if watchdog_stops > self._last_watchdog_stop_count:
            self._emit("safety", event_code="WATCHDOG_STOP", count=watchdog_stops - self._last_watchdog_stop_count, priority=True)
            self._last_watchdog_stop_count = watchdog_stops
        try:
            self._consume_t265(now, live_time=live_time)
            if live_time:
                # A hardware read can block; use a clock value taken after it.
                now = float(self.clock())
                self._current_step_s = now
            # Give the D500 a map-frame prediction before consuming its scans,
            # so wall association never depends on the D500's own ICP.
            self._publish_d500_pose_hint(now)
            self._consume_d500(now)
            self._consume_slam(now)
            if live_time:
                now = float(self.clock())
                self._current_step_s = now
            if self.mode is RuntimeMode.REPLAY:
                self._consume_replay(now)
            estimate = self.fusion.estimate(now)
            self._emit(
                "fused_pose",
                x_m=None if estimate.pose is None else estimate.pose.x_m,
                y_m=None if estimate.pose is None else estimate.pose.y_m,
                yaw_rad=None if estimate.pose is None else estimate.pose.yaw_rad,
                state=estimate.state.value,
                age_s=estimate.age_s,
                source_flags=estimate.source_flags,
                fusion_source=(
                    "T265_SLAM" if "slam" in estimate.source_flags and "fused" in estimate.source_flags
                    else "T265_FUSED" if "fused" in estimate.source_flags
                    else "D500_FALLBACK" if "d500_global_fallback" in estimate.source_flags
                    else "LOST" if estimate.pose is None
                    else estimate.state.value.upper()
                ),
                anchor_valid=estimate.anchor_initialized,
                slam_anchor_valid=(self.slam_bridge is not None and self.slam_bridge.healthy(now)),
                field_anchor_valid=(self.fusion.field_T_slam_map is not None),
                d500_accepted=estimate.d500_accepted,
                d500_innovation_m=estimate.last_d500_innovation_m,
                d500_innovation_yaw_rad=estimate.last_d500_innovation_yaw_rad,
                rejection_reason=estimate.rejection_reason,
                anchor_initialized=estimate.anchor_initialized,
                t265_confidence=estimate.t265_confidence,
                t265_time_alignment_ms=estimate.t265_time_alignment_ms,
                t265_continuity_broken=estimate.t265_continuity_broken,
                slam_consensus_pending=estimate.slam_consensus_pending,
                slam_observation_age_s=estimate.slam_observation_age_s,
                slam_accepted_age_s=estimate.d500_age_s if self.config.localization.backend == "slam_toolbox" else None,
            )
            localization_loss_pending = self._handle_localization(estimate, now)
            d500_global_pending = not self._hardware_global_localization_ready(now)
            if d500_global_pending and not self._d500_global_pending:
                self._emit("safety", event_code="D500_GLOBAL_PENDING", state=self.mission.state.value, priority=True)
            self._d500_global_pending = d500_global_pending

            if self.mission.state is RobocupMissionState.READY and self.motion.state is MotionActionState.RUNNING:
                self.mission.state = RobocupMissionState.NAVIGATING

            command = Twist2D(0.0, 0.0)
            if not localization_loss_pending and not d500_global_pending and self.mode is not RuntimeMode.HARDWARE_PROBE and self.mission.state in {
                RobocupMissionState.NAVIGATING,
                RobocupMissionState.RETURNING,
            }:
                motion_pose = self._motion_control_pose(estimate)
                pose_reference = ("t265_local" if self.motion.action_type in {MotionActionType.TRACK_LOCAL_LINE,
                    MotionActionType.ROTATE_LOCAL_TO, MotionActionType.ROTATE_RELATIVE} else "fused")
                motion_output = self.motion.step(
                    motion_pose,
                    now_s=now,
                    pose_state=estimate.state,
                    t265_pose=self.current_local_pose(estimate, now_s=now),
                    pose_reference=pose_reference,
                )
                navigation_output = self.motion.last_navigation_output
                command = motion_output.command
                if motion_output.state is MotionActionState.SUCCEEDED:
                    self._slam_action_started_with_fresh_anchor = False
                    self.mission.on_motion_done()
                    command = Twist2D(0.0, 0.0)
                elif motion_output.state in {MotionActionState.BLOCKED, MotionActionState.POSE_LOST, MotionActionState.SAFE_STOPPED, MotionActionState.ERROR}:
                    self._slam_action_started_with_fresh_anchor = False
                    self.mission.request_safe_stop(
                        str(motion_output.diagnostics.get("reason", motion_output.state.value))
                    )
                    command = Twist2D(0.0, 0.0)
                self._emit("motion_action", action=None if motion_output.action_type is None else motion_output.action_type.value,
                           state=motion_output.state.value, phase=None if motion_output.phase is None else motion_output.phase.value,
                           diagnostics=motion_output.diagnostics, command=command, pose_reference=pose_reference)
                if navigation_output is not None:
                    self._emit(
                        "navigation",
                        state=navigation_output.state.value,
                        diagnostics=navigation_output.diagnostics,
                        path=navigation_output.path,
                        command=command,
                        goal=self.navigator.goal,
                    )

            if self.mission.state in {
                RobocupMissionState.SAFE_STOP,
                RobocupMissionState.ERROR,
                RobocupMissionState.FINISHED,
                RobocupMissionState.TARGET_OPERATION,
            }:
                command = Twist2D(0.0, 0.0)
                self._safe_stop_drive()
                if self._last_safety_state is not self.mission.state:
                    self._emit(
                        "safety",
                        event_code="POSE_LOST" if estimate.state is PoseFusionState.LOST else self.mission.state.value.upper(),
                        state=self.mission.state.value,
                        reason=self.mission.last_error or estimate.state.value,
                        priority=True,
                    )
                    self._last_safety_state = self.mission.state
            elif localization_loss_pending or d500_global_pending:
                command = Twist2D(0.0, 0.0)
                if self.drive.is_running:
                    self.drive.stop()
                self._emit(
                    "safety",
                    event_code="POSE_LOSS_PENDING" if localization_loss_pending else "D500_GLOBAL_PENDING",
                    state=self.mission.state.value,
                    elapsed_s=(
                        0.0 if self._pose_lost_since_s is None
                        else max(0.0, now - self._pose_lost_since_s)
                    ) if localization_loss_pending else None,
                )
            elif self.mode is RuntimeMode.HARDWARE_PROBE:
                command = Twist2D(0.0, 0.0)
            elif command.linear_x_m_s != 0.0 or command.angular_z_rad_s != 0.0:
                drive_now = self._ensure_drive_started_and_get_time(now)
                self.drive.command(command, now_s=drive_now)
            elif self.drive.is_running:
                self.drive.stop()

            limited = self.drive.last_limited_twist
            wheels = self.drive.kinematics.twist_to_wheels(limited)
            encoded_wheels = (getattr(self.drive.backend, "last_encoded_wheel_speeds_m_s", None)
                              if self.drive.is_running else None)
            c10b_command = (getattr(self.drive.backend, "last_c10b_chassis_command", None)
                            if self.drive.is_running else None)
            encoded_left = None if encoded_wheels is None else encoded_wheels[0]
            encoded_right = None if encoded_wheels is None else encoded_wheels[1]
            self._emit(
                "drive_command",
                requested_v_m_s=command.linear_x_m_s,
                requested_omega_rad_s=command.angular_z_rad_s,
                limited_v_m_s=limited.linear_x_m_s,
                limited_omega_rad_s=limited.angular_z_rad_s,
                left_target_m_s=wheels.left_m_s,
                right_target_m_s=wheels.right_m_s,
                c10b_encoded_left_m_s=encoded_left,
                c10b_encoded_right_m_s=encoded_right,
                c10b_vx_mm_s=None if c10b_command is None else c10b_command.linear_mm_s,
                c10b_vz_mrad_s=None if c10b_command is None else c10b_command.angular_mrad_s,
                c10b_reverse_compensation_applied=(
                    self.config.drive.protocol_mode == "differential_vx_vz"
                    and limited.linear_x_m_s < 0.0
                    and encoded_wheels is not None
                ),
                protocol_mode=self.config.drive.protocol_mode,
                actuation_enabled=self.drive.is_running,
            )

            return RuntimeStep(now, estimate, self.mission.state, command, navigation_output, motion=motion_output)
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            self.mission.request_error(self._error)
            self._safe_stop_drive()
            if type(exc).__name__ in {"UnsupportedFirmwareMotion", "UnsupportedWheelCommand"} or "turn radius" in str(exc).lower():
                self._emit("c10b_reject", event_code="C10B_REJECT", reason=self._error, priority=True)
            self._emit("runtime_error", error=self._error, priority=True)
            LOG.exception("RoboCup runtime step failed")
            estimate = self.fusion.estimate(now)
            return RuntimeStep(now, estimate, self.mission.state, Twist2D(0.0, 0.0), navigation_output, self._error, motion_output)

    def current_local_pose(self, estimate=None, *, now_s=None):
        """Fresh, confident, continuous base_link T265 odometry; never SLAM feed."""
        now = float(self.clock()) if now_s is None else float(now_s)
        estimate = self.fusion.estimate(now) if estimate is None else estimate
        pose = self.fusion.continuous_t265_pose
        if (pose is None or estimate.t265_continuity_broken
                or estimate.t265_age_s is None or estimate.t265_confidence is None
                or not 0 <= estimate.t265_age_s <= self.config.fusion.t265_max_age_s
                or estimate.t265_confidence < self.config.fusion.t265_min_tracker_confidence / 3
                or not all(math.isfinite(v) for v in (pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s))
                or not 0 <= now - pose.timestamp_s <= self.config.fusion.t265_max_age_s):
            return None
        return pose

    def _motion_control_pose(self, estimate):
        if self.motion.action_type in {MotionActionType.TRACK_LOCAL_LINE,
                                      MotionActionType.ROTATE_LOCAL_TO,
                                      MotionActionType.ROTATE_RELATIVE}:
            return self.current_local_pose(estimate, now_s=self._current_step_s)
        return estimate.pose

    def run_steps(self, count: int, *, period_s: float = 0.05) -> list[RuntimeStep]:
        if count < 0 or period_s <= 0.0:
            raise ValueError("count must be non-negative and period_s positive")
        if not self._started:
            self.start()
        results: list[RuntimeStep] = []
        try:
            for _ in range(count):
                results.append(self.step())
                if self.mission.state in {RobocupMissionState.FINISHED, RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR}:
                    break
                time.sleep(period_s)
        finally:
            self.close()
        return results

    def run(self, *, period_s: float = 0.05) -> None:
        if not self._started:
            self.start()
        try:
            while self.is_running and self.mission.state not in {
                RobocupMissionState.FINISHED,
                RobocupMissionState.SAFE_STOP,
                RobocupMissionState.ERROR,
            }:
                self.step()
                time.sleep(period_s)
        except KeyboardInterrupt:
            LOG.info("keyboard interrupt; stopping RoboCup runtime")
            self.mission.request_safe_stop("keyboard interrupt")
        finally:
            self.close()

    def run_replay(self) -> list[RuntimeStep]:
        """Run recorded event times without sleeping or consulting wall time."""
        if not self._started:
            self.start()
        results: list[RuntimeStep] = []
        assert self._start_time_s is not None
        try:
            for timestamp in sorted({event.t_s for event in self._replay_events}):
                results.append(self.step(now_s=self._start_time_s + timestamp))
                if self.mission.state in {RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR}:
                    break
        finally:
            self.close()
        return results

    def close(self) -> None:
        if self._closed:
            return
        self._emit("runtime_closed", mission_state=self.mission.state.value, priority=True)
        # Stop the base first; source shutdown can block while joining workers.
        self._safe_stop_drive()
        for source in (self.t265_source, self.d500_source):
            if source is not None:
                self._stop_source(source)
        if self.slam_bridge is not None:
            self._stop_source(self.slam_bridge)
        try:
            self.drive.close()
        except Exception:
            LOG.exception("failed to close drive")
        if self.event_logger is not None:
            try:
                self.event_logger.close()
            except Exception:
                LOG.exception("failed to close JSONL diagnostics logger")
        self._closed = True

    def _consume_t265(self, now_s: float, *, live_time: bool = False) -> None:
        if self.t265_source is None:
            return
        raw = self.t265_source.read()
        if raw is None:
            return
        mapped_measurement_time = raw.measurement_monotonic_s
        time_clamped = (live_time and self.config.localization.backend == "slam_toolbox"
                and raw.measurement_monotonic_s is not None
                and raw.measurement_monotonic_s > raw.received_monotonic_s)
        if time_clamped:
            # The device-clock fit can lead the host receipt by a few ms.
            # Keep the production SLAM path causal, as in the manual probe.
            raw = replace(raw, measurement_monotonic_s=raw.received_monotonic_s)
        sample_now = float(self.clock()) if live_time else now_s
        if live_time:
            self._current_step_s = sample_now
        update = self.t265_adapter.adapt(raw, now_s=sample_now)
        if update.pose is not None and update.quality.valid:
            self.fusion.update_t265(update.pose, update.quality)
            if self.slam_bridge is not None:
                # Feed the bridge through the SLAM-specific accessor, not the
                # navigation gate: a continuity break must keep navigation
                # gated but must never starve the only producer of a SLAM
                # anchor (see PoseFusion.slam_feed_t265_pose).
                continuous = self.fusion.slam_feed_t265_pose
                if continuous is not None:
                    try:
                        self.slam_bridge.push_t265(continuous, update.quality)
                    except Exception:
                        LOG.exception("could not queue T265 pose for SLAM")
            self._emit(
                "t265_pose",
                x_m=update.pose.x_m,
                y_m=update.pose.y_m,
                yaw_rad=update.pose.yaw_rad,
                confidence=update.quality.position_confidence,
                age_s=update.quality.age_s,
                t265_measurement_time=update.pose.timestamp_s,
                t265_received_time=raw.received_monotonic_s,
                t265_device_timestamp_ms=raw.device_timestamp_ms,
                raw_translation_xyz=raw.translation_xyz,
                raw_quaternion_xyzw=raw.quaternion_xyzw,
                raw_velocity_xyz=raw.velocity_xyz,
                raw_angular_velocity_xyz=raw.angular_velocity_xyz,
                t265_mapped_measurement_time_before_causal_clamp=mapped_measurement_time,
                t265_measurement_time_used=raw.measurement_monotonic_s,
                t265_received_time_monotonic=raw.received_monotonic_s,
                measurement_to_receive_ms=(None if mapped_measurement_time is None else
                                           (raw.received_monotonic_s - mapped_measurement_time) * 1000.0),
                measurement_time_clamped_to_receive=bool(time_clamped),
                tracker_confidence=raw.tracker_confidence,
                mapper_confidence=raw.mapper_confidence,
            )
        else:
            self.fusion.update_t265(None, update.quality)
            self._emit(
                "t265_rejected",
                reason=update.reason or "invalid",
                event_code="LOW_T265_CONFIDENCE" if update.reason == "tracker_confidence_too_low" else "T265_REJECT",
                confidence=raw.tracker_confidence,
                t265_measurement_time=raw.measurement_monotonic_s,
                t265_received_time=raw.received_monotonic_s,
                t265_device_timestamp_ms=raw.device_timestamp_ms,
                raw_translation_xyz=raw.translation_xyz,
                raw_quaternion_xyzw=raw.quaternion_xyzw,
                raw_velocity_xyz=raw.velocity_xyz,
                raw_angular_velocity_xyz=raw.angular_velocity_xyz,
                t265_mapped_measurement_time_before_causal_clamp=mapped_measurement_time,
                t265_measurement_time_used=raw.measurement_monotonic_s,
                t265_received_time_monotonic=raw.received_monotonic_s,
                measurement_to_receive_ms=(None if mapped_measurement_time is None else
                                           (raw.received_monotonic_s - mapped_measurement_time) * 1000.0),
                measurement_time_clamped_to_receive=bool(time_clamped),
                mapper_confidence=raw.mapper_confidence,
            )

    def _publish_d500_pose_hint(self, now_s: float) -> None:
        """Push the fused/T265 map pose to the D500 for wall association.

        The D500 needs a map-frame prediction to associate the safety-net edges.
        Before this existed it fell back on its own scan-to-scan ICP pose, which
        is exactly what degrades during in-place rotation, so the absolute
        boundary observer lost its prior whenever it was most needed.

        The hint is a prediction only: it narrows the wall search, and every
        candidate still has to pass the residual/innovation gates.
        """

        source = self.d500_source
        if self.d500_fake or source is None:
            return
        setter = getattr(source, "set_global_pose_hint", None)
        if setter is None:
            return
        try:
            estimate = self.fusion.estimate(now_s)
        except Exception:  # noqa: BLE001 - a hint must never break the loop
            LOG.exception("could not compute a D500 pose hint")
            return
        pose = estimate.pose if estimate.anchor_initialized else None
        setter(None if pose is None else self.radar_adapter.from_map_base_pose(pose))

    def _consume_d500(self, now_s: float) -> None:
        if self.d500_fake:
            if self.d500_source is None:
                return
            sample = self.d500_source.read()
            if sample is not None:
                pose = self.radar_adapter.to_map_base_pose(sample, timestamp_s=sample.timestamp_s)
                quality = PoseQuality("d500", True, False, age_s=max(0.0, now_s - sample.timestamp_s))
                self.fusion.update_d500_absolute(pose, quality)
                self._d500_global_alignment_trusted = True
                self.fusion.update_d500_global_fallback(
                    pose, quality, map_alignment_valid=True
                )
                self._emit("d500_pose", x_m=pose.x_m, y_m=pose.y_m, yaw_rad=pose.yaw_rad)
        else:
            while True:
                try:
                    observation = self._d500_events.get_nowait()
                except queue.Empty:
                    break
                if self.config.localization.backend == "slam_toolbox":
                    if (observation.absolute_observation_accepted
                            and observation.map_pose is not None
                            and observation.confidence is not None
                            and observation.confidence >= self.config.d500_localization.min_confidence):
                        quality = PoseQuality("fixed_wall", True, False,
                                              position_confidence=observation.confidence)
                        if self.fusion.update_field_wall(observation.map_pose, quality):
                            self._d500_global_alignment_trusted = True
                            self._last_d500_absolute_update_s = observation.timestamp_s
                            self.d500_abs_accept_count += 1
                        else:
                            self._emit("d500_abs_rejected", reason="field_alignment_gate")
                    continue
                mode = observation.mode
                confidence = None
                pose = None
                accepted_absolute = False
                absolute_correction_accepted = False
                if observation.map_pose_valid:
                    self._last_d500_map_pose_update_s = observation.timestamp_s
                if observation.absolute_observation_accepted and observation.map_pose is not None:
                    confidence = observation.confidence
                    if confidence is None or confidence < self.config.d500_localization.min_confidence:
                        self.d500_abs_reject_low_confidence += 1
                        self._emit("d500_abs_rejected", reason="low_confidence", confidence=confidence)
                    else:
                        # Fusion gates innovations against T265 at the D500
                        # measurement timestamp, not against the latest pose.
                        # Producing a candidate is NOT an acceptance: the
                        # trusted flag and the last-absolute time are only
                        # refreshed below, after PoseFusion accepts.
                        pose = observation.map_pose
                        accepted_absolute = True
                quality = PoseQuality(
                    "d500_wall_absolute" if observation.absolute_observation_accepted else "d500_local_or_propagated",
                    True,
                    not observation.absolute_observation_accepted,
                    position_confidence=confidence if observation.absolute_observation_accepted else None,
                    heading_confidence=confidence if observation.absolute_observation_accepted else None,
                    age_s=max(0.0, now_s - observation.timestamp_s),
                )
                if accepted_absolute and pose is not None:
                    self.fusion.update_d500_absolute(pose, quality)
                    fusion_estimate = self.fusion.estimate(now_s)
                    absolute_correction_accepted = fusion_estimate.d500_accepted is True
                    if absolute_correction_accepted:
                        # Only a fusion-accepted absolute observation may
                        # establish or refresh the map anchor.
                        self._d500_global_alignment_trusted = True
                        self._last_d500_absolute_update_s = observation.timestamp_s
                        self.d500_abs_accept_count += 1
                    else:
                        reason = fusion_estimate.rejection_reason or "fusion_rejected"
                        if reason == "position_innovation_gate":
                            self.d500_abs_reject_position_gate += 1
                        elif reason == "yaw_innovation_gate":
                            self.d500_abs_reject_yaw_gate += 1
                        self._emit("d500_abs_rejected", reason=reason, confidence=confidence)
                    self._emit(
                        "d500_pose",
                        x_m=pose.x_m,
                        y_m=pose.y_m,
                        yaw_rad=pose.yaw_rad,
                        d500_mode=mode.value,
                        confidence=confidence,
                        d500_update_accepted=absolute_correction_accepted,
                        d500_reject_reason=fusion_estimate.rejection_reason,
                        d500_absolute_observation_accepted=observation.absolute_observation_accepted,
                    )
                elif observation.map_pose_valid and observation.map_pose is not None:
                    self._emit(
                        "d500_pose",
                        x_m=observation.map_pose.x_m,
                        y_m=observation.map_pose.y_m,
                        yaw_rad=observation.map_pose.yaw_rad,
                        d500_mode="GLOBAL_PREDICTED",
                        confidence=None,
                        d500_map_alignment_established=observation.map_alignment_established,
                        d500_global_alignment_trusted=self._d500_global_alignment_trusted,
                    )
                elif observation.local_pose is not None:
                    self._emit("d500_pose", x_m=observation.local_pose.x_m, y_m=observation.local_pose.y_m, yaw_rad=observation.local_pose.yaw_rad, d500_mode=D500LocalizationMode.LOCAL_ONLY.value, confidence=None)
                else:
                    self._emit("d500_rejected", reason="localization_lost", d500_mode=D500LocalizationMode.LOST.value)
                now = float(now_s)
                # The propagated global fallback is a *validated* map pose: it
                # requires a valid map pose (map anchor established) but is
                # deliberately not restricted to fusion-accepted absolute
                # observations, because its job is to keep a usable global pose
                # available when T265 continuity is lost.  A candidate must
                # still never promote itself to a trusted anchor -- that is
                # governed by _d500_global_alignment_trusted and
                # _last_d500_absolute_update_s, which only the fusion-accepted
                # branch sets.
                if observation.map_pose_valid and observation.map_pose is not None:
                    fallback_quality = PoseQuality(
                        "d500_global_fallback",
                        True,
                        False,
                        position_confidence=confidence,
                        heading_confidence=confidence,
                        age_s=max(0.0, now - observation.timestamp_s),
                    )
                    self.fusion.update_d500_global_fallback(
                        observation.map_pose,
                        fallback_quality,
                        map_alignment_valid=(
                            observation.map_alignment_established
                            and self._d500_global_alignment_trusted
                        ),
                    )
                self._emit(
                    "d500_localization_status",
                    d500_map_alignment_established=observation.map_alignment_established,
                    d500_map_pose_valid=observation.map_pose_valid,
                    d500_absolute_observation_available=observation.absolute_observation_available,
                    d500_absolute_observation_accepted=absolute_correction_accepted,
                    d500_last_absolute_age_s=None if self._last_d500_absolute_update_s is None else max(0.0, now - self._last_d500_absolute_update_s),
                    d500_last_map_pose_age_s=None if self._last_d500_map_pose_update_s is None else max(0.0, now - self._last_d500_map_pose_update_s),
                )

    def on_d500_update(self, update) -> None:
        """Thread-safe callback passed to the real D500 component."""

        received_s = float(self.clock())
        scan = getattr(update, "scan", None)
        timestamp_s = self._map_d500_timestamp(
            getattr(scan, "timestamp_ms", None), received_s
        )
        if scan is not None and self.slam_bridge is not None:
            try:
                self.slam_bridge.push_d500_scan(scan, timestamp_s)
            except Exception:
                LOG.exception("could not queue D500 scan for SLAM")

        # A rejected scan-to-scan ICP only invalidates the local delta; it must
        # not discard the scan.  A complete scan can still carry a usable
        # absolute boundary observation, which is exactly the case that matters
        # when the car rotates in place and ICP rejects.
        local_valid = bool(
            getattr(update, "local_pose_valid", update.odometry.accepted)
        )
        absolute_available = bool(
            getattr(update, "absolute_observation_available", False)
        )
        if not local_valid and not absolute_available:
            self._emit(
                "d500_rejected",
                reason=update.odometry.rejection_reason or "odometry_rejected",
                d500_mode="LOST",
                priority=True,
            )
            return
        local_pose = self.radar_adapter.to_map_base_pose(update.odometry.pose, timestamp_s=timestamp_s)
        confidence = update.global_confidence
        map_pose = None
        if update.map_pose_valid and update.global_pose is not None:
            map_pose = self.radar_adapter.to_map_base_pose(update.global_pose, timestamp_s=timestamp_s)
        mode = D500LocalizationMode.GLOBAL if update.map_pose_valid else D500LocalizationMode.LOCAL_ONLY
        self._d500_events.put(
            D500PoseObservation(
                local_pose,
                map_pose,
                mode,
                update.map_alignment_established,
                update.map_pose_valid,
                update.absolute_observation_available,
                update.absolute_observation_accepted,
                confidence,
                timestamp_s,
            )
        )
        icp = update.odometry.icp
        diagnostic_pose = update.global_pose if update.map_pose_valid and update.global_pose is not None else update.odometry.pose
        self._emit(
            "d500_diagnostic",
            raw_pose_cm=(diagnostic_pose.x_cm, diagnostic_pose.y_cm),
            raw_yaw_cw_deg=diagnostic_pose.yaw_cw_deg,
            icp_mean_error_cm=None if icp is None else icp.mean_error_cm,
            icp_matched_points=None if icp is None else icp.matched_points,
            icp_iterations=None if icp is None else icp.iterations,
            wall_fusion_status=None if update.wall_fusion is None else update.wall_fusion.status.value,
            d500_candidate_mode=mode.value.upper(),
            d500_map_alignment_established=update.map_alignment_established,
            d500_map_pose_valid=update.map_pose_valid,
            d500_absolute_observation_available=update.absolute_observation_available,
            d500_absolute_observation_accepted=update.absolute_observation_accepted,
            global_confidence=confidence,
        )

    def _map_d500_timestamp(self, timestamp_ms, received_s: float) -> float:
        """Map the D500's wrapping uint16 millisecond clock to monotonic time."""
        return self._d500_clock.map_milliseconds(timestamp_ms, received_s)

    def _consume_slam(self, now_s: float) -> None:
        bridge = self.slam_bridge
        if bridge is None:
            return
        anchor = bridge.latest_slam_anchor()
        if (anchor is not None and anchor.is_finite()
                and anchor.timestamp_s != self._last_slam_anchor_timestamp_s):
            self._last_slam_anchor_timestamp_s = anchor.timestamp_s
            if self.config.localization.backend == "slam_toolbox" and anchor.age_s(now_s) <= 0.5:
                self.fusion.update_slam_anchor(
                    anchor.pose, valid=True, timestamp_s=anchor.timestamp_s,
                    loop_closure=anchor.loop_closure,
                    source_timestamp_s=anchor.source_timestamp_s,
                )
                anchor_estimate = self.fusion.estimate(now_s)
                anchor_diagnostics = self.fusion.slam_anchor_diagnostics(now_s)
                self._emit(
                    "slam_anchor",
                    x_m=anchor.pose.x_m, y_m=anchor.pose.y_m,
                    yaw_rad=anchor.pose.yaw_rad,
                    source_timestamp_s=anchor.source_timestamp_s,
                    received_timestamp_s=anchor.timestamp_s,
                    source_age_s=(None if anchor.source_timestamp_s is None
                                  else now_s - anchor.source_timestamp_s),
                    accepted=anchor_estimate.d500_accepted,
                    rejection_reason=anchor_estimate.rejection_reason,
                    consensus_pending=anchor_estimate.slam_consensus_pending,
                    **anchor_diagnostics,
                )
        if self._last_slam_metrics_s is None or now_s - self._last_slam_metrics_s >= 1.0:
            metrics = bridge.metrics()
            previous = self._last_slam_metrics or metrics
            elapsed = max(1.0, now_s - self._last_slam_metrics_s) if self._last_slam_metrics_s is not None else 1.0
            self._emit(
                "slam_status", state=bridge.state(now_s),
                anchor_age_ms=None if anchor is None else anchor.age_s(now_s) * 1000.0,
                scan_input_hz=(metrics["slam.scan_input_count"] - previous["slam.scan_input_count"]) / elapsed,
                scan_publish_hz=(metrics["slam.scan_publish_count"] - previous["slam.scan_publish_count"]) / elapsed,
                anchor_hz=(metrics["slam.pose_received_count"] - previous["slam.pose_received_count"]) / elapsed,
                **metrics,
            )
            self._last_slam_metrics_s = now_s
            self._last_slam_metrics = metrics

    def _consume_replay(self, now_s: float) -> None:
        if self._start_time_s is None:
            return
        replay_t = max(0.0, now_s - self._start_time_s)
        while self._replay_index < len(self._replay_events):
            event = self._replay_events[self._replay_index]
            if event.t_s > replay_t + 1e-9:
                break
            pose = Pose2D(event.pose.x_m, event.pose.y_m, event.pose.yaw_rad, self._start_time_s + event.t_s)
            if event.source == "t265":
                self.fusion.update_t265(pose, event.quality)
                self._emit("t265_pose", x_m=pose.x_m, y_m=pose.y_m, yaw_rad=pose.yaw_rad, replay=True)
            elif event.source == "d500_absolute":
                self.fusion.update_d500_absolute(pose, event.quality)
                self._d500_global_alignment_trusted = (
                    self._d500_global_alignment_trusted or event.map_alignment_valid
                )
            else:
                self.fusion.update_d500_global_fallback(
                    pose,
                    event.quality,
                    map_alignment_valid=(
                        self._d500_global_alignment_trusted and event.map_alignment_valid
                    ),
                )
                self._emit(
                    "d500_pose", x_m=pose.x_m, y_m=pose.y_m,
                    yaw_rad=pose.yaw_rad, d500_mode="GLOBAL_PREDICTED", replay=True,
                )
            self._replay_index += 1

    def record_event(self, event_type: str, **fields) -> None:
        """Record an upper-level mission event using the runtime's safe sink."""
        self._current_step_s = float(self.clock())
        self._emit(event_type, priority=True, **fields)

    def _emit(self, event_type: str, *, priority: bool = False, **fields) -> None:
        if self.event_logger is None:
            return
        try:
            event_time = self._current_step_s if self._current_step_s is not None else float(self.clock())
            relative_t = 0.0 if self._start_time_s is None else max(0.0, event_time - self._start_time_s)
            self.event_logger.emit({"t": relative_t, "type": event_type, **fields}, priority=priority)
        except Exception:
            # A diagnostics sink is never allowed to break the safety loop.
            pass

    def _safe_stop_drive(self) -> None:
        if self.drive.is_running:
            try:
                self.drive.stop()
            except Exception:
                LOG.exception("failed to stop differential drive")



    def _ensure_drive_started_and_get_time(self, now_s: float) -> float:
        """Start lazily and return a timestamp no earlier than drive startup."""

        if self.drive.is_running:
            return now_s
        self.drive.start()
        return float(self.clock())

    def _handle_localization(self, estimate: FusedPoseEstimate, now_s: float) -> bool:
        """Return whether navigation is suspended during a timed pose dropout."""

        if estimate.pose is not None:
            pose_values = (estimate.pose.x_m, estimate.pose.y_m, estimate.pose.yaw_rad)
            if not all(math.isfinite(value) for value in pose_values):
                raise ValueError("localization pose contains a non-finite value")
        pose_lost = estimate.pose is None or estimate.state in {
            PoseFusionState.LOST,
            PoseFusionState.UNANCHORED,
        }
        if not pose_lost:
            self._pose_lost_since_s = None
            self.mission.on_localization(estimate)
            return False

        if self._pose_lost_since_s is None:
            expiry_times = []
            if estimate.t265_age_s is not None:
                expiry_times.append(now_s - estimate.t265_age_s + self.config.fusion.t265_max_age_s)
            if estimate.d500_age_s is not None:
                expiry_times.append(now_s - estimate.d500_age_s + self.config.fusion.d500_max_age_s)
            self._pose_lost_since_s = min(now_s, max(expiry_times)) if expiry_times else now_s
        if self.mission.state in {RobocupMissionState.NAVIGATING, RobocupMissionState.RETURNING}:
            elapsed = max(0.0, now_s - self._pose_lost_since_s)
            if elapsed >= self.config.safety.stop_if_pose_lost_s:
                self.mission.request_safe_stop("localization lost beyond configured timeout")
                return False
            return True
        self.mission.on_localization(estimate)
        return False

    def _hardware_global_localization_ready(self, now_s: float) -> bool:
        if self.config.localization.backend == "slam_toolbox":
            if self.mode is not RuntimeMode.HARDWARE_MISSION:
                return True
            accepted_age = self.fusion.slam_anchor_age_s(now_s)
            bridge_ok = self.slam_bridge is not None and self.slam_bridge.state(now_s) == "SLAM_OK"
            if self.motion.state is not MotionActionState.RUNNING:
                self._slam_action_started_with_fresh_anchor = False
            elif bridge_ok and accepted_age is not None and accepted_age <= 0.5:
                self._slam_action_started_with_fresh_anchor = True
            if (not bridge_ok or accepted_age is None or
                    (accepted_age > 0.5 and not (
                        self._slam_action_started_with_fresh_anchor
                        and self.fusion.slam_consensus_pending(now_s)))):
                return False
            if self.config.localization.slam.require_field_anchor:
                return (
                    self.fusion.field_T_slam_map is not None
                    and self._last_d500_absolute_update_s is not None
                    and now_s - self._last_d500_absolute_update_s <= D500_ANCHOR_STALE_AFTER_S
                )
            return True
        if (
            self.mode is not RuntimeMode.HARDWARE_MISSION
            or not self.config.d500.enabled
            or not self.config.d500_localization.require_global_for_hardware
        ):
            return True
        # Readiness needs a *trusted anchor*, which is only ever set after
        # PoseFusion accepted an absolute observation, and that anchor must not
        # be so old that the map can no longer be trusted.  This deliberately
        # does not use the short d500_max_age_s measurement timeout.
        return (
            self._d500_global_alignment_trusted
            and self._last_d500_absolute_update_s is not None
            and now_s - self._last_d500_absolute_update_s <= D500_ANCHOR_STALE_AFTER_S
        )

    @staticmethod
    def _stop_source(source) -> None:
        try:
            if hasattr(source, "stop"):
                source.stop()
            elif hasattr(source, "close"):
                source.close()
        except Exception:
            LOG.exception("failed to stop sensor source")


def build_runtime(
    config: DifferentialRobotConfig,
    mode: RuntimeMode,
    *,
    replay_file: str | None = None,
    mission_profile: str = "default",
    clock: Callable[[], float] = time.monotonic,
    fake_sample_count: int = 32,
    event_logger: JsonlEventLogger | None = None,
    sensor_only: bool = False,
) -> RobocupRuntime:
    """Build all runtime dependencies without starting a device."""

    if sensor_only and mode is not RuntimeMode.HARDWARE_PROBE:
        raise ValueError("sensor_only requires hardware-probe mode")
    readiness = validate_runtime_readiness(config, mode)
    if readiness:
        raise RuntimeReadinessError("; ".join(readiness))
    replay_events = read_pose_events(replay_file) if mode is RuntimeMode.REPLAY and replay_file else ()
    fake_mode = mode in {RuntimeMode.DRY_RUN, RuntimeMode.REPLAY}
    relative_slam_scans = (
        not fake_mode and config.d500.enabled
        and config.localization.backend == "slam_toolbox"
        and not config.localization.slam.require_field_anchor
    )
    now = float(clock())
    period_s = 0.05

    drive = build_differential_drive(config, fake=fake_mode or sensor_only, clock=clock)
    fusion = build_pose_fusion(config)
    navigator = build_differential_navigator(config)
    motion = build_basic_motion_controller(config, navigator)
    slam_bridge = None
    if config.localization.slam.enabled and not fake_mode:
        from components.slam_bridge import SlamBridge

        slam_bridge = SlamBridge(config.d500_mount)
    t265_adapter = T265PoseAdapter(
        config.t265_mount,
        min_tracker_confidence=config.fusion.t265_min_tracker_confidence,
        max_age_s=config.fusion.t265_max_age_s,
    )
    radar_adapter = RadarPoseAdapter()

    if mode is RuntimeMode.REPLAY and replay_file:
        t265_source = None
        d500_source = None
        d500_fake = False
    elif fake_mode:
        t265_samples = [
            T265RawPose(
                translation_xyz=(0.0, 0.0, 0.0),
                quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
                velocity_xyz=(0.0, 0.0, 0.0),
                angular_velocity_xyz=(0.0, 0.0, 0.0),
                tracker_confidence=3,
                mapper_confidence=0,
                device_timestamp_ms=(now + index * period_s) * 1000.0,
                received_monotonic_s=now + index * period_s,
                measurement_monotonic_s=now + index * period_s,
            )
            for index in range(max(1, fake_sample_count))
        ]
        t265_source = FakeT265PoseSource(t265_samples) if config.t265.enabled else None
        d500_samples = [
            FakeD500PoseSample(0.0, 0.0, 0.0, now + index * period_s)
            for index in range(max(1, fake_sample_count))
        ]
        d500_source = FakeD500PoseSource(d500_samples) if config.d500.enabled else None
        d500_fake = True
    else:
        t265_source = RealSenseT265PoseSource(config.t265.serial) if config.t265.enabled else None
        if config.d500.enabled and not relative_slam_scans:
            from components.radar_driver import (
                D500RadarComponent,
                DroneGlobalAlignment,
                GlobalCorrectionMode,
                RadarMount,
                WallFusionConfig,
                WallLineConfig,
                WallLineLocalizer,
            )
            from localization.field_reference import build_field_wall_reference

            mount = RadarMount(
                x_forward_cm=config.d500_mount.x_m * 100.0,
                y_left_cm=config.d500_mount.y_m * 100.0,
                yaw_cw_deg=-math.degrees(config.d500_mount.yaw_rad),
            )
            d500_source = D500RadarComponent(
                port=config.d500.port,
                baudrate=config.d500.baudrate,
                mount=mount,
                alignment=DroneGlobalAlignment(0.0, 0.0, 0.0) if config.d500_localization.enable_wall_absolute else None,
                on_update=None,
                global_correction_mode=GlobalCorrectionMode.UPDATE_ALIGNMENT,
            )
            if config.d500_localization.enable_wall_absolute:
                d500_source.enable_wall_fusion(
                    build_field_wall_reference(config.d500_localization),
                    line_config=WallLineConfig(),
                    fusion_config=WallFusionConfig.car_slow_drift(),
                )
        else:
            d500_source = None
        d500_fake = False

    constraints = runtime_constraints(config, mode)
    runtime = RobocupRuntime(
        config,
        mode,
        drive=drive,
        d500_source=d500_source,
        t265_source=t265_source,
        d500_fake=d500_fake,
        t265_adapter=t265_adapter,
        radar_adapter=radar_adapter,
        fusion=fusion,
        navigator=navigator,
        motion=motion,
        clock=clock,
        mission_profile=mission_profile,
        constraints=constraints,
        event_logger=event_logger,
        replay_events=tuple(replay_events),
        slam_bridge=slam_bridge,
    )
    if relative_slam_scans:
        from components.slam_scan_source import ScanOnlyD500Source

        runtime.d500_source = ScanOnlyD500Source(
            runtime, config.d500.port, config.d500.baudrate,
        )
    elif not d500_fake and d500_source is not None:
        d500_source.on_update = runtime.on_d500_update
    return runtime


def load_runtime_config(path: str | None = None) -> DifferentialRobotConfig:
    return load_v2_config(path)


# ===========================================================================
# Wall-following runtime
#
# The competition chain does NOT use the fused runtime above. This thin runtime
# composes one raw-scan wall observer, one wall-driven motion controller and the
# existing ``DifferentialDrive``. It deliberately constructs none of
# RealSenseT265PoseSource, T265PoseAdapter, PoseFusion, SlamBridge,
# D500RadarComponent, RadarOdometry, ICPScanMatcher, the map anchor or
# DifferentialNavigator/BasicMotionController: a competition run must import and
# build with no ROS, no pyrealsense2 and no SLAM workspace at all.
# ===========================================================================


class WallRuntimeReadinessError(RuntimeError):
    """The configured robot is not ready for wall-following motion."""


@dataclass(frozen=True, slots=True)
class WallStep:
    """One control tick of the wall runtime; there is deliberately no estimate."""

    now_s: float
    walls: "WallSnapshot | None"
    phase: str
    command: Twist2D
    action_state: str
    limited_command: Twist2D | None = None
    error: str | None = None


class WallMission:
    """Small mission-state holder; carries no navigator and no motion object."""

    def __init__(self) -> None:
        self.state = RobocupMissionState.INIT
        self.last_error: str | None = None
        self.task_counts = None
        self.phase = "idle"

    def set_task_counts(self, counts) -> None:
        if self.task_counts is not None:
            raise RuntimeError("task board result already set")
        self.task_counts = counts

    def finish(self) -> None:
        self.state = RobocupMissionState.FINISHED

    def on_payload_action_start(self) -> None:
        self.state = RobocupMissionState.TARGET_OPERATION

    def on_payload_action_done(self) -> None:
        if self.state is RobocupMissionState.TARGET_OPERATION:
            self.state = RobocupMissionState.READY

    def request_safe_stop(self, reason: str = "operator requested safe stop") -> None:
        self.last_error = reason
        self.state = RobocupMissionState.SAFE_STOP

    def request_error(self, reason: str) -> None:
        self.last_error = reason
        self.state = RobocupMissionState.ERROR


def validate_wall_runtime_config(config: DifferentialRobotConfig,
                                 mode: RuntimeMode) -> list[str]:
    """Narrow wall-motion preflight; deliberately not the fused readiness gate.

    It keeps the existing chassis checks that still matter (a differential
    protocol, usable limits and a configured D500 port) and drops every
    requirement about map anchors, T265, SLAM services and the measured field
    reference, because none of those participate in wall following.
    """
    errors: list[str] = []
    if mode is RuntimeMode.HARDWARE_MISSION:
        if config.drive.kinematics != "differential":
            errors.append("wall runtime requires differential kinematics")
        if not config.d500.enabled or not config.d500.port:
            errors.append("wall runtime requires the D500 lidar to be enabled with a port")
        if (config.drive.protocol_mode == "differential_vx_vz"
                and config.safety.require_verified_c10b_diff_firmware_for_curved_motion
                and not config.calibration.c10b_diff_firmware_verified):
            errors.append(
                "differential_vx_vz requires verified C10B differential firmware; "
                "select ackermann_firmware_compat for the compatibility backend")
    return errors


class WallRuntime:
    """Observation -> wall action -> one differential drive command.

    The caller owns the action sequence. Every non-zero command leaves this
    object through one place, under one command lock, after a final check of the
    irreversible stop latch, so a STOP that arrives between "compute" and "send"
    cannot let a stale command reach the motors.
    """

    def __init__(self, config: DifferentialRobotConfig, mode: RuntimeMode, *,
                 drive: DifferentialDrive,
                 observer,
                 clock: Callable[[], float] = time.monotonic,
                 event_logger: JsonlEventLogger | None = None,
                 limits=None) -> None:
        from components.wall_navigation import WallActionLimits

        self.config = config
        self.mode = mode
        self.drive = drive
        self.observer = observer
        self.clock = clock
        self.event_logger = event_logger
        self.limits = limits or WallActionLimits.from_config(config)
        self.mission = WallMission()
        self.action = None
        self._command_lock = threading.RLock()
        self._stop_latched = False
        self._stop_reason: str | None = None
        self._started = False
        self._closed = False
        self._start_time_s: float | None = None
        self._drive_started = False

    # ---- observability ---------------------------------------------------
    @property
    def is_running(self) -> bool:
        return self._started and not self._closed

    @property
    def stop_latched(self) -> bool:
        return self._stop_latched

    @property
    def stop_reason(self) -> str | None:
        return self._stop_reason

    def record_event(self, event_type: str, **fields) -> None:
        if self.event_logger is None:
            return
        try:
            event_time = float(self.clock())
            relative_t = 0.0 if self._start_time_s is None else max(0.0, event_time - self._start_time_s)
            self.event_logger.emit({"t": relative_t, "type": event_type, **fields},
                                   priority=True)
        except Exception:
            LOG.exception("wall runtime event logging failed")

    # ---- lifecycle -------------------------------------------------------
    def start(self) -> None:
        if self._closed:
            raise RuntimeError("WallRuntime is closed; build a new one for the next RUN")
        if self._started:
            return
        # A new run must not inherit the previous run's scans, wall reference or
        # stop latch.
        self._stop_latched = False
        self._stop_reason = None
        self.action = None
        self.mission = WallMission()
        self.observer.start()
        self._started = True
        self._start_time_s = float(self.clock())
        self.record_event("wall_runtime_started", mode=self.mode.value)

    def _ensure_drive_started(self, now_s: float) -> float:
        if not self.drive.is_running:
            self.drive.start()
            self._drive_started = True
        return now_s

    def request_stop(self, reason: str = "operator requested safe stop") -> None:
        """Irreversible per-run stop: latch, cancel the action, zero the output."""
        with self._command_lock:
            self._stop_latched = True
            self._stop_reason = reason
            self.action = None
            self.mission.request_safe_stop(reason)
            try:
                self.drive.stop()
            except Exception:
                LOG.exception("wall runtime drive stop failed")
        self.record_event("wall_runtime_stop", reason=reason)

    def reset_action(self) -> None:
        self.action = None

    def close(self, *, timeout_s: float = 2.0) -> None:
        # Permanently forbid further non-zero output for this run, then release
        # owned resources. One failing resource must not skip the others.
        with self._command_lock:
            self._stop_latched = True
            self.action = None
        errors = []
        for name, cleanup in (("drive_stop", self.drive.stop),
                              ("observer_close", lambda: self.observer.close(timeout_s=timeout_s)),
                              ("drive_close", self.drive.close),
                              ("logger_close", self._close_logger)):
            try:
                cleanup()
            except Exception as exc:
                errors.append(f"{name}: {exc}")
                LOG.exception("wall runtime cleanup failed: %s", name)
        self._closed = True
        self._started = False
        if errors:
            self.record_event("wall_runtime_cleanup_failed", errors=errors)

    def _close_logger(self) -> None:
        """Drain and close the async event log; it owns its own writer thread."""
        if self.event_logger is not None and hasattr(self.event_logger, "close"):
            self.event_logger.close()

    # ---- control ---------------------------------------------------------
    def step(self) -> WallStep:
        now_s = float(self.clock())
        if self._stop_latched:
            return self._idle_step(now_s, "stopped", self._stop_reason)
        if self.mission.state in {RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR}:
            return self._idle_step(now_s, "stopped", self.mission.last_error)
        snapshot = self.observer.latest()
        action = self.action
        if action is None:
            return self._idle_step(now_s, "idle", None)
        try:
            outcome = action.step(snapshot, now_s=now_s)
        except Exception as exc:
            self.mission.request_error(str(exc))
            self.request_stop(f"wall action failed: {exc}")
            return self._idle_step(now_s, "error", str(exc))
        command = outcome.command
        fields = {key: value for key, value in outcome.fields.items()
                  if key not in {"command", "right_wall_normal"}}
        self.record_event("wall_control", action=action.name, phase=self.mission.phase,
                          done=outcome.done, reason=outcome.reason, **fields)
        limited = self._send(command, now_s)
        # ``error`` is for a real failure only. An ordinary "waiting for a fresh
        # wall" outcome belongs in the action state: reporting it as an error
        # would abort exactly the action that is supposed to keep running.
        if outcome.done:
            state = "done"
        elif outcome.reason:
            state = "waiting"
        else:
            state = "running"
        return WallStep(now_s, snapshot, self.mission.phase, command, state,
                        limited_command=limited, error=None)

    def _idle_step(self, now_s: float, phase: str, error: str | None) -> WallStep:
        """A tick with no action: go through the same zero-output gate.

        The idle, stopped and error branches must not bypass the single output
        gate, or a earlier non-zero command could stay latched in the drive while
        the runtime reports that it is doing nothing.
        """
        self._send(Twist2D(0.0, 0.0), now_s)
        return WallStep(now_s, self.observer.latest(), phase, Twist2D(0.0, 0.0), phase,
                        limited_command=Twist2D(0.0, 0.0), error=error)

    def _send(self, command: Twist2D, now_s: float) -> Twist2D | None:
        """Single output gate. Re-checks the stop latch inside the lock."""
        with self._command_lock:
            if self._stop_latched:
                return None
            if command.linear_x_m_s == 0.0 and command.angular_z_rad_s == 0.0:
                if self.drive.is_running:
                    self.drive.stop()
                return Twist2D(0.0, 0.0)
            drive_now = self._ensure_drive_started(now_s)
            self.drive.command(command, now_s=drive_now)
            return self.drive.last_limited_twist


def build_wall_runtime(config: DifferentialRobotConfig, mode: RuntimeMode, *,
                       clock: Callable[[], float] = time.monotonic,
                       event_logger: JsonlEventLogger | None = None,
                       observer=None, drive=None,
                       limits=None) -> WallRuntime:
    """Compose the wall-following competition runtime without starting a device."""
    if mode is RuntimeMode.HARDWARE_PROBE:
        raise ValueError("the wall runtime never runs a hardware probe")
    errors = validate_wall_runtime_config(config, mode)
    if errors:
        raise WallRuntimeReadinessError("; ".join(errors))
    if drive is None:
        drive = build_differential_drive(config, fake=mode is not RuntimeMode.HARDWARE_MISSION,
                                        clock=clock)
    if observer is None:
        from components.wall_observer import WallObserver
        from components.radar_driver import RadarMount

        mount = RadarMount(
            x_forward_cm=config.d500_mount.x_m * 100.0,
            y_left_cm=config.d500_mount.y_m * 100.0,
            yaw_cw_deg=-math.degrees(config.d500_mount.yaw_rad),
        )
        observer = WallObserver(port=config.d500.port, baudrate=config.d500.baudrate,
                               mount=mount, clock=clock)
    return WallRuntime(config, mode, drive=drive, observer=observer, clock=clock,
                       event_logger=event_logger, limits=limits)
