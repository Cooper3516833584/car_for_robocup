"""Composition root and safe event loop for the RoboCup differential platform."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import logging
import math
import queue
import time
from typing import Callable, Iterable

from components.differential_drive import DifferentialDrive
from components.differential_navigation import DifferentialNavigator, NavigationOutput, NavigationState
from components.diagnostics_log import JsonlEventLogger
from components.navigation_common import NavigationGoal, NavigationGrid
from components.pose_fusion import FusedPoseEstimate, PoseFusion, PoseFusionState
from components.pose_log_replay import PoseLogEvent, read_pose_events
from components.radar_pose_adapter import RadarPoseAdapter
from components.t265_driver import FakeT265PoseSource, RealSenseT265PoseSource, T265RawPose
from components.t265_pose_adapter import T265PoseAdapter
from config.v2_factory import (
    build_competition_world,
    build_differential_drive,
    build_differential_navigator,
    build_pose_fusion,
)
from config.v2_loader import load_v2_config
from config.v2_models import DifferentialRobotConfig
from config.v2_runtime import RuntimeConstraints, RuntimeMode, runtime_constraints, validate_runtime_readiness
from core.types import Pose2D, PoseQuality, Twist2D

LOG = logging.getLogger("robocup-runtime")


class RuntimeReadinessError(RuntimeError):
    """The configured robot is not ready for the requested runtime mode."""


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


@dataclass(frozen=True, slots=True)
class FakeD500PoseSample:
    x_cm: float
    y_cm: float
    yaw_cw_deg: float
    timestamp_s: float


@dataclass(frozen=True, slots=True)
class D500PoseObservation:
    local_pose: Pose2D
    global_pose: Pose2D | None
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

    def __init__(self, navigator: DifferentialNavigator, profile_name: str = "default") -> None:
        self.navigator = navigator
        self.profile_name = profile_name
        self.state = RobocupMissionState.INIT
        self._goal_pending = False
        self.last_error: str | None = None

    def set_navigation_goal(self, goal: NavigationGoal) -> None:
        if self.state in {RobocupMissionState.FINISHED, RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR}:
            raise RuntimeError(f"cannot set a goal while mission is {self.state.value}")
        self.navigator.set_goal(goal)
        self._goal_pending = True
        if self.state is RobocupMissionState.READY:
            self.state = RobocupMissionState.NAVIGATING

    def on_localization(self, estimate: FusedPoseEstimate) -> None:
        if self.state in {RobocupMissionState.FINISHED, RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR}:
            return
        if estimate.pose is None or estimate.state in {PoseFusionState.LOST, PoseFusionState.UNANCHORED}:
            if self.state in {RobocupMissionState.NAVIGATING, RobocupMissionState.RETURNING}:
                self.request_safe_stop("localization lost during motion")
            else:
                self.state = RobocupMissionState.WAIT_FOR_LOCALIZATION
            return
        if self.state in {RobocupMissionState.INIT, RobocupMissionState.WAIT_FOR_LOCALIZATION}:
            self.state = RobocupMissionState.NAVIGATING if self._goal_pending else RobocupMissionState.READY

    def on_goal_reached(self) -> None:
        if self.state in {RobocupMissionState.NAVIGATING, RobocupMissionState.RETURNING}:
            self._goal_pending = False
            self.state = RobocupMissionState.TARGET_OPERATION

    def on_target_detected(self) -> None:
        if self.state is RobocupMissionState.NAVIGATING:
            self.state = RobocupMissionState.TARGET_OPERATION

    def on_payload_action_done(self) -> None:
        if self.state is RobocupMissionState.TARGET_OPERATION:
            self.state = RobocupMissionState.READY

    def set_return_goal(self, goal: NavigationGoal) -> None:
        self.navigator.set_goal(goal)
        self._goal_pending = True
        self.state = RobocupMissionState.RETURNING

    def finish(self) -> None:
        self.state = RobocupMissionState.FINISHED

    def request_safe_stop(self, reason: str = "operator requested safe stop") -> None:
        self.last_error = reason
        self.state = RobocupMissionState.SAFE_STOP

    def request_error(self, reason: str) -> None:
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
        clock: Callable[[], float],
        mission_profile: str = "default",
        constraints: RuntimeConstraints | None = None,
        world: NavigationGrid | None = None,
        event_logger: JsonlEventLogger | None = None,
        replay_events: tuple[PoseLogEvent, ...] = (),
    ) -> None:
        self.config = config
        self.mode = mode
        self.drive = drive
        self.d500_source = d500_source
        self.t265_source = t265_source
        self.d500_fake = d500_fake
        self.t265_adapter = t265_adapter
        self.radar_adapter = radar_adapter
        self.fusion = fusion
        self.navigator = navigator
        self.mission = RobocupMission(navigator, mission_profile)
        self.clock = clock
        self.constraints = constraints or runtime_constraints(config, mode)
        self.world = world
        self.event_logger = event_logger
        self._replay_events = replay_events
        self._replay_index = 0
        self._start_time_s: float | None = None
        self._current_step_s: float | None = None
        self._last_safety_state: RobocupMissionState | None = None
        self._last_watchdog_stop_count = drive.watchdog_stop_count
        self._d500_events: queue.SimpleQueue[D500PoseObservation] = queue.SimpleQueue()
        self.d500_abs_accept_count = 0
        self.d500_abs_reject_low_confidence = 0
        self.d500_abs_reject_position_gate = 0
        self.d500_abs_reject_yaw_gate = 0
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
        now = float(self.clock() if now_s is None else now_s)
        if not math.isfinite(now):
            raise ValueError("runtime clock must be finite monotonic time")
        self._current_step_s = now
        watchdog_stops = self.drive.watchdog_stop_count
        if watchdog_stops > self._last_watchdog_stop_count:
            self._emit("safety", event_code="WATCHDOG_STOP", count=watchdog_stops - self._last_watchdog_stop_count, priority=True)
            self._last_watchdog_stop_count = watchdog_stops
        navigation_output: NavigationOutput | None = None
        try:
            self._consume_t265(now)
            self._consume_d500(now)
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
                d500_accepted=estimate.d500_accepted,
                d500_innovation_m=estimate.last_d500_innovation_m,
                d500_innovation_yaw_rad=estimate.last_d500_innovation_yaw_rad,
                rejection_reason=estimate.rejection_reason,
            )
            self.mission.on_localization(estimate)

            command = Twist2D(0.0, 0.0)
            if self.mode is not RuntimeMode.HARDWARE_PROBE and self.mission.state in {
                RobocupMissionState.NAVIGATING,
                RobocupMissionState.RETURNING,
            }:
                navigation_output = self.navigator.step(
                    estimate.pose,
                    self.world,
                    now_s=now,
                    pose_state=estimate.state,
                )
                command = navigation_output.command
                if navigation_output.state is NavigationState.GOAL_REACHED:
                    self.mission.on_goal_reached()
                    command = Twist2D(0.0, 0.0)
                elif navigation_output.state in {NavigationState.BLOCKED, NavigationState.POSE_LOST, NavigationState.ERROR}:
                    self.mission.request_safe_stop(
                        str(navigation_output.diagnostics.get("reason", navigation_output.state.value))
                    )
                    command = Twist2D(0.0, 0.0)
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
            } or estimate.state is PoseFusionState.LOST:
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
            elif self.mode is RuntimeMode.HARDWARE_PROBE:
                command = Twist2D(0.0, 0.0)
            elif command.linear_x_m_s != 0.0 or command.angular_z_rad_s != 0.0:
                drive_now = self._ensure_drive_started_and_get_time(now)
                self.drive.command(command, now_s=drive_now)
            elif self.drive.is_running:
                self.drive.stop()

            limited = self.drive.last_limited_twist
            wheels = self.drive.kinematics.twist_to_wheels(limited)
            self._emit(
                "drive_command",
                requested_v_m_s=command.linear_x_m_s,
                requested_omega_rad_s=command.angular_z_rad_s,
                limited_v_m_s=limited.linear_x_m_s,
                limited_omega_rad_s=limited.angular_z_rad_s,
                left_target_m_s=wheels.left_m_s,
                right_target_m_s=wheels.right_m_s,
                protocol_mode=self.config.drive.protocol_mode,
                actuation_enabled=self.drive.is_running,
            )

            return RuntimeStep(now, estimate, self.mission.state, command, navigation_output)
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            self.mission.request_error(self._error)
            self._safe_stop_drive()
            if type(exc).__name__ in {"UnsupportedFirmwareMotion", "UnsupportedWheelCommand"} or "turn radius" in str(exc).lower():
                self._emit("c10b_reject", event_code="C10B_REJECT", reason=self._error, priority=True)
            self._emit("runtime_error", error=self._error, priority=True)
            LOG.exception("RoboCup runtime step failed")
            estimate = self.fusion.estimate(now)
            return RuntimeStep(now, estimate, self.mission.state, Twist2D(0.0, 0.0), navigation_output, self._error)

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

    def _consume_t265(self, now_s: float) -> None:
        if self.t265_source is None:
            return
        raw = self.t265_source.read()
        if raw is None:
            return
        update = self.t265_adapter.adapt(raw, now_s=now_s)
        if update.pose is not None and update.quality.valid:
            self.fusion.update_t265(update.pose, update.quality)
            self._emit(
                "t265_pose",
                x_m=update.pose.x_m,
                y_m=update.pose.y_m,
                yaw_rad=update.pose.yaw_rad,
                confidence=update.quality.position_confidence,
                age_s=update.quality.age_s,
                raw_translation_xyz=raw.translation_xyz,
                raw_quaternion_xyzw=raw.quaternion_xyzw,
                tracker_confidence=raw.tracker_confidence,
                mapper_confidence=raw.mapper_confidence,
            )
        else:
            self._emit(
                "t265_rejected",
                reason=update.reason or "invalid",
                event_code="LOW_T265_CONFIDENCE" if update.reason == "tracker_confidence_too_low" else "T265_REJECT",
                confidence=raw.tracker_confidence,
                raw_translation_xyz=raw.translation_xyz,
                raw_quaternion_xyzw=raw.quaternion_xyzw,
                mapper_confidence=raw.mapper_confidence,
            )

    def _consume_d500(self, now_s: float) -> None:
        if self.d500_fake:
            if self.d500_source is None:
                return
            sample = self.d500_source.read()
            if sample is not None:
                pose = self.radar_adapter.to_map_base_pose(sample, timestamp_s=sample.timestamp_s)
                self.fusion.update_d500(pose, PoseQuality("d500", True, False, age_s=max(0.0, now_s - sample.timestamp_s)))
                self._emit("d500_pose", x_m=pose.x_m, y_m=pose.y_m, yaw_rad=pose.yaw_rad)
        else:
            while True:
                try:
                    observation = self._d500_events.get_nowait()
                except queue.Empty:
                    break
                mode = "LOCAL_ONLY"
                pose = observation.local_pose
                confidence = None
                if observation.global_pose is not None:
                    confidence = observation.confidence
                    if confidence is None or confidence < self.config.d500_localization.min_confidence:
                        self.d500_abs_reject_low_confidence += 1
                        self._emit("d500_abs_rejected", reason="low_confidence", confidence=confidence)
                    else:
                        prior = self.fusion.estimate(now_s).pose
                        if prior is not None:
                            position_jump = math.hypot(observation.global_pose.x_m - prior.x_m, observation.global_pose.y_m - prior.y_m)
                            yaw_jump = abs((observation.global_pose.yaw_rad - prior.yaw_rad + math.pi) % (2.0 * math.pi) - math.pi)
                            if position_jump > self.config.d500_localization.max_position_jump_m:
                                self.d500_abs_reject_position_gate += 1
                                self._emit("d500_abs_rejected", reason="position_gate", innovation_m=position_jump)
                            elif yaw_jump > self.config.d500_localization.max_yaw_jump_rad:
                                self.d500_abs_reject_yaw_gate += 1
                                self._emit("d500_abs_rejected", reason="yaw_gate", innovation_yaw_rad=yaw_jump)
                            else:
                                pose = observation.global_pose
                                mode = "GLOBAL"
                                self.d500_abs_accept_count += 1
                        else:
                            pose = observation.global_pose
                            mode = "GLOBAL"
                            self.d500_abs_accept_count += 1
                quality = PoseQuality(
                    "d500_global" if mode == "GLOBAL" else "d500_local",
                    True,
                    mode != "GLOBAL",
                    position_confidence=confidence if mode == "GLOBAL" else None,
                    heading_confidence=confidence if mode == "GLOBAL" else None,
                    age_s=max(0.0, now_s - observation.timestamp_s),
                )
                self.fusion.update_d500(pose, quality)
                self._emit("d500_pose", x_m=pose.x_m, y_m=pose.y_m, yaw_rad=pose.yaw_rad, d500_mode=mode, confidence=confidence)

    def on_d500_update(self, update) -> None:
        """Thread-safe callback passed to the real D500 component."""

        if not update.odometry.accepted:
            self._emit("d500_rejected", reason=update.odometry.rejection_reason or "odometry_rejected", d500_mode="LOST", priority=True)
            return
        timestamp_s = float(self.clock())
        local_pose = self.radar_adapter.to_map_base_pose(update.odometry.pose, timestamp_s=timestamp_s)
        confidence = update.global_confidence
        global_pose = None
        if update.global_is_absolute and update.global_pose is not None:
            global_pose = self.radar_adapter.to_map_base_pose(update.global_pose, timestamp_s=timestamp_s)
        self._d500_events.put(D500PoseObservation(local_pose, global_pose, confidence, timestamp_s))
        icp = update.odometry.icp
        diagnostic_pose = update.global_pose if global_pose is not None else update.odometry.pose
        self._emit(
            "d500_diagnostic",
            raw_pose_cm=(diagnostic_pose.x_cm, diagnostic_pose.y_cm),
            raw_yaw_cw_deg=diagnostic_pose.yaw_cw_deg,
            icp_mean_error_cm=None if icp is None else icp.mean_error_cm,
            icp_matched_points=None if icp is None else icp.matched_points,
            icp_iterations=None if icp is None else icp.iterations,
            wall_fusion_status=None if update.wall_fusion is None else update.wall_fusion.status.value,
            d500_mode="GLOBAL" if global_pose is not None else "LOCAL_ONLY",
            global_confidence=confidence,
        )

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
            else:
                self.fusion.update_d500(pose, event.quality)
                self._emit("d500_pose", x_m=pose.x_m, y_m=pose.y_m, yaw_rad=pose.yaw_rad, replay=True)
            self._replay_index += 1

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
    world: NavigationGrid | None = None,
    event_logger: JsonlEventLogger | None = None,
) -> RobocupRuntime:
    """Build all runtime dependencies without starting a device."""

    readiness = validate_runtime_readiness(config, mode)
    if readiness:
        raise RuntimeReadinessError("; ".join(readiness))
    replay_events = read_pose_events(replay_file) if mode is RuntimeMode.REPLAY and replay_file else ()
    fake_mode = mode in {RuntimeMode.DRY_RUN, RuntimeMode.REPLAY}
    now = float(clock())
    period_s = 0.05

    drive = build_differential_drive(config, fake=fake_mode, clock=clock)
    fusion = build_pose_fusion(config)
    navigator = build_differential_navigator(config)
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
                host_monotonic_s=now + index * period_s,
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
        if config.d500.enabled:
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
    # Dry-run uses a clearly synthetic open field so the full planner can be
    # exercised without implying that hardware has a surveyed competition map.
    if world is None:
        if mode is RuntimeMode.DRY_RUN:
            world = NavigationGrid(120, 120, 0.1, origin_x_m=-6.0, origin_y_m=-6.0)
        elif mode in {RuntimeMode.HARDWARE_MISSION, RuntimeMode.REPLAY}:
            world = build_competition_world(config)
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
        clock=clock,
        mission_profile=mission_profile,
        constraints=constraints,
        world=world,
        event_logger=event_logger,
        replay_events=tuple(replay_events),
    )
    if not d500_fake and d500_source is not None:
        d500_source.on_update = runtime.on_d500_update
    if mode is RuntimeMode.DRY_RUN:
        # A short synthetic goal ensures the default acceptance command
        # exercises planning and the fake actuation path without touching hardware.
        runtime.mission.set_navigation_goal(NavigationGoal(0.8, 0.0))
    return runtime


def load_runtime_config(path: str | None = None) -> DifferentialRobotConfig:
    return load_v2_config(path)
