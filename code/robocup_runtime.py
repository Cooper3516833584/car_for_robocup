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
from components.navigation_common import NavigationGoal, NavigationGrid
from components.pose_fusion import FusedPoseEstimate, PoseFusion, PoseFusionState
from components.radar_pose_adapter import RadarPoseAdapter
from components.t265_driver import FakeT265PoseSource, RealSenseT265PoseSource, T265RawPose
from components.t265_pose_adapter import T265PoseAdapter
from config.v2_factory import build_differential_drive, build_differential_navigator, build_pose_fusion
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
        self._d500_events: queue.SimpleQueue[tuple[object, float]] = queue.SimpleQueue()
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
            self._started = True
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
        navigation_output: NavigationOutput | None = None
        try:
            self._consume_t265(now)
            self._consume_d500(now)
            estimate = self.fusion.estimate(now)
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

            if self.mission.state in {
                RobocupMissionState.SAFE_STOP,
                RobocupMissionState.ERROR,
                RobocupMissionState.FINISHED,
                RobocupMissionState.TARGET_OPERATION,
            } or estimate.state is PoseFusionState.LOST:
                command = Twist2D(0.0, 0.0)
                self._safe_stop_drive()
            elif self.mode is RuntimeMode.HARDWARE_PROBE:
                command = Twist2D(0.0, 0.0)
            elif command.linear_x_m_s != 0.0 or command.angular_z_rad_s != 0.0:
                if not self.drive.is_running:
                    self.drive.start()
                self.drive.command(command, now_s=now)
            elif self.drive.is_running:
                self.drive.stop()

            return RuntimeStep(now, estimate, self.mission.state, command, navigation_output)
        except Exception as exc:
            self._error = f"{type(exc).__name__}: {exc}"
            self.mission.request_error(self._error)
            self._safe_stop_drive()
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

    def close(self) -> None:
        if self._closed:
            return
        # Stop the base first; source shutdown can block while joining workers.
        self._safe_stop_drive()
        for source in (self.t265_source, self.d500_source):
            if source is not None:
                self._stop_source(source)
        try:
            self.drive.close()
        except Exception:
            LOG.exception("failed to close drive")
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

    def _consume_d500(self, now_s: float) -> None:
        if self.d500_fake:
            if self.d500_source is None:
                return
            sample = self.d500_source.read()
            if sample is not None:
                pose = self.radar_adapter.to_map_base_pose(sample, timestamp_s=sample.timestamp_s)
                self.fusion.update_d500(pose, PoseQuality("d500", True, False, age_s=max(0.0, now_s - sample.timestamp_s)))
        else:
            while True:
                try:
                    legacy_pose, timestamp_s = self._d500_events.get_nowait()
                except queue.Empty:
                    break
                pose = self.radar_adapter.to_map_base_pose(legacy_pose, timestamp_s=timestamp_s)
                self.fusion.update_d500(pose, PoseQuality("d500", True, False, age_s=max(0.0, now_s - timestamp_s)))

    def on_d500_update(self, update) -> None:
        """Thread-safe callback passed to the real D500 component."""

        if not update.odometry.accepted:
            return
        pose = update.global_pose or update.odometry.pose
        self._d500_events.put((pose, float(self.clock())))

    def _safe_stop_drive(self) -> None:
        if self.drive.is_running:
            try:
                self.drive.stop()
            except Exception:
                LOG.exception("failed to stop differential drive")

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
) -> RobocupRuntime:
    """Build all runtime dependencies without starting a device."""

    readiness = validate_runtime_readiness(config, mode)
    if readiness:
        raise RuntimeReadinessError("; ".join(readiness))
    if mode is RuntimeMode.REPLAY and replay_file:
        raise NotImplementedError("pose-log replay source is added in migration step 13")
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

    if fake_mode:
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
            from components.radar_driver import D500RadarComponent, RadarMount

            mount = RadarMount(
                x_forward_cm=config.d500_mount.x_m * 100.0,
                y_left_cm=config.d500_mount.y_m * 100.0,
                yaw_cw_deg=-math.degrees(config.d500_mount.yaw_rad),
            )
            d500_source = D500RadarComponent(
                port=config.d500.port,
                mount=mount,
                on_update=None,
            )
        else:
            d500_source = None
        d500_fake = False

    constraints = runtime_constraints(config, mode)
    # Dry-run uses a clearly synthetic open field so the full planner can be
    # exercised without implying that hardware has a surveyed competition map.
    if world is None and mode is RuntimeMode.DRY_RUN:
        world = NavigationGrid(120, 120, 0.1, origin_x_m=-6.0, origin_y_m=-6.0)
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
    )
    if not d500_fake and d500_source is not None:
        d500_source.on_update = runtime.on_d500_update
    return runtime


def load_runtime_config(path: str | None = None) -> DifferentialRobotConfig:
    return load_v2_config(path)
