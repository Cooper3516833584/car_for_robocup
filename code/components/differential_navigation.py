"""Grid planning and pose-based navigation for the differential platform."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from config.v2_models import DifferentialDriveConfig, NavigationConfig
from core.frames import normalize_angle_rad
from core.types import Pose2D, Twist2D

from .navigation_common import NavigationGoal


class NavigationState(Enum):
    IDLE = "idle"
    ROTATING_TO_PATH = "rotating_to_path"
    TRACKING = "tracking"
    FINAL_ALIGN = "final_align"
    GOAL_REACHED = "goal_reached"
    BLOCKED = "blocked"
    POSE_LOST = "pose_lost"
    ERROR = "error"


@dataclass(frozen=True, slots=True)
class NavigationOutput:
    command: Twist2D
    state: NavigationState
    path: tuple[tuple[float, float], ...]
    diagnostics: dict[str, float | str | bool]


@dataclass(frozen=True, slots=True)
class ControllerOutput:
    command: Twist2D
    state: NavigationState
    diagnostics: dict[str, float | str | bool]


class DifferentialPathController:
    """Pure pursuit/rotate-first controller returning only body ``Twist2D``."""

    def __init__(
        self,
        navigation: NavigationConfig,
        drive: DifferentialDriveConfig,
    ) -> None:
        self.navigation = navigation
        self.drive = drive
        self._path_index = 0

    def reset_path(self) -> None:
        self._path_index = 0

    def compute(
        self,
        pose: Pose2D,
        path: tuple[tuple[float, float], ...],
        goal: NavigationGoal,
        *,
        protocol_compat: bool = False,
    ) -> ControllerOutput:
        dx_goal, dy_goal = goal.x_m - pose.x_m, goal.y_m - pose.y_m
        distance_goal = math.hypot(dx_goal, dy_goal)
        if distance_goal <= self.navigation.position_tolerance_m:
            if goal.yaw_rad is None:
                return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.GOAL_REACHED, {"goal_distance_m": distance_goal})
            yaw_error = normalize_angle_rad(goal.yaw_rad - pose.yaw_rad)
            if abs(yaw_error) <= self.navigation.yaw_tolerance_rad:
                return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.GOAL_REACHED, {"goal_distance_m": distance_goal, "yaw_error_rad": yaw_error})
            if not self.drive.allow_in_place_rotation:
                return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.BLOCKED, {"reason": "final_yaw_requires_in_place_rotation", "yaw_error_rad": yaw_error})
            omega = self._clamp(
                self.navigation.final_yaw_gain * yaw_error,
                self.drive.max_angular_speed_rad_s,
            )
            return ControllerOutput(Twist2D(0.0, omega), NavigationState.FINAL_ALIGN, {"goal_distance_m": distance_goal, "yaw_error_rad": yaw_error})

        target = self._lookahead_point(pose, path)
        dx, dy = target[0] - pose.x_m, target[1] - pose.y_m
        heading = math.atan2(dy, dx)
        heading_error = normalize_angle_rad(heading - pose.yaw_rad)
        if protocol_compat:
            if abs(heading_error) > self.navigation.yaw_tolerance_rad:
                if not self.drive.allow_in_place_rotation:
                    return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.BLOCKED, {"reason": "compatibility_path_requires_in_place_rotation", "heading_error_rad": heading_error})
                omega = self._clamp(self.navigation.path_yaw_gain * heading_error, self.drive.max_angular_speed_rad_s)
                return ControllerOutput(Twist2D(0.0, omega), NavigationState.ROTATING_TO_PATH, {"heading_error_rad": heading_error, "compatibility_rotate_first": True})
            speed = self._forward_speed(distance_goal, heading_error)
            return ControllerOutput(Twist2D(speed, 0.0), NavigationState.TRACKING, {"heading_error_rad": heading_error, "compatibility_rotate_first": True})

        if abs(heading_error) > self.navigation.rotate_in_place_threshold_rad:
            if not self.drive.allow_in_place_rotation:
                return ControllerOutput(Twist2D(0.0, 0.0), NavigationState.BLOCKED, {"reason": "path_heading_requires_in_place_rotation", "heading_error_rad": heading_error})
            omega = self._clamp(self.navigation.path_yaw_gain * heading_error, self.drive.max_angular_speed_rad_s)
            return ControllerOutput(Twist2D(0.0, omega), NavigationState.ROTATING_TO_PATH, {"heading_error_rad": heading_error})

        local_x = math.cos(pose.yaw_rad) * dx + math.sin(pose.yaw_rad) * dy
        local_y = -math.sin(pose.yaw_rad) * dx + math.cos(pose.yaw_rad) * dy
        lookahead_sq = max(dx * dx + dy * dy, 1e-9)
        curvature = 2.0 * local_y / lookahead_sq
        speed = self._forward_speed(distance_goal, heading_error)
        omega = self._clamp(speed * curvature, self.drive.max_angular_speed_rad_s)
        return ControllerOutput(
            Twist2D(speed, omega),
            NavigationState.TRACKING,
            {"heading_error_rad": heading_error, "curvature_inv_m": curvature, "goal_distance_m": distance_goal},
        )

    def _forward_speed(self, distance_goal: float, heading_error: float) -> float:
        speed_scale = max(0.25, math.cos(min(abs(heading_error), math.pi / 2.0)))
        if self.navigation.slowdown_distance_m > 0.0:
            speed_scale *= min(1.0, distance_goal / self.navigation.slowdown_distance_m)
        return min(self.drive.max_linear_speed_m_s, self.drive.max_linear_speed_m_s * speed_scale)

    def _lookahead_point(
        self, pose: Pose2D, path: tuple[tuple[float, float], ...]
    ) -> tuple[float, float]:
        if not path:
            raise RuntimeError("path is empty")
        nearest = min(
            range(self._path_index, len(path)),
            key=lambda index: (path[index][0] - pose.x_m) ** 2 + (path[index][1] - pose.y_m) ** 2,
        )
        self._path_index = nearest
        remaining = self.navigation.lookahead_m
        current = (pose.x_m, pose.y_m)
        for point in path[nearest + 1 :]:
            segment = math.hypot(point[0] - current[0], point[1] - current[1])
            if segment >= remaining and segment > 1e-12:
                fraction = remaining / segment
                return (current[0] + (point[0] - current[0]) * fraction, current[1] + (point[1] - current[1]) * fraction)
            remaining -= segment
            current = point
        return path[-1]

    @staticmethod
    def _clamp(value: float, maximum: float) -> float:
        return max(-maximum, min(maximum, value))


class DifferentialNavigator:
    """Direct relative-goal controller; motor I/O stays in runtime."""

    def __init__(
        self,
        drive: DifferentialDriveConfig,
        navigation: NavigationConfig,
    ) -> None:
        self.drive = drive
        self.navigation = navigation
        self.controller = DifferentialPathController(navigation, drive)
        self.goal: NavigationGoal | None = None
        self._path: tuple[tuple[float, float], ...] = ()
        self._state = NavigationState.IDLE

    def set_goal(self, goal: NavigationGoal) -> None:
        self.goal = goal
        self._path = ()
        self.controller.reset_path()
        self._state = NavigationState.IDLE

    def clear_goal(self) -> None:
        self.goal = None
        self._path = ()
        self.controller.reset_path()
        self._state = NavigationState.IDLE

    def step(
        self,
        pose: Pose2D | None,
        *,
        now_s: float,
        pose_state: object = "ok",
    ) -> NavigationOutput:
        now = float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")
        state_name = getattr(pose_state, "value", pose_state)
        if pose is None or state_name not in {"ok", "t265_degraded", "d500_degraded"}:
            self._state = NavigationState.POSE_LOST
            return self._output(Twist2D(0.0, 0.0), {"pose_state": str(state_name)})
        if not all(math.isfinite(value) for value in (pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s)):
            self._state = NavigationState.POSE_LOST
            return self._output(Twist2D(0.0, 0.0), {"reason": "non_finite_pose"})
        if self.goal is None:
            self._state = NavigationState.IDLE
            return self._output(Twist2D(0.0, 0.0), {})
        if not self._path:
            self._path = ((pose.x_m, pose.y_m), (self.goal.x_m, self.goal.y_m))
            self.controller.reset_path()

        result = self.controller.compute(
            pose,
            self._path,
            self.goal,
            protocol_compat=self.drive.protocol_mode == "ackermann_firmware_compat",
        )
        command = result.command
        if state_name in {"d500_degraded", "t265_degraded"}:
            command = Twist2D(
                command.linear_x_m_s * self.navigation.degraded_speed_scale,
                command.angular_z_rad_s * self.navigation.degraded_speed_scale,
            )
        self._state = result.state
        return self._output(command, result.diagnostics)

    def _output(self, command: Twist2D, diagnostics: dict[str, float | str | bool]) -> NavigationOutput:
        return NavigationOutput(command, self._state, self._path, diagnostics)
