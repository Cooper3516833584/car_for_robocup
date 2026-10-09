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


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class TurnController:
    """Shared yaw controller for explicit turns and final navigation alignment."""
    def reset(self): pass

    def compute(self, error, t265_pose, now_s, navigation, drive):
        info = {"yaw_error_rad": error}
        if abs(error) <= navigation.yaw_tolerance_rad:
            return ControllerOutput(Twist2D(0, 0), NavigationState.GOAL_REACHED, info)
        if not drive.allow_in_place_rotation:
            return ControllerOutput(Twist2D(0, 0), NavigationState.BLOCKED, {**info, "reason": "in_place_rotation_unavailable"})
        omega = math.copysign(min(abs(navigation.final_yaw_gain*error), drive.max_angular_speed_rad_s,
                                 math.sqrt(2*drive.max_angular_accel_rad_s2*abs(error))), error)
        return ControllerOutput(Twist2D(0, omega), NavigationState.FINAL_ALIGN, info)


class DifferentialPathController:
    """The only translation kernel: directed line Pure Pursuit, no motor I/O."""

    def __init__(self, navigation, drive, *, track_width_m=0.198):
        self.navigation, self.drive = navigation, drive
        self.track_width_m = track_width_m
        self.reset_path()

    def reset_path(self):
        self.progress_m = 0.0

    def compute_line(self, pose, start, end, previous_progress_m=0.0, *, reverse=False,
                     position_tolerance_m=None, terminal_lateral_m=0.03):
        lookahead_m = self.navigation.lookahead_m
        speed_limit_m_s = self.drive.max_linear_speed_m_s
        stop_tolerance_m = (self.navigation.position_tolerance_m if position_tolerance_m is None
                            else position_tolerance_m)
        max_omega_rad_s = self.drive.max_angular_speed_rad_s
        track_width_m = self.track_width_m
        decel_m_s2 = self.drive.max_linear_accel_m_s2
        px, py, yaw = pose.x_m, pose.y_m, pose.yaw_rad
        ax, ay = start
        bx, by = end
        values = (px, py, yaw, ax, ay, bx, by, previous_progress_m, lookahead_m,
                  speed_limit_m_s, stop_tolerance_m, terminal_lateral_m,
                  max_omega_rad_s, track_width_m, decel_m_s2)
        if not all(math.isfinite(x) for x in values):
            raise ValueError("non-finite Pure Pursuit input")
        if min(lookahead_m, speed_limit_m_s, stop_tolerance_m, terminal_lateral_m,
               max_omega_rad_s, track_width_m, decel_m_s2) <= 0.0:
            raise ValueError("positive limits required")

        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-9:
            raise ValueError("line endpoints must be different")
        ux, uy = dx / length, dy / length
        raw_s = (px - ax) * ux + (py - ay) * uy
        cross = ux * (py - ay) - uy * (px - ax)
        progress = max(clamp(previous_progress_m, 0.0, length),
                       clamp(raw_s, 0.0, length))
        remaining = length - raw_s
        info: dict[str, float | str | bool] = {
            "progress_m": progress, "reverse": reverse,
            "raw_progress_m": raw_s,
            "remaining_m": remaining,
            "cross_track_m": cross,
            "segment_length_m": length,
        }

        def result(v, omega, state, status):
            info.update(line_state=status, command_v_m_s=v, command_omega_rad_s=omega)
            if state is NavigationState.BLOCKED:
                info["reason"] = status
            return ControllerOutput(Twist2D(v, omega), state, info)

        # Stop based on un-clamped physical along-track progress, never on
        # monotonic carrot progress alone. Do not pivot toward a lateral endpoint.
        if remaining <= stop_tolerance_m:
            if abs(cross) > terminal_lateral_m:
                return result(0.0, 0.0, NavigationState.BLOCKED, "terminal_lateral_error")
            return result(0.0, 0.0, NavigationState.GOAL_REACHED, "arrived")

        carrot_s = min(length, progress + lookahead_m)
        cx, cy = ax + ux * carrot_s, ay + uy * carrot_s
        wx, wy = cx - px, cy - py
        co, si = math.cos(yaw), math.sin(yaw)
        local_x = co * wx + si * wy
        local_y = -si * wx + co * wy
        lookahead_sq = local_x * local_x + local_y * local_y
        info.update(carrot_x_m=cx, carrot_y_m=cy,
                    carrot_local_x_m=local_x, carrot_local_y_m=local_y)
        if lookahead_sq < 1e-8:
            return result(0.0, 0.0, NavigationState.BLOCKED, "carrot_degenerate")

        # For reverse the robot's forward axis should point away from the carrot.
        # This angle is correct even if the chassis yaw is not exactly on the line.
        heading_error = (math.atan2(-local_y, -local_x) if reverse
                         else math.atan2(local_y, local_x))
        info["heading_error_rad"] = heading_error
        if abs(heading_error) > math.radians(60.0):
            # Rotate in place toward the FORWARD travel-facing angle. This is not
            # a backwards path recovery; the separate turn/stable-stop controller
            # should own actual completion on hardware.
            omega = clamp(1.5 * heading_error, -0.30, 0.30)
            return result(0.0, omega, NavigationState.ROTATING_TO_PATH, "align_forward")

        curvature = 2.0 * local_y / lookahead_sq
        # A 0.045 m/s floor reflects prior car tests; not a universal motor value.
        speed = min(speed_limit_m_s, math.sqrt(2.0 * decel_m_s2 * max(remaining, 0.0)))
        speed = max(0.045, speed / (1.0 + 0.5 * abs(curvature)))
        speed = min(speed, speed_limit_m_s)
        if length <= 0.10:
            speed = min(speed, 0.08)  # conservative first trial for 7 cm moves
        speed = max(0.0, speed)
        signed_v = -speed if reverse else speed
        omega = signed_v * curvature
        # For translated motion, disallow one wheel reversing while the other
        # goes forward: wheel reversal was unreliable immediately after turns.
        # In-place turns above are deliberately exempt.
        no_wheel_reversal_omega = 1.8 * abs(signed_v) / track_width_m
        omega_limit = min(max_omega_rad_s, no_wheel_reversal_omega)
        omega = clamp(omega, -omega_limit, omega_limit)
        wheel_peak = abs(signed_v) + abs(omega) * track_width_m / 2
        scale = min(1.0, self.drive.max_wheel_speed_m_s / max(wheel_peak, 1e-9))
        signed_v *= scale
        omega *= scale
        info.update(curvature_inv_m=curvature, carrot_progress_m=carrot_s,
                    max_tracking_omega_rad_s=omega_limit)
        return result(signed_v, omega, NavigationState.TRACKING, "tracking")

    def compute(self, pose, path, goal, *, protocol_compat=False):
        # Compatibility changes the backend, never selects a second controller.
        if not path:
            raise ValueError("path is empty")
        if math.dist(path[0], path[-1]) < 1e-9:
            return ControllerOutput(Twist2D(0, 0), NavigationState.GOAL_REACHED, {})
        output = self.compute_line(pose, path[0], path[-1], self.progress_m)
        self.progress_m = output.diagnostics["progress_m"]
        return output


class DifferentialNavigator:
    """Direct relative-goal controller; motor I/O stays in runtime."""

    def __init__(
        self,
        drive: DifferentialDriveConfig,
        navigation: NavigationConfig,
        *, track_width_m: float = 0.198,
    ) -> None:
        self.drive = drive
        self.navigation = navigation
        self.controller = DifferentialPathController(navigation, drive, track_width_m=track_width_m)
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
