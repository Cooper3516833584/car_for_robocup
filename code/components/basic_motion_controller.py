"""Non-blocking task actions for the differential robot."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from config.v2_models import DifferentialDriveConfig, NavigationConfig
from core.frames import normalize_angle_rad
from core.types import Pose2D, Twist2D

from .differential_navigation import DifferentialNavigator, NavigationOutput, NavigationState
from .navigation_common import NavigationGoal


SEGMENT_STRONG_ENTER_M = 0.10
SEGMENT_RECOVERY_ENTER_M = 0.20
SEGMENT_RECOVERY_EXIT_M = 0.15
SEGMENT_STRONG_SPEED_SCALE = 0.35
SEGMENT_RECOVERY_MAX_SPEED_M_S = 0.10
SEGMENT_STRONG_GAIN_MULTIPLIER = 2.0
SEGMENT_MAX_CORRECTION_RAD = math.radians(35.0)
REVERSE_DISTANCE_MAX_HEADING_DRIFT_RAD = math.radians(20.0)
STRAIGHT_STARTUP_MAX_COUNTER_DRIFT_RAD = math.radians(2.0)
# Stop-at-target actions must not command below the drivetrain's effective
# minimum speed: the deceleration profile reaches zero at the target, and the
# 2026-10-05 session measured that a reverse move stalls there (a 0.027 m/s
# command produced 3 mm in 4 s; 0.042 m/s tracked at 76%), leaving the vehicle
# 3.33 cm short of a 50 cm target.
#
# 0.045 m/s sits ~1.7x above the measured stall threshold and its braking
# distance `v^2 / (2 * max_linear_accel) = 6.75 mm` stays inside the 1 cm
# position tolerance, so the floor can never itself carry the vehicle past the
# goal: whichever sample first sees `remaining` inside the tolerance band, the
# worst-case stop is 6.75 mm short or long of it.
LINE_MINIMUM_SPEED_M_S = 0.045
_ZERO = Twist2D(0.0, 0.0)


class MotionBusyError(RuntimeError):
    """A task tried to replace an unfinished motion action."""


class MotionActionType(Enum):
    DRIVE_DISTANCE = "drive_distance"
    ROTATE_RELATIVE = "rotate_relative"
    ROTATE_TO = "rotate_to"
    FACE_POINT = "face_point"
    DRIVE_TO = "drive_to"
    FOLLOW_SEGMENT = "follow_segment"
    NAVIGATE_TO = "navigate_to"
    NAVIGATE_TO_POSE = "navigate_to_pose"


class MotionActionState(Enum):
    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    CANCELLED = "cancelled"
    BLOCKED = "blocked"
    POSE_LOST = "pose_lost"
    SAFE_STOPPED = "safe_stopped"
    ERROR = "error"


class MotionPhase(Enum):
    INITIALIZING = "initializing"
    ALIGNING = "aligning"
    TRACKING = "tracking"
    STRONG_CORRECTION = "strong_correction"
    RECOVERY_ALIGN = "recovery_align"
    RECOVERY_RETURN = "recovery_return"
    FINAL_ALIGN = "final_align"
    DONE = "done"


@dataclass(frozen=True, slots=True)
class MotionOutput:
    command: Twist2D
    state: MotionActionState
    action_type: MotionActionType | None
    phase: MotionPhase | None
    diagnostics: dict[str, float | str | bool]


@dataclass
class _Action:
    kind: MotionActionType
    args: tuple
    initialized: bool = False
    start: tuple[float, float] | None = None
    start_yaw: float = 0.0
    previous_yaw: float = 0.0
    accumulated_yaw: float = 0.0
    recovering: bool = False
    recovery_started: bool = False
    lateral_tolerance_m: float | None = None
    heading_reference_rad: float | None = None
    started_at_s: float | None = None
    startup_yaw_bias_rad_s: float = 0.0
    startup_duration_s: float = 0.0
    startup_finished: bool = False


def _finite(value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("motion parameters must be finite")
    return result


def _point(value) -> tuple[float, float]:
    try:
        x, y = value
    except (TypeError, ValueError) as exc:
        raise ValueError("point must contain exactly two coordinates") from exc
    return _finite(x), _finite(y)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class BasicMotionController:
    """Turn one task action into a Twist2D on each runtime cycle."""

    def __init__(self, navigator: DifferentialNavigator, navigation: NavigationConfig, drive: DifferentialDriveConfig,
                 *, track_width_m: float | None = None) -> None:
        self.navigator = navigator
        self.navigation = navigation
        self.drive = drive
        self.track_width_m = None if track_width_m is None else _finite(track_width_m)
        if self.track_width_m is not None and self.track_width_m <= 0.0:
            raise ValueError("physical track width must be positive")
        self._active: _Action | None = None
        self._state = MotionActionState.IDLE
        self._phase: MotionPhase | None = None
        self._action_type: MotionActionType | None = None
        self._diagnostics: dict[str, float | str | bool] = {}
        self.last_navigation_output: NavigationOutput | None = None

    @property
    def state(self) -> MotionActionState:
        return self._state

    @property
    def phase(self) -> MotionPhase | None:
        return self._phase

    @property
    def action_type(self) -> MotionActionType | None:
        return self._action_type

    def _begin(self, kind: MotionActionType, *args) -> None:
        if self._active is not None:
            raise MotionBusyError("stop the current motion before starting another")
        if self._state is MotionActionState.SAFE_STOPPED:
            raise RuntimeError("motion is safety stopped")
        self.navigator.clear_goal()
        self._active = _Action(kind, args)
        self._action_type = kind
        self._state = MotionActionState.RUNNING
        self._phase = MotionPhase.INITIALIZING
        self._diagnostics = {}
        self.last_navigation_output = None
        if kind in {MotionActionType.NAVIGATE_TO, MotionActionType.NAVIGATE_TO_POSE}:
            self.navigator.set_goal(NavigationGoal(*args))

    def drive_distance(self, distance_m: float, *, lateral_tolerance_m: float | None = None,
                       heading_yaw_rad: float | None = None,
                       startup_yaw_bias_rad_s: float = 0.0, startup_duration_s: float = 0.0) -> None:
        """Drive a signed distance; an explicit lateral bound selects straight travel.

        Straight travel stops on along-track distance and never pivots toward an
        endpoint's lateral residual. A saved heading can be reused for the return.
        Existing callers without this option retain their point-closure behavior.
        """
        distance = _finite(distance_m)
        lateral = None if lateral_tolerance_m is None else _finite(lateral_tolerance_m)
        heading = None if heading_yaw_rad is None else normalize_angle_rad(_finite(heading_yaw_rad))
        bias, duration = _finite(startup_yaw_bias_rad_s), _finite(startup_duration_s)
        if lateral is not None and lateral <= 0:
            raise ValueError("lateral tolerance must be positive")
        if heading is not None and lateral is None:
            raise ValueError("a saved distance heading requires an explicit lateral tolerance")
        if not 0.0 <= duration <= 2.0 or abs(bias) > self.drive.max_angular_speed_rad_s:
            raise ValueError("startup compensation must fit the angular limit and 0..2 second window")
        if bias != 0.0 and (duration <= 0.0 or distance <= 0.0 or heading is None or lateral is None
                            or self.track_width_m is None):
            raise ValueError("startup compensation requires a positive straight move with saved heading")
        self._begin(MotionActionType.DRIVE_DISTANCE, distance)
        self._active.lateral_tolerance_m = lateral
        self._active.heading_reference_rad = heading
        self._active.startup_yaw_bias_rad_s = bias
        self._active.startup_duration_s = duration

    def rotate(self, angle_rad: float) -> None:
        self._begin(MotionActionType.ROTATE_RELATIVE, _finite(angle_rad))

    def rotate_to(self, yaw_rad: float) -> None:
        self._begin(MotionActionType.ROTATE_TO, _finite(yaw_rad))

    def face_point(self, x_m: float, y_m: float) -> None:
        self._begin(MotionActionType.FACE_POINT, _finite(x_m), _finite(y_m))

    def drive_to(self, x_m: float, y_m: float) -> None:
        self._begin(MotionActionType.DRIVE_TO, _finite(x_m), _finite(y_m))

    def follow_segment(self, start_xy, end_xy) -> None:
        start, end = _point(start_xy), _point(end_xy)
        if math.dist(start, end) <= 0.0:
            raise ValueError("segment must have positive length")
        self._begin(MotionActionType.FOLLOW_SEGMENT, start, end)

    def navigate_to(self, x_m: float, y_m: float) -> None:
        self._begin(MotionActionType.NAVIGATE_TO, _finite(x_m), _finite(y_m))

    def navigate_to_pose(self, x_m: float, y_m: float, yaw_rad: float) -> None:
        self._begin(MotionActionType.NAVIGATE_TO_POSE, _finite(x_m), _finite(y_m), _finite(yaw_rad))

    def stop(self) -> None:
        if self._state is MotionActionState.SAFE_STOPPED:
            return
        self.navigator.clear_goal()
        self._active = None
        self._state = MotionActionState.CANCELLED
        self._phase = None
        self._diagnostics = {}
        self.last_navigation_output = None

    def safe_stop(self, reason: str) -> None:
        self.navigator.clear_goal()
        self._active = None
        self._state = MotionActionState.SAFE_STOPPED
        self._phase = None
        self._diagnostics = {"reason": str(reason)}
        self.last_navigation_output = None

    def step(self, pose: Pose2D | None, *, now_s: float, pose_state: object = "ok") -> MotionOutput:
        _finite(now_s)
        action = self._active
        if action is None:
            return self._output(_ZERO)
        state_name = getattr(pose_state, "value", pose_state)
        if pose is None or state_name not in {"ok", "t265_degraded", "d500_degraded"} or not all(
            math.isfinite(value) for value in (pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s)
        ):
            self._state = MotionActionState.POSE_LOST
            return self._output(_ZERO, {"reason": "pose_unavailable", "pose_state": str(state_name)})
        self._state = MotionActionState.RUNNING
        if action.kind in {MotionActionType.NAVIGATE_TO, MotionActionType.NAVIGATE_TO_POSE}:
            return self._step_navigation(pose, now_s, pose_state)
        if not action.initialized:
            action.started_at_s = now_s
            action.start = (pose.x_m, pose.y_m)
            action.start_yaw = pose.yaw_rad
            if action.heading_reference_rad is not None:
                action.start_yaw = action.heading_reference_rad
            action.previous_yaw = pose.yaw_rad
            action.initialized = True
        kind = action.kind
        if kind is MotionActionType.ROTATE_RELATIVE:
            command, diagnostics = self._step_rotate_relative(pose, action)
        elif kind is MotionActionType.ROTATE_TO:
            command, diagnostics = self._turn_to(pose, action.args[0])
        elif kind is MotionActionType.FACE_POINT:
            dx, dy = action.args[0] - pose.x_m, action.args[1] - pose.y_m
            if math.hypot(dx, dy) <= self.navigation.position_tolerance_m:
                return self._succeed()
            command, diagnostics = self._turn_to(pose, math.atan2(dy, dx))
        elif kind is MotionActionType.DRIVE_DISTANCE:
            command, diagnostics = self._step_drive_distance(pose, action)
            if (self._state is MotionActionState.RUNNING and action.startup_yaw_bias_rad_s != 0.0):
                elapsed = max(0.0, now_s - action.started_at_s)
                drift = diagnostics.get("heading_drift_rad", 0.0)
                if (action.startup_yaw_bias_rad_s * drift > 0.0
                        and abs(drift) >= STRAIGHT_STARTUP_MAX_COUNTER_DRIFT_RAD):
                    action.startup_finished = True
                weight = 0.0 if action.startup_finished else max(0.0, 1.0 - elapsed / action.startup_duration_s)
                bias = action.startup_yaw_bias_rad_s * weight
                # Keep both targets forward: compensation must not introduce
                # another wheel reversal while synchronizing the straight start.
                limit = min(self.drive.max_angular_speed_rad_s,
                            2.0 * abs(command.linear_x_m_s) / self.track_width_m)
                if weight > 0.0:
                    command = Twist2D(command.linear_x_m_s,
                                      _clamp(command.angular_z_rad_s + bias, limit))
                diagnostics.update(startup_yaw_bias_rad_s=bias, startup_elapsed_s=elapsed,
                                   startup_finished=action.startup_finished)
        elif kind is MotionActionType.DRIVE_TO:
            command, diagnostics = self._step_drive_to(pose, action)
        else:
            command, diagnostics = self._step_follow_segment(pose, action)
        if self._state is MotionActionState.SUCCEEDED:
            return self._output(_ZERO, diagnostics)
        if self._state is MotionActionState.BLOCKED:
            return self._output(_ZERO, diagnostics)
        if state_name in {"t265_degraded", "d500_degraded"}:
            scale = self.navigation.degraded_speed_scale
            command = Twist2D(command.linear_x_m_s * scale, command.angular_z_rad_s * scale)
        return self._output(command, diagnostics)

    def _step_navigation(self, pose, now_s, pose_state):
        result = self.navigator.step(pose, now_s=now_s, pose_state=pose_state)
        self.last_navigation_output = result
        self._phase = {
            NavigationState.ROTATING_TO_PATH: MotionPhase.ALIGNING,
            NavigationState.TRACKING: MotionPhase.TRACKING,
            NavigationState.FINAL_ALIGN: MotionPhase.FINAL_ALIGN,
        }.get(result.state, self._phase)
        if result.state is NavigationState.GOAL_REACHED:
            return self._succeed(result.diagnostics)
        if result.state in {NavigationState.BLOCKED, NavigationState.ERROR}:
            self._active = None
            self._state = MotionActionState.BLOCKED if result.state is NavigationState.BLOCKED else MotionActionState.ERROR
            return self._output(_ZERO, result.diagnostics)
        if result.state is NavigationState.POSE_LOST:
            self._state = MotionActionState.POSE_LOST
        return self._output(result.command, result.diagnostics)

    def _step_rotate_relative(self, pose, action):
        delta = normalize_angle_rad(pose.yaw_rad - action.previous_yaw)
        action.accumulated_yaw += delta
        action.previous_yaw = pose.yaw_rad
        error = action.args[0] - action.accumulated_yaw
        return self._turn_error(error)

    def _turn_to(self, pose, target_yaw):
        return self._turn_error(normalize_angle_rad(target_yaw - pose.yaw_rad))

    def _turn_error(self, error):
        if abs(error) <= self.navigation.yaw_tolerance_rad:
            self._succeed()
            return _ZERO, {"yaw_error_rad": error}
        if not self.drive.allow_in_place_rotation:
            return self._block("in_place_rotation_unavailable")
        self._phase = MotionPhase.ALIGNING
        omega = _clamp(self.navigation.final_yaw_gain * error, self.drive.max_angular_speed_rad_s)
        return Twist2D(0.0, omega), {"yaw_error_rad": error}

    def _step_drive_distance(self, pose, action):
        distance = action.args[0]
        start_x, start_y = action.start
        tx, ty = math.cos(action.start_yaw), math.sin(action.start_yaw)
        dx, dy = pose.x_m - start_x, pose.y_m - start_y
        progress = dx * tx + dy * ty
        cross = -dx * ty + dy * tx
        remaining = distance - progress
        goal_x, goal_y = start_x + distance * tx, start_y + distance * ty
        goal_distance = math.hypot(goal_x - pose.x_m, goal_y - pose.y_m)
        heading_drift = normalize_angle_rad(pose.yaw_rad - action.start_yaw)
        diagnostics = {
            "remaining_m": remaining, "cross_track_error_m": cross,
            "goal_distance_m": goal_distance, "heading_drift_rad": heading_drift,
        }
        if distance < 0.0 and abs(heading_drift) > REVERSE_DISTANCE_MAX_HEADING_DRIFT_RAD:
            return self._block("reverse_heading_diverged", **diagnostics)
        if action.lateral_tolerance_m is not None:
            diagnostics.update({"distance_policy": "straight", "lateral_tolerance_m": action.lateral_tolerance_m,
                                "heading_reference_rad": action.start_yaw})
            if abs(heading_drift) > REVERSE_DISTANCE_MAX_HEADING_DRIFT_RAD:
                return self._block("straight_distance_heading_diverged", **diagnostics)
            if abs(cross) > action.lateral_tolerance_m:
                return self._block("straight_distance_lateral_error_exceeded", **diagnostics)
            if abs(remaining) <= self.navigation.position_tolerance_m:
                self._succeed()
                return _ZERO, diagnostics
            direction = 1.0 if distance >= 0.0 else -1.0
            if direction * remaining < 0:
                return self._block("straight_distance_overshoot", **diagnostics)
            correction = _clamp(math.atan(self.navigation.path_yaw_gain * cross), SEGMENT_MAX_CORRECTION_RAD)
            error = normalize_angle_rad(action.start_yaw - direction * correction - pose.yaw_rad)
            diagnostics["heading_error_rad"] = error
            if abs(error) > self.navigation.rotate_in_place_threshold_rad:
                return self._block("straight_distance_requires_turn", **diagnostics)
            self._phase = MotionPhase.TRACKING
            return self._line_command(error, direction * remaining, direction,
                                      min_speed_m_s=LINE_MINIMUM_SPEED_M_S), diagnostics
        if goal_distance <= self.navigation.position_tolerance_m and abs(remaining) <= self.navigation.position_tolerance_m:
            self._succeed()
            return _ZERO, diagnostics
        direction = 1.0 if distance >= 0.0 else -1.0
        if direction * remaining <= 0.0:
            command, error = self._approach_goal(pose, goal_x, goal_y, goal_distance)
            diagnostics["heading_error_rad"] = error
            target_heading_drift = normalize_angle_rad(pose.yaw_rad + error - action.start_yaw)
            if distance < 0.0 and abs(target_heading_drift) > REVERSE_DISTANCE_MAX_HEADING_DRIFT_RAD:
                return self._block("reverse_goal_requires_turn", **diagnostics)
            return command, diagnostics
        correction = _clamp(math.atan(self.navigation.path_yaw_gain * cross), SEGMENT_MAX_CORRECTION_RAD)
        error = normalize_angle_rad(action.start_yaw - direction * correction - pose.yaw_rad)
        diagnostics["heading_error_rad"] = error
        return self._line_command(error, goal_distance, direction,
                                  min_speed_m_s=LINE_MINIMUM_SPEED_M_S), diagnostics

    def _step_drive_to(self, pose, action):
        end = action.args
        if math.dist(action.start, end) <= self.navigation.position_tolerance_m:
            self._succeed()
            return _ZERO, {"goal_distance_m": math.dist((pose.x_m, pose.y_m), end)}
        geometry = self._segment_geometry(action.start, end, pose)
        if geometry["goal_distance_m"] <= self.navigation.position_tolerance_m:
            self._succeed()
            return _ZERO, geometry
        self._phase = MotionPhase.TRACKING
        if geometry["remaining_m"] < 0.0:
            # Past the end point: close the residual with the same approach used by
            # drive_distance.  Facing a point a few centimetres away made atan2
            # noise-dominated, and the in-place turn that followed spun the vehicle
            # (177 deg observed on 2026-10-05) although the goal distance itself was
            # already inside the position tolerance.
            command, error = self._approach_goal(pose, end[0], end[1], geometry["goal_distance_m"])
            geometry["heading_error_rad"] = error
            return command, geometry
        correction = _clamp(math.atan(self.navigation.path_yaw_gain * geometry["cross_track_error_m"]), SEGMENT_MAX_CORRECTION_RAD)
        target_yaw = geometry["segment_yaw_rad"] - correction
        error = normalize_angle_rad(target_yaw - pose.yaw_rad)
        geometry["heading_error_rad"] = error
        return self._line_command(error, geometry["goal_distance_m"], 1.0,
                                  min_speed_m_s=LINE_MINIMUM_SPEED_M_S), geometry

    def _step_follow_segment(self, pose, action):
        g = self._segment_geometry(action.args[0], action.args[1], pose)
        cross = abs(g["cross_track_error_m"])
        diagnostics = dict(g)
        diagnostics["abs_cross_track_error_m"] = cross
        diagnostics.update({
            "target_yaw_rad": g["segment_yaw_rad"],
            "heading_error_rad": normalize_angle_rad(g["segment_yaw_rad"] - pose.yaw_rad),
            "recovery_active": action.recovering,
            "recovery_distance_m": math.hypot(
                g["recovery_target_x_m"] - pose.x_m, g["recovery_target_y_m"] - pose.y_m,
            ),
            "speed_scale": 0.0,
        })
        if g["goal_distance_m"] <= self.navigation.position_tolerance_m:
            self._succeed()
            return _ZERO, diagnostics
        exited_recovery = False
        if action.recovering and cross <= SEGMENT_RECOVERY_EXIT_M and g["remaining_m"] > 0.0:
            action.recovering = False
            action.recovery_started = False
            exited_recovery = True
        elif action.recovering and g["remaining_m"] <= 0.0:
            # Past the end of the segment the recovery exit test (`remaining_m > 0`)
            # can never come true, so the vehicle kept aligning for the rest of the
            # deadline (138 deg observed on 2026-10-05).  Hand the endpoint to the
            # drive_distance approach instead.
            action.recovering = False
            action.recovery_started = False
            return self._segment_endpoint_approach(pose, g, diagnostics)
        elif action.recovering:
            return self._segment_recovery(pose, action, g, diagnostics)
        elif cross > SEGMENT_RECOVERY_ENTER_M or g["remaining_m"] <= 0.0:
            action.recovering = True
            action.recovery_started = False
            return self._segment_recovery(pose, action, g, diagnostics)
        strong = cross >= SEGMENT_STRONG_ENTER_M
        self._phase = MotionPhase.STRONG_CORRECTION if strong or exited_recovery else MotionPhase.TRACKING
        gain = self.navigation.path_yaw_gain * (SEGMENT_STRONG_GAIN_MULTIPLIER if strong else 1.0)
        correction = _clamp(math.atan(gain * g["cross_track_error_m"]), SEGMENT_MAX_CORRECTION_RAD)
        target_yaw = g["segment_yaw_rad"] - correction
        error = normalize_angle_rad(target_yaw - pose.yaw_rad)
        scale = SEGMENT_STRONG_SPEED_SCALE if strong else 1.0
        diagnostics.update({"target_yaw_rad": target_yaw, "heading_error_rad": error, "speed_scale": scale, "recovery_active": False})
        command = self._line_command(
            error, max(0.0, g["remaining_m"]), 1.0, scale,
            SEGMENT_STRONG_GAIN_MULTIPLIER if strong else 1.0,
            min_speed_m_s=LINE_MINIMUM_SPEED_M_S,
        )
        return command, diagnostics

    def _segment_endpoint_approach(self, pose, geometry, diagnostics):
        """Close the last centimetres to the segment end without pivoting on it.

        The recovery turn aims at the clamped endpoint, whose bearing is
        noise-dominated at centimetre range, and its exit test can never be met
        once the vehicle is past the end of the segment.
        """
        command, error = self._approach_goal(
            pose, geometry["segment_end_x_m"], geometry["segment_end_y_m"],
            geometry["goal_distance_m"])
        diagnostics.update({
            "target_yaw_rad": math.atan2(geometry["segment_end_y_m"] - pose.y_m,
                                         geometry["segment_end_x_m"] - pose.x_m),
            "heading_error_rad": error,
            "speed_scale": 1.0,
            "recovery_active": False,
        })
        return command, diagnostics

    def _segment_recovery(self, pose, action, geometry, diagnostics):
        qx, qy = geometry["recovery_target_x_m"], geometry["recovery_target_y_m"]
        dx, dy = qx - pose.x_m, qy - pose.y_m
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        error = normalize_angle_rad(target_yaw - pose.yaw_rad)
        diagnostics.update({"target_yaw_rad": target_yaw, "heading_error_rad": error, "recovery_active": True, "recovery_distance_m": distance, "speed_scale": 0.25})
        if not self.drive.allow_in_place_rotation:
            return self._block("in_place_rotation_unavailable")
        omega = _clamp(self.navigation.path_yaw_gain * SEGMENT_STRONG_GAIN_MULTIPLIER * error, self.drive.max_angular_speed_rad_s)
        if not action.recovery_started:
            action.recovery_started = True
            self._phase = MotionPhase.RECOVERY_ALIGN
            return Twist2D(0.0, omega), diagnostics
        if abs(error) > (math.radians(20.0) if self._phase is MotionPhase.RECOVERY_RETURN else self.navigation.yaw_tolerance_rad):
            self._phase = MotionPhase.RECOVERY_ALIGN
            return Twist2D(0.0, omega), diagnostics
        self._phase = MotionPhase.RECOVERY_RETURN
        speed = min(SEGMENT_RECOVERY_MAX_SPEED_M_S, self.drive.max_linear_speed_m_s * 0.25)
        return Twist2D(speed, omega), diagnostics

    def _segment_geometry(self, start, end, pose):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        tx, ty = dx / length, dy / length
        rx, ry = pose.x_m - start[0], pose.y_m - start[1]
        along = rx * tx + ry * ty
        clamped = max(0.0, min(length, along))
        return {
            "segment_start_x_m": start[0], "segment_start_y_m": start[1],
            "segment_end_x_m": end[0], "segment_end_y_m": end[1],
            "segment_length_m": length, "along_track_m": along,
            "remaining_m": length - along, "cross_track_error_m": -rx * ty + ry * tx,
            "segment_yaw_rad": math.atan2(dy, dx),
            "goal_distance_m": math.hypot(end[0] - pose.x_m, end[1] - pose.y_m),
            "recovery_target_x_m": start[0] + clamped * tx,
            "recovery_target_y_m": start[1] + clamped * ty,
        }

    def _approach_goal(self, pose, goal_x, goal_y, goal_distance):
        """Close a small end-of-distance residual instead of spinning on the spot.

        ``drive_distance`` reaches this branch once its along-track target is met
        but the vehicle is still outside the position tolerance -- normally a
        lateral residual of a few centimetres.  The previous rule always faced
        ``yaw - pi`` (the reverse of the travel direction) and reversed to the
        goal.  That target sits behind the goal, so the vehicle turned away from
        it, a turn in place never changes ``remaining``, and the action stayed in
        ``aligning`` at the angular limit until its deadline instead of
        completing.  Picking the body direction by bearing keeps ``|error|``
        within 90 deg of the goal and lets the vehicle translate, which is what
        makes the residual shrink.
        """
        bearing = math.atan2(goal_y - pose.y_m, goal_x - pose.x_m)
        forward_error = normalize_angle_rad(bearing - pose.yaw_rad)
        if abs(forward_error) <= math.pi / 2.0:
            direction, error = 1.0, forward_error
        else:
            direction, error = -1.0, normalize_angle_rad(forward_error - math.pi)
        speed = self.drive.max_linear_speed_m_s
        speed *= min(1.0, goal_distance / self.navigation.slowdown_distance_m)
        # Raise the ramp above the drivetrain's effective minimum speed first ...
        speed = max(speed, min(self.drive.max_linear_speed_m_s, LINE_MINIMUM_SPEED_M_S))
        speed *= max(0.0, math.cos(error))
        # ... then cap the turn radius v/omega at the remaining gap.  The cap has
        # to win: with v/omega larger than ``goal_distance`` the vehicle orbits the
        # goal at a fixed radius and never lands inside the position tolerance, and
        # the per-wheel speeds still stay above the stall threshold because the
        # angular term dominates this close in.
        speed = min(speed, 0.5 * self.drive.max_angular_speed_rad_s * goal_distance)
        omega = _clamp(self.navigation.path_yaw_gain * error, self.drive.max_angular_speed_rad_s)
        self._phase = MotionPhase.ALIGNING
        return Twist2D(direction * speed, omega), error

    def _line_command(self, error, remaining, direction, speed_scale=1.0, yaw_gain_scale=1.0,
                      min_speed_m_s=0.0):
        if abs(error) > self.navigation.rotate_in_place_threshold_rad:
            if not self.drive.allow_in_place_rotation:
                self._block("in_place_rotation_unavailable")
                return _ZERO
            self._phase = MotionPhase.ALIGNING
            return Twist2D(0.0, _clamp(self.navigation.path_yaw_gain * yaw_gain_scale * error, self.drive.max_angular_speed_rad_s))
        speed = self.drive.max_linear_speed_m_s * speed_scale
        speed *= min(1.0, remaining / self.navigation.slowdown_distance_m)
        speed *= max(0.25, math.cos(min(abs(error), math.pi / 2.0)))
        # Applied last so the floor survives the shaping above; only
        # stop-at-target callers pass it, because segment recovery deliberately
        # wants a much slower approach.
        speed = max(speed, min(self.drive.max_linear_speed_m_s, min_speed_m_s))
        omega = _clamp(self.navigation.path_yaw_gain * yaw_gain_scale * error, self.drive.max_angular_speed_rad_s)
        return Twist2D(direction * speed, omega)

    def _block(self, reason, **diagnostics):
        self._active = None
        self._state = MotionActionState.BLOCKED
        return _ZERO, {"reason": reason, **diagnostics}

    def _succeed(self, diagnostics=None):
        self._active = None
        self._state = MotionActionState.SUCCEEDED
        self._phase = MotionPhase.DONE
        return self._output(_ZERO, diagnostics or {})

    def _output(self, command, diagnostics=None):
        if diagnostics is not None:
            self._diagnostics = dict(diagnostics)
        if self._phase is not None:
            self._diagnostics["phase"] = self._phase.value
        return MotionOutput(command, self._state, self._action_type, self._phase, dict(self._diagnostics))
