"""Non-blocking actions; all translation uses one directed-line PP kernel."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math
from core.frames import normalize_angle_rad
from core.types import Pose2D, Twist2D
from .differential_navigation import DifferentialNavigator, NavigationOutput, NavigationState, TurnController
from .navigation_common import NavigationGoal

_ZERO = Twist2D(0.0, 0.0)

class MotionBusyError(RuntimeError):
    pass

class MotionActionType(Enum):
    TRACK_GLOBAL_LINE = "track_global_line"
    TRACK_LOCAL_LINE = "track_local_line"
    ROTATE_RELATIVE = "rotate_relative"
    ROTATE_TO = "rotate_to"
    ROTATE_LOCAL_TO = "rotate_local_to"
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
    FINAL_ALIGN = "final_align"
    DONE = "done"

@dataclass(frozen=True, slots=True)
class MotionOutput:
    command: Twist2D
    state: MotionActionState
    action_type: MotionActionType | None
    phase: MotionPhase | None
    diagnostics: dict[str, float | str | bool | None]

@dataclass
class _Action:
    kind: MotionActionType
    args: tuple
    start_xy: tuple | None = None
    end_xy: tuple | None = None
    reverse: bool = False
    progress_s: float = 0.0
    started_at_s: float | None = None
    previous_yaw: float = 0.0
    accumulated_yaw: float = 0.0
    final_align: bool = False


def _finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("motion parameters must be finite")
    return value


def _point(value):
    x, y = value
    return _finite(x), _finite(y)


class BasicMotionController:
    def __init__(self, navigator, navigation, drive, *, track_width_m=None):
        self.navigator, self.navigation, self.drive = navigator, navigation, drive
        self.track_width_m = navigator.controller.track_width_m if track_width_m is None else _finite(track_width_m)
        if self.track_width_m <= 0:
            raise ValueError("physical track width must be positive")
        self.turn = TurnController()
        self._active = None
        self._state = MotionActionState.IDLE
        self._phase = None
        self._action_type = None
        self._diagnostics = {}
        self.last_navigation_output = None

    @property
    def state(self): return self._state
    @property
    def phase(self): return self._phase
    @property
    def action_type(self): return self._action_type

    def _begin(self, kind, *args):
        if self._active is not None:
            raise MotionBusyError("stop the current motion before starting another")
        if self._state is MotionActionState.SAFE_STOPPED:
            raise RuntimeError("motion is safety stopped")
        self.navigator.clear_goal()
        self.turn.reset()
        self._active = _Action(kind, args)
        self._action_type, self._state, self._phase = kind, MotionActionState.RUNNING, MotionPhase.INITIALIZING
        self._diagnostics = {}
        self.last_navigation_output = None

    def _line(self, kind, start_xy, end_xy, reverse=False):
        start, end = _point(start_xy), _point(end_xy)
        if math.dist(start, end) < 1e-9:
            raise ValueError("line endpoints must be different")
        self._begin(kind, start, end)
        self._active.start_xy, self._active.end_xy, self._active.reverse = start, end, bool(reverse)

    def track_global_line(self, start_xy, end_xy):
        self._line(MotionActionType.TRACK_GLOBAL_LINE, start_xy, end_xy)

    def track_local_line(self, start_xy, end_xy, *, reverse=False):
        self._line(MotionActionType.TRACK_LOCAL_LINE, start_xy, end_xy, reverse)

    def rotate(self, angle_rad): self._begin(MotionActionType.ROTATE_RELATIVE, _finite(angle_rad))
    def rotate_to(self, yaw_rad): self._begin(MotionActionType.ROTATE_TO, _finite(yaw_rad))
    def rotate_local_to(self, yaw_rad): self._begin(MotionActionType.ROTATE_LOCAL_TO, _finite(yaw_rad))
    def navigate_to(self, x_m, y_m):
        self._begin(MotionActionType.NAVIGATE_TO, _finite(x_m), _finite(y_m))
        self.navigator.set_goal(NavigationGoal(*self._active.args))
    def navigate_to_pose(self, x_m, y_m, yaw_rad):
        self._begin(MotionActionType.NAVIGATE_TO_POSE, _finite(x_m), _finite(y_m), _finite(yaw_rad))
        self.navigator.set_goal(NavigationGoal(*self._active.args))

    def stop(self):
        if self._state is MotionActionState.SAFE_STOPPED: return
        self.navigator.clear_goal()
        self.turn.reset()
        self._active, self._state, self._phase = None, MotionActionState.CANCELLED, None
        self._diagnostics, self.last_navigation_output = {}, None

    def safe_stop(self, reason):
        self.navigator.clear_goal()
        self.turn.reset()
        self._active, self._state, self._phase = None, MotionActionState.SAFE_STOPPED, None
        self._diagnostics, self.last_navigation_output = {"reason": str(reason)}, None

    def step(self, pose, *, now_s, pose_state="ok", t265_pose=None, pose_reference="fused"):
        now_s = _finite(now_s)
        action = self._active
        if action is None: return self._output(_ZERO)
        state = getattr(pose_state, "value", pose_state)
        if pose is None or state not in {"ok", "t265_degraded", "d500_degraded"} or not all(
                math.isfinite(v) for v in (pose.x_m, pose.y_m, pose.yaw_rad, pose.timestamp_s)):
            self.turn.reset()
            self._state = MotionActionState.POSE_LOST
            return self._output(_ZERO, {"reason": "pose_unavailable", "pose_reference": pose_reference})
        self._state = MotionActionState.RUNNING
        navigating = action.kind in {MotionActionType.NAVIGATE_TO, MotionActionType.NAVIGATE_TO_POSE}
        turning = action.kind in {MotionActionType.ROTATE_TO, MotionActionType.ROTATE_LOCAL_TO, MotionActionType.ROTATE_RELATIVE}
        if action.started_at_s is None:
            action.started_at_s, action.previous_yaw = now_s, pose.yaw_rad
            if navigating:
                action.start_xy, action.end_xy = (pose.x_m, pose.y_m), action.args[:2]
            if action.start_xy is not None:
                dx, dy = action.end_xy[0]-action.start_xy[0], action.end_xy[1]-action.start_xy[1]
                length = math.hypot(dx, dy)
                action.progress_s = 0.0 if length < 1e-9 else max(0., min(length,
                    ((pose.x_m-action.start_xy[0])*dx+(pose.y_m-action.start_xy[1])*dy)/length))
        controller = self.navigator.controller
        controller.navigation, controller.drive, controller.track_width_m = self.navigation, self.drive, self.track_width_m
        if turning or action.final_align:
            self._phase = MotionPhase.FINAL_ALIGN if action.final_align else MotionPhase.ALIGNING
            if action.kind is MotionActionType.ROTATE_RELATIVE:
                action.accumulated_yaw += normalize_angle_rad(pose.yaw_rad-action.previous_yaw)
                action.previous_yaw = pose.yaw_rad
                error = action.args[0]-action.accumulated_yaw
            else:
                error = normalize_angle_rad(action.args[-1]-pose.yaw_rad)
            result = self.turn.compute(error, t265_pose, now_s, self.navigation, self.drive)
        elif math.dist(action.start_xy, action.end_xy) < 1e-9:
            from .differential_navigation import ControllerOutput
            result = ControllerOutput(_ZERO, NavigationState.GOAL_REACHED, {})
        else:
            result = controller.compute_line(pose, action.start_xy, action.end_xy, action.progress_s,
                                             reverse=action.reverse)
            action.progress_s = result.diagnostics["progress_m"]
            self._phase = MotionPhase.ALIGNING if result.state is NavigationState.ROTATING_TO_PATH else MotionPhase.TRACKING
        diagnostics = {**result.diagnostics, "pose_reference": pose_reference, "reverse": action.reverse}
        if result.state is NavigationState.GOAL_REACHED:
            if navigating and action.kind is MotionActionType.NAVIGATE_TO_POSE and not action.final_align:
                action.final_align = True
                self.turn.reset()
                self._phase = MotionPhase.FINAL_ALIGN
            else:
                self._active, self._state, self._phase = None, MotionActionState.SUCCEEDED, MotionPhase.DONE
        elif result.state in {NavigationState.BLOCKED, NavigationState.POSE_LOST}:
            self._state = MotionActionState.BLOCKED if result.state is NavigationState.BLOCKED else MotionActionState.POSE_LOST
            if self._state is MotionActionState.BLOCKED: self._active = None
        command = result.command
        if state in {"t265_degraded", "d500_degraded"}:
            scale = self.navigation.degraded_speed_scale
            command = Twist2D(command.linear_x_m_s*scale, command.angular_z_rad_s*scale)
        if navigating:
            self.last_navigation_output = NavigationOutput(command, result.state, (action.start_xy, action.end_xy), diagnostics)
        return self._output(command, diagnostics)

    def _output(self, command, diagnostics=None):
        if diagnostics is not None: self._diagnostics = dict(diagnostics)
        if self._phase is not None: self._diagnostics["phase"] = self._phase.value
        return MotionOutput(command, self._state, self._action_type, self._phase, dict(self._diagnostics))
