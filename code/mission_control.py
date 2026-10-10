"""Task-independent checks for advancing the production runtime."""

from components.pose_fusion import PoseFusionState
from robocup_runtime import RobocupMissionState
from contextlib import contextmanager
from contextvars import ContextVar

_task_guard = ContextVar("mission_task_guard", default=None)


@contextmanager
def task_scope(guard):
    """Apply cancellation/deadline checks to nested task motion, never workers."""
    token = _task_guard.set(guard)
    try:
        yield
    finally:
        _task_guard.reset(token)


class LocalizationLostError(RuntimeError):
    """Fused localization cannot support further motion."""


def usable_pose(result):
    estimate = result.estimate
    return (estimate.pose is not None and estimate.state not in {
        PoseFusionState.LOST, PoseFusionState.UNANCHORED, PoseFusionState.INITIALIZING})


def step_runtime(runtime):
    guard = _task_guard.get()
    if guard is not None:
        guard()
    result = runtime.step()
    if result.error or runtime.mission.state in {
            RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR,
            RobocupMissionState.FINISHED}:
        reason = runtime.mission.last_error or result.error or runtime.mission.state.value
        if not usable_pose(result):
            raise LocalizationLostError(reason)
        raise RuntimeError(reason)
    return result
