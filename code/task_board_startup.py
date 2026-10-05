"""One-shot mission task acquisition, outside the periodic control loop."""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Callable

from components.basic_motion_controller import MotionActionState
from components.pose_fusion import PoseFusionState
from config.v2_runtime import RuntimeMode
from robocup_runtime import RobocupMissionState

if TYPE_CHECKING:
    from components.task_board_reader import TaskBoardReader, TaskBoardResult
    from robocup_runtime import RobocupRuntime


TASK_BOARD_FALLBACK = (1, 2, 1)
TASK_BOARD_FALLBACK_SOURCE = "fallback_1_2_1"


def acquire_task_board(
    runtime: RobocupRuntime,
    reader: TaskBoardReader,
    *,
    camera: int | str,
    turn_rad: float,
    timeout_s: float = 30.0,
    settle_s: float = 0.3,
    sleep: Callable[[float], None] = time.sleep,
) -> TaskBoardResult:
    """Call after referee start, with the measured signed camera-facing turn.

    The caller owns runtime.close(). No camera is opened until the turn has
    succeeded and both requested and limited chassis commands are zero.
    """
    from components.task_board_reader import TaskBoardResult, TaskCounts

    started_at = float(runtime.clock())
    terminal = {RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR, RobocupMissionState.FINISHED}

    def fatal_fail(reason: str) -> TaskBoardResult:
        runtime.mission.request_safe_stop(reason)
        runtime.drive.stop()
        runtime.record_event("task_board_startup_fatal", reason=reason)
        return TaskBoardResult(None, 0.0, reason=reason, votes=0)

    def perception_fallback(reason: str, source_result: TaskBoardResult | None,
                            turn_time_ms: float, read_time_ms: float) -> TaskBoardResult:
        counts = TaskCounts(*TASK_BOARD_FALLBACK)
        runtime.mission.set_task_counts(counts)
        runtime.record_event(
            "task_board_recognition_fallback", **counts.as_dict(),
            fallback=True, source=TASK_BOARD_FALLBACK_SOURCE, reason=reason,
            original_confidence=None if source_result is None else source_result.confidence,
            original_votes=0 if source_result is None else source_result.votes,
            original_raw_lines=() if source_result is None else source_result.raw_lines,
            original_source=None if source_result is None else source_result.source,
            turn_rad=turn_rad, turn_time_ms=turn_time_ms, capture_to_result_ms=read_time_ms,
            total_startup_ms=(float(runtime.clock()) - started_at) * 1000.0,
        )
        runtime.mission.on_payload_action_done()
        return TaskBoardResult(
            counts, 0.0,
            raw_lines=() if source_result is None else source_result.raw_lines,
            board_quad=None if source_result is None else source_result.board_quad,
            source=TASK_BOARD_FALLBACK_SOURCE, reason=reason, votes=0,
        )

    try:
        if runtime.mode not in {RuntimeMode.HARDWARE_MISSION, RuntimeMode.REPLAY}:
            return fatal_fail("task acquisition requires hardware-mission (or replay for tests)")
        if not all(math.isfinite(value) for value in (turn_rad, timeout_s, settle_s)) or timeout_s <= 0 or settle_s < 0:
            return fatal_fail("invalid task-board turn or timing parameters")
        if runtime.mission.task_counts is not None:
            return fatal_fail("task board result already set")
        if runtime.motion.state is MotionActionState.RUNNING or runtime.mission.state not in {
            RobocupMissionState.INIT, RobocupMissionState.WAIT_FOR_LOCALIZATION, RobocupMissionState.READY,
        }:
            return fatal_fail("task acquisition must precede mission motion")
        if not runtime.is_running:
            runtime.start()

        def check_runtime_step(result, *, require_pose: bool = False):
            if result.error or runtime.mission.state in terminal:
                raise RuntimeError(runtime.mission.last_error or result.error or runtime.mission.state.value)
            if require_pose and (result.estimate.pose is None or result.estimate.state in {
                PoseFusionState.LOST, PoseFusionState.UNANCHORED,
            }):
                raise RuntimeError("localization lost during task-board startup")

        def step(*, require_pose: bool = False):
            if float(runtime.clock()) - started_at >= timeout_s:
                raise TimeoutError("task-board localization/turn timed out")
            result = runtime.step()
            check_runtime_step(result, require_pose=require_pose)
            return result

        while runtime.mission.state is not RobocupMissionState.READY:
            step()
            sleep(0.05)
        turn_started = float(runtime.clock())
        runtime.motion.rotate(turn_rad)
        while True:
            stopped_step = step(require_pose=True)
            if runtime.motion.state is MotionActionState.SUCCEEDED:
                break
            sleep(0.05)
        turn_time_ms = (float(runtime.clock()) - turn_started) * 1000.0
        if runtime.mission.state is not RobocupMissionState.TARGET_OPERATION:
            return fatal_fail("task-board turn did not reach stopped target operation")
        runtime.drive.stop()
        settled_at = float(runtime.clock())
        while float(runtime.clock()) - settled_at < settle_s:
            sleep(0.05)
            stopped_step = step(require_pose=True)
            if runtime.mission.state is not RobocupMissionState.TARGET_OPERATION:
                return fatal_fail("localization lost while settling for task board")
        limited = runtime.drive.last_limited_twist
        if any(value != 0.0 for value in (
            stopped_step.command.linear_x_m_s, stopped_step.command.angular_z_rad_s,
            limited.linear_x_m_s, limited.angular_z_rad_s,
        )):
            return fatal_fail("base is not stopped for task-board recognition")

        read_started = float(runtime.clock())
        perception_exception = None
        result = None
        try:
            result = reader.recognize_camera(camera)
        except Exception as exc:
            perception_exception = exc
        read_time_ms = (float(runtime.clock()) - read_started) * 1000.0
        # Both successful OCR and ordinary perception exceptions must pass the
        # same safety refresh before either real counts or fallback can be set.
        refreshed = runtime.step()
        check_runtime_step(refreshed, require_pose=True)
        if runtime.mission.state is not RobocupMissionState.TARGET_OPERATION:
            return fatal_fail(runtime.mission.last_error or "localization lost during task-board recognition")
        limited = runtime.drive.last_limited_twist
        if runtime.motion.state is not MotionActionState.SUCCEEDED or any(value != 0.0 for value in (
            refreshed.command.linear_x_m_s, refreshed.command.angular_z_rad_s,
            limited.linear_x_m_s, limited.angular_z_rad_s,
        )):
            return fatal_fail("base is not stopped after task-board recognition")
        if perception_exception is not None:
            return perception_fallback(
                f"task-board perception exception: {type(perception_exception).__name__}: {perception_exception}",
                None, turn_time_ms, read_time_ms,
            )
        if result is None:
            return perception_fallback("task-board perception returned no result", None, turn_time_ms, read_time_ms)
        if not result.valid:
            return perception_fallback(result.reason or "task-board recognition failed", result, turn_time_ms, read_time_ms)
        if result.votes < reader.config.required_consensus_votes:
            return perception_fallback(
                result.reason or f"task-board consensus not reached: {result.votes}/{reader.config.required_consensus_votes}",
                result, turn_time_ms, read_time_ms,
            )
        runtime.mission.set_task_counts(result.counts)
        runtime.record_event(
            "task_board_recognition", **result.task,
            confidence=result.confidence, votes=result.votes,
            source=result.source, fallback=False,
            inferred_colors=result.inferred_colors, raw_lines=result.raw_lines,
            turn_rad=turn_rad, turn_time_ms=turn_time_ms, capture_to_result_ms=read_time_ms,
            total_startup_ms=(float(runtime.clock()) - started_at) * 1000.0,
        )
        runtime.mission.on_payload_action_done()
        return result
    except Exception as exc:
        return fatal_fail(f"task-board startup failed: {type(exc).__name__}: {exc}")
    except BaseException as exc:
        fatal_fail(f"task-board startup interrupted: {type(exc).__name__}")
        raise
