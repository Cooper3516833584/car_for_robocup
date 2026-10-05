"""One-shot mission task acquisition, outside the periodic control loop."""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Callable

from components.basic_motion_controller import MotionActionState
from config.v2_runtime import RuntimeMode
from robocup_runtime import RobocupMissionState

if TYPE_CHECKING:
    from components.task_board_reader import TaskBoardReader, TaskBoardResult
    from robocup_runtime import RobocupRuntime


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
    from components.task_board_reader import TaskBoardResult

    started_at = float(runtime.clock())
    terminal = {RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR, RobocupMissionState.FINISHED}

    def fail(reason: str) -> TaskBoardResult:
        runtime.mission.request_safe_stop(reason)
        runtime.drive.stop()
        runtime.record_event("task_board_recognition_failed", reason=reason)
        return TaskBoardResult(None, 0.0, reason=reason, votes=0)

    try:
        if runtime.mode not in {RuntimeMode.HARDWARE_MISSION, RuntimeMode.REPLAY}:
            return fail("task acquisition requires hardware-mission (or replay for tests)")
        if not all(math.isfinite(value) for value in (turn_rad, timeout_s, settle_s)) or timeout_s <= 0 or settle_s < 0:
            return fail("invalid task-board turn or timing parameters")
        if runtime.mission.task_counts is not None:
            return fail("task board result already set")
        if runtime.motion.state is MotionActionState.RUNNING or runtime.mission.state not in {
            RobocupMissionState.INIT, RobocupMissionState.WAIT_FOR_LOCALIZATION, RobocupMissionState.READY,
        }:
            return fail("task acquisition must precede mission motion")
        if not runtime.is_running:
            runtime.start()

        def step():
            if float(runtime.clock()) - started_at >= timeout_s:
                raise TimeoutError("task-board localization/turn timed out")
            result = runtime.step()
            if runtime.mission.state in terminal:
                raise RuntimeError(runtime.mission.last_error or runtime.mission.state.value)
            return result

        while runtime.mission.state is not RobocupMissionState.READY:
            step()
            sleep(0.05)
        turn_started = float(runtime.clock())
        runtime.motion.rotate(turn_rad)
        while True:
            stopped_step = step()
            if runtime.motion.state is MotionActionState.SUCCEEDED:
                break
            sleep(0.05)
        turn_time_ms = (float(runtime.clock()) - turn_started) * 1000.0
        if runtime.mission.state is not RobocupMissionState.TARGET_OPERATION:
            return fail("task-board turn did not reach stopped target operation")
        runtime.drive.stop()
        settled_at = float(runtime.clock())
        while float(runtime.clock()) - settled_at < settle_s:
            sleep(0.05)
            stopped_step = step()
            if runtime.mission.state is not RobocupMissionState.TARGET_OPERATION:
                return fail("localization lost while settling for task board")
        limited = runtime.drive.last_limited_twist
        if any(value != 0.0 for value in (
            stopped_step.command.linear_x_m_s, stopped_step.command.angular_z_rad_s,
            limited.linear_x_m_s, limited.angular_z_rad_s,
        )):
            return fail("base is not stopped for task-board recognition")

        read_started = float(runtime.clock())
        result = reader.recognize_camera(camera)
        read_time_ms = (float(runtime.clock()) - read_started) * 1000.0
        if not result.valid or result.votes < reader.config.required_consensus_votes:
            return fail(result.reason or "task-board recognition consensus not reached")
        # Refresh localization after the potentially long OCR call, before
        # allowing any subsequent mission movement.
        runtime.step()
        if runtime.mission.state is not RobocupMissionState.TARGET_OPERATION:
            return fail(runtime.mission.last_error or "localization lost during task-board recognition")
        runtime.mission.set_task_counts(result.counts)
        runtime.record_event(
            "task_board_recognition", **result.task,
            confidence=result.confidence, votes=result.votes,
            inferred_colors=result.inferred_colors, raw_lines=result.raw_lines,
            turn_rad=turn_rad, turn_time_ms=turn_time_ms, capture_to_result_ms=read_time_ms,
            total_startup_ms=(float(runtime.clock()) - started_at) * 1000.0,
        )
        runtime.mission.on_payload_action_done()
        return result
    except Exception as exc:
        return fail(f"task-board startup failed: {type(exc).__name__}: {exc}")
    except BaseException:
        runtime.mission.request_safe_stop("task-board startup interrupted")
        runtime.drive.stop()
        raise
