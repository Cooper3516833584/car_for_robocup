"""Independent production entry point for the RoboCup differential platform."""

from __future__ import annotations

# 快速调整区：距离单位米，右墙距离顺序为左边、底边、右边、顶边。
# 无参数运行此文件：通过开机监听服务启动正式任务，保留任务板识别。
WALL_RIGHT_DISTANCE_M = (1.600, 1.895, 1.690, 2.330)
STANDARD_DROP_M = 0.43       # 普通投放前进距离（槽位 0..3）
RIGHT_MOUNT_DROP_M = 0.32    # 右侧投放前进距离（黄色，槽位 4）
FINISH_FRONT_DISTANCE_M = 1.10

import argparse
import logging
import math
from pathlib import Path
import sys
import signal
import threading
from components.payload_servos import open_payload

from components.diagnostics_log import JsonlEventLogger
from components.navigation_common import NavigationGoal
from config.relative_slam_profile import accepted_relative_slam_profile
from config.mission_profile import route_speed_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import (
    RuntimeReadinessError,
    WallRuntimeReadinessError,
    build_runtime,
    build_wall_runtime,
    load_runtime_config,
)


DIRECT_COMPETITION_STAGES = {"task-board", "hc-send", "yellow-detect", "drop"}
# Stages that must never run through the fused runtime: they need the raw wall
# observer and the wall-action controller instead.
WALL_COMPETITION_STAGES = {"wall-observe", "wall-leg", "wall-turn", "drop-standard",
                           "drop-right", "top-return"}
# Retained CLI compatibility value; wall missions no longer abort on a timer.
COMPETITION_MAX_SECONDS = 1200.0


def operator_options():
    from components.competition_options import CompetitionOptions
    return CompetitionOptions(wall_right_distance_m=WALL_RIGHT_DISTANCE_M,
                              standard_drop_m=STANDARD_DROP_M,
                              right_mount_drop_m=RIGHT_MOUNT_DROP_M,
                              finish_front_m=FINISH_FRONT_DISTANCE_M).as_dict()


def launch(argv=None):
    """Run-button entry; explicit CLI arguments retain the diagnostic interface."""
    arguments = sys.argv[1:] if argv is None else argv
    if arguments:
        return main(arguments)
    from components.local_mission_control import run_local
    return run_local(operator_options())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="RoboCup differential robot runtime")
    parser.add_argument("--config", help="schema-v2 TOML configuration")
    parser.add_argument(
        "--mode",
        choices=[mode.value for mode in RuntimeMode],
        default=RuntimeMode.DRY_RUN.value,
    )
    parser.add_argument("--log-dir", help="directory for runtime log output")
    parser.add_argument("--replay-file", help="pose replay log (replay mode)")
    parser.add_argument("--mission-profile", default="default")
    localization = parser.add_mutually_exclusive_group()
    localization.add_argument("--relative-slam", action="store_true",
                              help="use accepted T265(2/3)+D500/SLAM localization for relative task actions")
    localization.add_argument("--localization-from-config", action="store_true",
                              help="use localization settings from the TOML profile instead of the hardware-mission default")
    parser.add_argument("--steps", type=int, default=8, help="fixed dry-run/replay step count")
    parser.add_argument("--goal-x", type=float, help="optional goal x in the current fused pose frame, metres")
    parser.add_argument("--goal-y", type=float, help="optional goal y in the current fused pose frame, metres")
    parser.add_argument("--goal-yaw", type=float, help="optional final yaw in the current fused pose frame, radians")
    parser.add_argument("--task-board-camera", help="startup/competition task camera path or index")
    parser.add_argument("--task-board-turn-deg", type=float,
                        help="measured signed chassis turn toward the board, required with --task-board-camera")
    parser.add_argument("--task-board-debug-dir", type=Path, help="optional task-board images and OCR evidence")
    parser.add_argument("--task-red-count", type=int, help="hc-send only: actual red quota")
    parser.add_argument("--task-blue-count", type=int, help="hc-send only: actual blue quota")
    parser.add_argument("--task-green-count", type=int, help="hc-send only: actual green quota")
    parser.add_argument("--competition", action="store_true", help="run the complete competition mission (hardware-mission)")
    parser.add_argument("--competition-stage", choices=(
        "full", "lane", "task-board", "hc-send", "corner1", "cross-lane",
        "yellow-detect", "yellow-search", "yellow-align", "drop-route", "drop",
        "drop-align", "corner2", "finish",
        "wall-observe", "wall-leg", "wall-turn", "drop-standard", "drop-right",
        "top-return"),
        help="run only this stage; task-board/hc-send/yellow-detect/drop use direct hardware without "
             "localization runtime; wall-observe is read-only; wall-leg/wall-turn take --leg 1..4")
    parser.add_argument("--leg", type=int, choices=(1, 2, 3, 4),
                        help="wall-leg/wall-turn only: which wall leg to exercise")
    parser.add_argument("--yellow-camera", help="competition camera index or stable device path")
    parser.add_argument("--yellow-model", type=Path, help="existing car YOLO weights path")
    parser.add_argument("--speed-scale", type=float, default=1,
                        help="shared motion speed multiplier; use 2 for the validated faster profile")
    parser.add_argument("--max-seconds", type=float, default=COMPETITION_MAX_SECONDS,
                        help="legacy diagnostic argument; wall missions do not abort on this timer")
    return parser


def configure_logging(log_dir: str | None) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_dir:
        directory = Path(log_dir)
        directory.mkdir(parents=True, exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        handlers=handlers,
    )


def camera_value(value):
    return int(value) if value is not None and value.isdigit() else value


def run_direct_competition_stage(args) -> int:
    """Four non-motion tests only; each owns just its required device."""
    import competition_task as task

    stage = args.competition_stage
    if stage == "task-board":
        from components.task_board_reader import TaskBoardConfig, TaskBoardReader

        counts = task.read_task_board(camera_value(args.task_board_camera), reader=TaskBoardReader(
            TaskBoardConfig(debug_directory=args.task_board_debug_dir)))
        logging.info("task-board result: red=%d blue=%d green=%d", counts.red, counts.blue, counts.green)
        return 0  # The existing fallback warns and remains a valid task.
    if stage == "hc-send":
        from components.task_board_reader import TaskCounts

        counts = TaskCounts(args.task_red_count, args.task_blue_count, args.task_green_count)
        ok = task.send_task_to_drone_once(counts, start_drone=False)
        return 0 if ok is True else 1
    if stage == "yellow-detect":
        detector = task.load_detector(args.yellow_model or task.YELLOW_MODEL_PATH)
        detection = task.detect_yellow_from_camera(camera_value(args.yellow_camera), detector)
        if detection is None:
            logging.error("yellow target not detected")
            return 1
        logging.info("yellow: cx=%.1f cy=%.1f w=%.1f h=%.1f conf=%.3f", detection.cx_px,
                     detection.cy_px, detection.width_px, detection.height_px, detection.confidence)
        return 0
    if stage == "drop":
        return 1 if open_payload() is False else 0
    raise ValueError(f"unsupported direct competition stage: {stage}")


def stop_marker_signature(path: Path):
    try:
        st = path.stat()
    except FileNotFoundError:
        return None
    return (st.st_dev, st.st_ino, st.st_size, st.st_mtime_ns, st.st_ctime_ns)


def main(argv: list[str] | None = None, *, stop_event=None, send_drone_task=None,
         on_runtime_ready=None, car_only_test=False, simulate_payload=False,
         stop_file_baseline=None, run_started_at_s=None, run_options=None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.log_dir)
    competition_enabled = args.competition or args.competition_stage is not None
    direct_stage = args.competition_stage in DIRECT_COMPETITION_STAGES
    if car_only_test and (not competition_enabled or
                          args.competition_stage not in {None, "full"}):
        logging.error("car full-flow test requires the full competition mission")
        return 2
    if not math.isfinite(args.speed_scale) or args.speed_scale <= 0:
        logging.error("--speed-scale must be finite and positive")
        return 2
    if not math.isfinite(args.max_seconds) or not 1 <= args.max_seconds <= COMPETITION_MAX_SECONDS:
        logging.error("--max-seconds must be finite and between 1 and %g", COMPETITION_MAX_SECONDS)
        return 2
    wall_stage = args.competition_stage in WALL_COMPETITION_STAGES
    if args.leg is not None and args.competition_stage not in {"wall-leg", "wall-turn"}:
        logging.error("--leg applies only to wall-leg and wall-turn")
        return 2
    if args.competition_stage in {"wall-leg", "wall-turn"} and args.leg is None:
        logging.error("--competition-stage %s requires --leg 1..4", args.competition_stage)
        return 2
    if competition_enabled and not direct_stage and args.mode != RuntimeMode.HARDWARE_MISSION.value:
        logging.error("competition requires --mode hardware-mission; use fake unit tests for software route validation")
        return 2
    if competition_enabled and (args.goal_x is not None or args.goal_y is not None
                                or args.goal_yaw is not None or args.task_board_turn_deg is not None):
        logging.error("competition owns its route; do not combine with --goal-* or startup --task-board-turn-deg")
        return 2
    if (args.goal_x is None) != (args.goal_y is None):
        logging.error("--goal-x and --goal-y must be provided together")
        return 2
    if args.goal_yaw is not None and args.goal_x is None:
        logging.error("--goal-yaw requires --goal-x and --goal-y")
        return 2
    task_board_enabled = args.task_board_camera is not None and not competition_enabled
    if task_board_enabled != (args.task_board_turn_deg is not None):
        logging.error("--task-board-camera and --task-board-turn-deg must be provided together")
        return 2
    if task_board_enabled and (args.mode != RuntimeMode.HARDWARE_MISSION.value
                               or not math.isfinite(args.task_board_turn_deg)):
        logging.error("task-board startup requires hardware-mission and a finite measured turn")
        return 2

    if direct_stage:
        try:
            if simulate_payload and args.competition_stage == 'drop':
                logging.info('SIMULATION direct payload OPEN skipped')
                return 0
            return run_direct_competition_stage(args)
        except KeyboardInterrupt:
            logging.warning("direct competition stage interrupted")
            return 130
        except Exception:
            logging.exception("direct competition stage failed: %s", args.competition_stage)
            return 1

    use_wall_runtime = wall_stage or (competition_enabled and not direct_stage)
    try:
        config = load_runtime_config(args.config)
        mode = RuntimeMode(args.mode)
        # The wall chain must be chosen BEFORE the fused profile default, or a
        # "new navigation" would still build the old localization stack.
        if not use_wall_runtime and (
                args.relative_slam
                or (mode is RuntimeMode.HARDWARE_MISSION and not args.localization_from_config)):
            config = accepted_relative_slam_profile(config)
        config = route_speed_config(config, args.speed_scale)
        if use_wall_runtime:
            runtime = build_wall_runtime(config, mode)
            from components.competition_options import CompetitionOptions
            options = operator_options()
            if run_options is not None:
                options.update(run_options)
            if car_only_test and options.get('task_counts') is None:
                options['task_counts'] = (1, 2, 1)
            runtime.competition_options = CompetitionOptions(**options)
            runtime.record_event("competition_runtime_chain", chain="wall",
                                 stage=args.competition_stage or "full",
                                 fused_localization=False, drone_communication=False)
        else:
            runtime = build_runtime(
                config,
                mode,
                replay_file=args.replay_file,
                mission_profile=args.mission_profile,
            )
        if args.log_dir is not None:
            runtime.event_logger = JsonlEventLogger(Path(args.log_dir) / "events.jsonl")
        elif task_board_enabled or competition_enabled:
            runtime.event_logger = JsonlEventLogger(Path("logs") / (
                "competition" if competition_enabled else "task-board") / "events.jsonl")
    except (RuntimeReadinessError, WallRuntimeReadinessError, FileNotFoundError,
            ValueError, NotImplementedError) as exc:
        logging.error("cannot start RoboCup runtime: %s", exc)
        if isinstance(exc, (RuntimeReadinessError, WallRuntimeReadinessError)) and args.log_dir is not None:
            try:
                readiness_log = JsonlEventLogger(Path(args.log_dir) / "events.jsonl")
                readiness_log.emit(
                    {
                        "t": 0.0,
                        "type": "safety",
                        "event_code": "UNMEASURED_CONFIG_REJECT",
                        "reason": str(exc),
                    },
                    priority=True,
                )
                readiness_log.close()
            except Exception:
                logging.exception("could not record readiness rejection")
        return 2

    previous_sigterm = None
    try:
        if on_runtime_ready is not None:
            on_runtime_ready(runtime)
        if competition_enabled:
            from competition_task import run_competition_stage
            from mission_control import task_scope

            stage = args.competition_stage or "full"
            runtime.record_event("competition_motion_profile", speed_scale=args.speed_scale,
                                 translation_speed_scale=config.navigation.translation_speed_scale,
                                 lookahead_base_m=config.navigation.lookahead_m,
                                 lookahead_max_extension_m=config.navigation.lookahead_max_extension_m,
                                 actuator="dual_servo")
            # Only explicit STOP / Ctrl+C ends an unfinished wall mission.
            stop_file = Path(args.log_dir or "logs/competition") / "STOP"

            def guard():
                if stop_event is not None and stop_event.is_set():
                    raise RuntimeError("competition interrupted by STOP")
                try:
                    current = stop_marker_signature(stop_file)
                except OSError:
                    current = None
                if current is not None and (stop_file_baseline is None or current != stop_file_baseline):
                    raise RuntimeError("competition interrupted by STOP file")

            def terminate(*_):
                raise KeyboardInterrupt

            if threading.current_thread() is threading.main_thread():
                previous_sigterm = signal.signal(signal.SIGTERM, terminate)
            with task_scope(guard):
                guard()
                result = run_competition_stage(
                    runtime, stage,
                    task_board_camera=camera_value(args.task_board_camera),
                    yellow_camera=camera_value(args.yellow_camera),
                    weights=args.yellow_model, send_drone_task=send_drone_task,
                    car_only_test=car_only_test,
                    simulate_release=simulate_payload,
                    payload_log_dir=Path(args.log_dir or "logs/competition") / "payload" if stage == "full" else None,
                    leg=args.leg,
                )
            logging.info("competition stage completed: %s; result=%s",
                         stage,
                         result if isinstance(result, (bool, type(None))) else type(result).__name__)
            if runtime.mission.state.value in {"error", "safe_stop"}:
                return 1
            if stage in {"yellow-search", "yellow-align", "drop-align"}:
                return 0 if result is not None else 1
            if stage in {"drop-route", "drop", "drop-standard", "drop-right"} and result is False:
                return 1
            if stage in {"wall-leg", "wall-turn", "top-return", "finish"}:
                return 0 if result is not False else 1
            if stage == "full":
                # Reaching the finish zone is what "finished" means; a
                # missing finish distance stays parked at the corner and reports
                # a partial result rather than a fake success.
                return 0 if runtime.mission.state.value == "finished" else 1
            return 0
        if task_board_enabled:
            from components.task_board_reader import TaskBoardConfig, TaskBoardReader
            from task_board_startup import acquire_task_board

            camera = int(args.task_board_camera) if args.task_board_camera.isdigit() else args.task_board_camera
            result = acquire_task_board(
                runtime, TaskBoardReader(TaskBoardConfig(debug_directory=args.task_board_debug_dir)),
                camera=camera, turn_rad=math.radians(args.task_board_turn_deg),
            )
            if not result.valid:
                logging.error("task-board acquisition failed: %s", result.reason)
                return 1
            if result.source == "fallback_1_2_1":
                logging.warning(
                    "task-board recognition failed; using fallback task=%s reason=%s",
                    result.task, result.reason,
                )
            else:
                logging.info("task-board task=%s confidence=%.3f votes=%d", result.task, result.confidence, result.votes)
        if args.goal_x is not None:
            runtime.mission.set_navigation_goal(
                NavigationGoal(args.goal_x, args.goal_y, args.goal_yaw)
            )
        if mode in {RuntimeMode.DRY_RUN, RuntimeMode.REPLAY}:
            results = runtime.run_replay() if mode is RuntimeMode.REPLAY and args.replay_file else runtime.run_steps(args.steps)
            logging.info("completed %d runtime steps; mission=%s", len(results), runtime.mission.state.value)
            return 0 if runtime.mission.state.value not in {"error", "safe_stop"} else 1
        runtime.run()
        return 0 if runtime.mission.state.value not in {"error", "safe_stop"} else 1
    except KeyboardInterrupt:
        # run() handles this itself. This catches interrupts during setup or
        # fixed-step execution while preserving the same cleanup order.
        runtime.mission.request_safe_stop("keyboard interrupt")
        runtime.close()
        return 130
    except Exception:
        if not competition_enabled:
            raise
        runtime.mission.request_safe_stop("competition task failed")
        logging.exception("competition stopped")
        return 1
    finally:
        try:
            runtime.close()
        finally:
            if competition_enabled and not simulate_payload:
                try:
                    if open_payload() is False:
                        logging.error('finally payload OPEN failed: returned False')
                except Exception:
                    logging.exception("finally payload OPEN failed")
            try:
                if on_runtime_ready is not None:
                    on_runtime_ready(None)
            finally:
                if previous_sigterm is not None:
                    signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    sys.exit(launch())
