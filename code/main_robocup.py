"""Independent production entry point for the RoboCup differential platform."""

from __future__ import annotations

import argparse
import logging
import math
from pathlib import Path
import sys

from components.diagnostics_log import JsonlEventLogger
from components.navigation_common import NavigationGoal
from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_runtime import RuntimeMode
from robocup_runtime import RuntimeReadinessError, build_runtime, load_runtime_config


DIRECT_COMPETITION_STAGES = {"task-board", "hc-send", "yellow-detect", "drop"}


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
    parser.add_argument("--competition", action="store_true", help="run the complete competition mission (hardware-mission)")
    parser.add_argument("--competition-stage", choices=(
        "full", "lane", "task-board", "hc-send", "corner1", "cross-lane",
        "yellow-detect", "yellow-search", "yellow-align", "drop-route", "drop", "corner2", "finish"),
        help="run only this stage; task-board/hc-send/yellow-detect/drop use direct hardware without localization runtime")
    parser.add_argument("--yellow-camera", help="competition camera index or stable device path")
    parser.add_argument("--yellow-model", type=Path, help="existing car YOLO weights path")
    parser.add_argument("--payload-slot", type=int, choices=(1, 2, 3), default=1,
                        help="slot for the standalone drop stage; full mission uses slot 1")
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

        ok = task.send_task_to_drone_once(TaskCounts(*task.TASK_BOARD_FALLBACK))
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
        from config.v2_factory import build_relay

        config = load_runtime_config(args.config)
        relay = build_relay(config, fake=False)
        if relay is None:
            logging.error("payload relay is disabled/unavailable")
            return 1
        ok = False
        try:
            relay.open()
            ok = task.drop_payload(relay, args.payload_slot)
        finally:
            # A direct test always requests all_off before closing an opened
            # relay, including when disconnect_on_shutdown is disabled.
            try:
                if relay.connected and relay.all_off(verify=config.relay.verify_writes) is False:
                    logging.error("direct drop all_off was not confirmed")
                    ok = False
            except Exception:
                logging.exception("direct drop all_off failed")
                ok = False
            finally:
                try:
                    relay.close()
                except Exception:
                    logging.exception("direct drop close failed")
                    ok = False
        return 0 if ok is True else 1
    raise ValueError(f"unsupported direct competition stage: {stage}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.log_dir)
    competition_enabled = args.competition or args.competition_stage is not None
    direct_stage = args.competition_stage in DIRECT_COMPETITION_STAGES
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
            return run_direct_competition_stage(args)
        except KeyboardInterrupt:
            logging.warning("direct competition stage interrupted")
            return 130
        except Exception:
            logging.exception("direct competition stage failed: %s", args.competition_stage)
            return 1

    try:
        config = load_runtime_config(args.config)
        mode = RuntimeMode(args.mode)
        if args.relative_slam or (mode is RuntimeMode.HARDWARE_MISSION and not args.localization_from_config):
            config = accepted_relative_slam_profile(config)
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
    except (RuntimeReadinessError, FileNotFoundError, ValueError, NotImplementedError) as exc:
        logging.error("cannot start RoboCup runtime: %s", exc)
        if isinstance(exc, RuntimeReadinessError) and args.log_dir is not None:
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

    try:
        if competition_enabled:
            from competition_task import run_competition_stage

            result = run_competition_stage(
                runtime, args.competition_stage or "full",
                task_board_camera=camera_value(args.task_board_camera),
                yellow_camera=camera_value(args.yellow_camera),
                weights=args.yellow_model, slot=args.payload_slot,
            )
            logging.info("competition stage completed: %s; result=%s",
                         args.competition_stage or "full",
                         result if isinstance(result, (bool, type(None))) else type(result).__name__)
            return 0 if runtime.mission.state.value not in {"error", "safe_stop"} else 1
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
        logging.exception("competition stopped")
        return 1
    finally:
        runtime.close()


if __name__ == "__main__":
    sys.exit(main())
