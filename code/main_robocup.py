"""Independent production entry point for the RoboCup differential platform."""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys

from components.diagnostics_log import JsonlEventLogger
from components.navigation_common import NavigationGoal
from config.v2_runtime import RuntimeMode
from robocup_runtime import RuntimeReadinessError, build_runtime, load_runtime_config


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
    parser.add_argument("--steps", type=int, default=8, help="fixed dry-run/replay step count")
    parser.add_argument("--goal-x", type=float, help="optional navigation goal x in metres")
    parser.add_argument("--goal-y", type=float, help="optional navigation goal y in metres")
    parser.add_argument("--goal-yaw", type=float, help="optional final yaw in radians")
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


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.log_dir)
    if (args.goal_x is None) != (args.goal_y is None):
        logging.error("--goal-x and --goal-y must be provided together")
        return 2

    try:
        config = load_runtime_config(args.config)
        mode = RuntimeMode(args.mode)
        if not config.calibration.geometry_measured or not config.calibration.sensor_extrinsics_measured:
            logging.warning(
                "configuration contains unmeasured geometry or sensor mounts; autonomous hardware mission remains gated"
            )
        runtime = build_runtime(
            config,
            mode,
            replay_file=args.replay_file,
            mission_profile=args.mission_profile,
        )
        if args.log_dir is not None:
            runtime.event_logger = JsonlEventLogger(Path(args.log_dir) / "events.jsonl")
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
    finally:
        runtime.close()


if __name__ == "__main__":
    sys.exit(main())
