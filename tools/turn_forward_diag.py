#!/usr/bin/env python3
"""Supervised reproduction of the post-left-turn straight-start drift."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from components.basic_motion_controller import MotionActionState  # noqa: E402
from components.diagnostics_log import JsonlEventLogger  # noqa: E402
from config.relative_slam_profile import accepted_relative_slam_profile  # noqa: E402
from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config  # noqa: E402
from config.v2_runtime import RuntimeMode, runtime_constraints  # noqa: E402
from core.frames import normalize_angle_rad  # noqa: E402
from robocup_runtime import RobocupMissionState, build_runtime  # noqa: E402
from tools.closed_loop_motion import _checked_step, _check_step, _sample_dict  # noqa: E402
from tools.motion_test_common import unwrap_delta  # noqa: E402

PERIOD_S = 0.05
TURN_TIMEOUT_S = 20.0
STRAIGHT_TIMEOUT_S = 10.0
SETTLE_AFTER_TURN_S = 0.20
MAX_FROM_START_M = 0.50


def run(runtime, config, *, abort, clock=time.monotonic, sleep=time.sleep,
        settle_after_turn_s=SETTLE_AFTER_TURN_S, skip_turn=False, turn_direction="left",
        startup_bias_rad_s=0.0, startup_duration_s=0.0):
    if not math.isfinite(settle_after_turn_s) or not 0.20 <= settle_after_turn_s <= 3.0:
        raise ValueError("turn settle interval must be between 0.20 and 3.0 seconds")
    if turn_direction not in {"left", "right"}:
        raise ValueError("turn direction must be left or right")
    runtime.start()
    ready_until = clock() + 30.0
    origin = None
    stable_since = None
    while clock() < ready_until:
        if abort():
            raise RuntimeError("operator aborted before motor start")
        step, sample = _checked_step(runtime, config)
        if sample is not None and step.mission_state is RobocupMissionState.READY:
            if (origin is None
                    or math.hypot(sample.x_m-origin.x_m, sample.y_m-origin.y_m) > 0.01
                    or abs(unwrap_delta(origin.yaw_rad, sample.yaw_rad)) > math.radians(1.0)):
                origin, stable_since = sample, clock()
            elif clock() - stable_since >= 5.0:
                break
        else:
            origin, stable_since = None, None
        sleep(PERIOD_S)
    else:
        raise TimeoutError("fresh T265+SLAM pose did not stay stable for 5 seconds")

    start = sample
    previous = start
    turn_samples = [start]
    if skip_turn:
        runtime.motion.stop()
        runtime.drive.stop()
        runtime.record_event("turn_forward_diag_turn_skipped", pose=start)
    else:
        stage = "%s_90deg" % turn_direction
        sign = 1.0 if turn_direction == "left" else -1.0
        target_yaw = normalize_angle_rad(start.yaw_rad + sign * math.pi / 2.0)
        runtime.motion.rotate_to(target_yaw)
        runtime.record_event("turn_forward_diag_stage_start", stage=stage, pose=start)
        deadline = clock() + TURN_TIMEOUT_S
        while clock() < deadline:
            if abort():
                raise RuntimeError("operator aborted during %s turn" % turn_direction)
            step, sample = _checked_step(runtime, config, allow_pending=True)
            if sample is None:
                raise RuntimeError("fused pose lost during %s turn" % turn_direction)
            _check_step(previous, sample, start)
            previous = sample
            turn_samples.append(sample)
            if step.motion is not None and step.motion.state is MotionActionState.SUCCEEDED:
                runtime.drive.stop()
                runtime.record_event("turn_forward_diag_stage_done", stage=stage, pose=sample)
                break
            if step.motion is not None and step.motion.state is not MotionActionState.RUNNING:
                raise RuntimeError("%s turn ended in %s" % (turn_direction, step.motion.state.value))
            sleep(PERIOD_S)
        else:
            raise TimeoutError("%s turn exceeded its bounded deadline" % turn_direction)

    # Match the production detour's stopped interval, while continuing sensor updates.
    hold_until = clock() + settle_after_turn_s
    while clock() < hold_until:
        if abort():
            raise RuntimeError("operator aborted while settling after turn")
        step, sample = _checked_step(runtime, config)
        if sample is None:
            raise RuntimeError("fused pose lost while settling after turn")
        _check_step(previous, sample, start)
        previous = sample
        sleep(min(PERIOD_S, max(0.0, hold_until-clock())))

    straight_start = sample
    # A completed motion leaves the mission in TARGET_OPERATION. Resume it
    # before submitting the next action, as the production payload detour does.
    runtime.motion.stop()
    if runtime.mission.state is RobocupMissionState.TARGET_OPERATION:
        runtime.mission.on_payload_action_done()
    runtime.record_event("turn_forward_diag_stage_start", stage="forward_20cm",
                         pose=straight_start, heading_reference_rad=straight_start.yaw_rad)
    runtime.motion.drive_distance(
        0.20,
        lateral_tolerance_m=config.navigation.position_tolerance_m,
        heading_yaw_rad=straight_start.yaw_rad,
        startup_yaw_bias_rad_s=(0.0 if skip_turn else
                               -startup_bias_rad_s if turn_direction == "left" else startup_bias_rad_s),
        startup_duration_s=startup_duration_s,
    )
    deadline = clock() + STRAIGHT_TIMEOUT_S
    previous = straight_start
    straight_samples = [straight_start]
    while clock() < deadline:
        if abort():
            raise RuntimeError("operator aborted during forward pulse")
        step, sample = _checked_step(runtime, config, allow_pending=True)
        if sample is None:
            raise RuntimeError("fused pose lost during forward pulse")
        _check_step(previous, sample, start)
        if math.hypot(sample.x_m-start.x_m, sample.y_m-start.y_m) > MAX_FROM_START_M:
            raise RuntimeError("vehicle left the bounded diagnostic area")
        previous = sample
        straight_samples.append(sample)
        if step.motion is not None and step.motion.state is MotionActionState.SUCCEEDED:
            runtime.drive.stop()
            runtime.record_event("turn_forward_diag_stage_done", stage="forward_20cm", pose=sample)
            break
        if step.motion is not None and step.motion.state is not MotionActionState.RUNNING:
            raise RuntimeError("forward pulse ended in %s" % step.motion.state.value)
        sleep(PERIOD_S)
    else:
        raise TimeoutError("20 cm forward pulse exceeded its bounded deadline")

    dx, dy = sample.x_m-straight_start.x_m, sample.y_m-straight_start.y_m
    c, s = math.cos(straight_start.yaw_rad), math.sin(straight_start.yaw_rad)
    return {
        "start": _sample_dict(start), "turn_end": _sample_dict(turn_samples[-1]),
        "straight_start": _sample_dict(straight_start), "straight_end": _sample_dict(sample),
        "turn_yaw_rad": sum(unwrap_delta(a.yaw_rad, b.yaw_rad)
                             for a, b in zip(turn_samples, turn_samples[1:])),
        "straight_along_m": dx*c + dy*s,
        "straight_lateral_m": -dx*s + dy*c,
        "straight_yaw_change_rad": unwrap_delta(straight_start.yaw_rad, sample.yaw_rad),
        "straight_samples": len(straight_samples),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True, help="new directory outside the checkout")
    parser.add_argument("--confirm-motor-test", action="store_true")
    parser.add_argument("--confirm-area-clear", action="store_true")
    parser.add_argument("--confirm-estop-ready", action="store_true")
    parser.add_argument("--settle-after-turn-s", type=float, default=SETTLE_AFTER_TURN_S,
                        help="bounded zero-output interval for transition A/B diagnostics (0.20..3.0 s)")
    parser.add_argument("--skip-turn", action="store_true",
                        help="straight-only baseline with identical forward control and limits")
    parser.add_argument("--turn-direction", choices=("left", "right"), default="left",
                        help="turn before forward motion; ignored with --skip-turn")
    parser.add_argument("--path-yaw-gain", type=float,
                        help="diagnostic-only straight feedback gain (0.5..8.0); retains turn gain")
    parser.add_argument("--startup-bias-rad-s", type=float, default=0.0,
                        help="counter-turn startup bias magnitude (0..0.4 rad/s)")
    parser.add_argument("--startup-duration-s", type=float, default=0.0,
                        help="linearly fading startup bias interval (0..2 s)")
    args = parser.parse_args(argv)
    if sys.platform != "linux":
        parser.error("this diagnostic must run on the car's Linux host")
    if not (args.confirm_motor_test and args.confirm_area_clear and args.confirm_estop_ready):
        parser.error("motor, clear-area and physical-estop confirmations are required")
    if not math.isfinite(args.settle_after_turn_s) or not 0.20 <= args.settle_after_turn_s <= 3.0:
        parser.error("--settle-after-turn-s must be between 0.20 and 3.0 seconds")
    if args.path_yaw_gain is not None and (
            not math.isfinite(args.path_yaw_gain) or not 0.5 <= args.path_yaw_gain <= 8.0):
        parser.error("--path-yaw-gain must be between 0.5 and 8.0")
    if (not math.isfinite(args.startup_bias_rad_s) or not 0.0 <= args.startup_bias_rad_s <= 0.4
            or not math.isfinite(args.startup_duration_s) or not 0.0 <= args.startup_duration_s <= 2.0
            or (args.startup_bias_rad_s > 0.0 and args.startup_duration_s <= 0.0)):
        parser.error("startup bias requires a magnitude in 0..0.4 and positive duration up to 2 seconds")
    config_path, output = Path(args.config).resolve(), Path(args.output).resolve()
    if config_path == DEFAULT_V2_CONFIG.resolve():
        parser.error("example configuration is not a measured car profile")
    if output == ROOT or ROOT in output.parents or output.exists():
        parser.error("output must be a new directory outside the checkout")
    config = accepted_relative_slam_profile(load_v2_config(config_path))
    if args.path_yaw_gain is not None:
        config = replace(config, navigation=replace(config.navigation, path_yaw_gain=args.path_yaw_gain))
    if config.relay.enabled:
        parser.error("disable the payload relay for this drive test")
    limits = runtime_constraints(config, RuntimeMode.HARDWARE_MISSION)
    config = replace(config, drive=replace(
        config.drive,
        max_linear_speed_m_s=min(config.drive.max_linear_speed_m_s, limits.max_linear_speed_m_s),
        max_angular_speed_rad_s=min(config.drive.max_angular_speed_rad_s, limits.max_angular_speed_rad_s),
        max_wheel_speed_m_s=min(config.drive.max_wheel_speed_m_s, limits.max_wheel_speed_m_s),
    ))
    output.mkdir(parents=True, exist_ok=False)
    logger = JsonlEventLogger(output / "events.jsonl")
    runtime = build_runtime(config, RuntimeMode.HARDWARE_MISSION, event_logger=logger)
    aborted = [False]
    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            signal.signal(sig, lambda *_: aborted.__setitem__(0, True))
    result, error = None, None
    try:
        result = run(runtime, config, abort=lambda: aborted[0],
                     settle_after_turn_s=args.settle_after_turn_s, skip_turn=args.skip_turn,
                     turn_direction=args.turn_direction, startup_bias_rad_s=args.startup_bias_rad_s,
                     startup_duration_s=args.startup_duration_s)
    except BaseException as exc:
        error = "%s: %s" % (type(exc).__name__, exc)
        runtime.mission.request_safe_stop(error)
    finally:
        runtime.close()
        logger.close()
    summary = {"valid": error is None, "error": error, "result": result,
               "parameters": {"skip_turn": args.skip_turn,
                              "path_yaw_gain": config.navigation.path_yaw_gain,
                              "startup_bias_rad_s": args.startup_bias_rad_s,
                              "startup_duration_s": args.startup_duration_s,
                              "turn_direction": args.turn_direction,
                              "settle_after_turn_s": args.settle_after_turn_s},
               "dropped_events": logger.dropped_events, "log_write_error": logger.write_error}
    (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if error is None else 1


if __name__ == "__main__":
    raise SystemExit(main())
