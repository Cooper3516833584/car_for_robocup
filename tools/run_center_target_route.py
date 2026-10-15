#!/usr/bin/env python3
"""280/420/250cm fused patrol and camera/beep test. Payload tests use run_ch3_drop_test.py."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))

from target_patrol import MODEL_PATH, YoloVision, run_route
from components.diagnostics_log import JsonlEventLogger
from components.sound_light_alarm import SoundLightAlarm
from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_factory import build_servo
from config.mission_profile import route_speed_config
from components.servo_positioning import park_servo
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import build_runtime


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/robocup_diffdrive.toml"))
    parser.add_argument("--weights", type=Path, default=MODEL_PATH)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--target-action", choices=("beep",), default="beep",
                        help="stop and sound the alarm on visual targets")
    parser.add_argument("--target-region", choices=("frame", "center"), default="center",
                        help="default: middle 50%% for the beep test")
    parser.add_argument("--target-class-name", default=None)
    parser.add_argument("--target-speed-cm-s", type=float, default=8,
                        help="patrol speed cap while a target is visible; normal acceleration limits apply")
    parser.add_argument("--speed-scale", type=float, default=1,
                        help="scale patrol, short moves, visible-target speed and turn targets; default 1")
    parser.add_argument("--drop-center-width-ratio", type=float, default=0.1,
                        help="horizontal trigger band, default middle 10%%; vertical position is unrestricted")
    parser.add_argument("--servo-angle-deg", type=float, default=90,
                        help="repository calibrated angle: 0=mid pulse, +90=2500us")
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--check-vision", action="store_true",
                        help="check camera/model freshness without opening motion, GPIO or servo")
    parser.add_argument("--max-seconds", type=float, default=600)
    parser.add_argument("--confirm-motor-test", action="store_true")
    args = parser.parse_args(argv)
    region = args.target_region
    if args.check_vision:
        vision = YoloVision(args.weights, args.camera, target_region=region,
                            drop_center_width_ratio=args.drop_center_width_ratio,
                            target_class_name=args.target_class_name)
        try:
            vision.start()
            deadline = time.monotonic() + 60
            while not vision.ready and time.monotonic() < deadline:
                time.sleep(0.05)
            entry = vision.poll(time.monotonic())
            print(f"[vision] READY; region={region}; target={entry}", flush=True)
            return 0
        except Exception as exc:
            print(f"[vision] FAILED: {exc}", file=sys.stderr, flush=True)
            return 1
        finally:
            vision.close()
    if not args.confirm_motor_test:
        parser.error("pass --confirm-motor-test for this explicitly requested hardware route")
    if args.log_dir is None:
        parser.error("--log-dir is required for a motor test")
    if not 1 <= args.max_seconds <= 600:
        parser.error("--max-seconds must be between 1 and 600")
    config = accepted_relative_slam_profile(load_v2_config(args.config))
    try:
        config = route_speed_config(config, args.speed_scale)
    except ValueError as exc:
        parser.error(str(exc))
    servo = build_servo(config)
    if servo is None:
        parser.error("camera servo must be enabled in the measured config")
    servo.pulse_us_for(args.servo_angle_deg)  # Validate before touching hardware.
    if not args.weights.is_file():
        parser.error(f"YOLO weights missing: {args.weights}")
    args.log_dir.mkdir(parents=True, exist_ok=False)
    stop_file = args.log_dir / "STOP"
    aborted = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: aborted.set())
    (args.log_dir / "pid").write_text(str(os.getpid()))
    logger = JsonlEventLogger(args.log_dir / "events.jsonl")
    runtime = None
    vision = YoloVision(args.weights, args.camera, target_region=region,
                        drop_center_width_ratio=args.drop_center_width_ratio,
                        target_class_name=args.target_class_name)
    alarm = SoundLightAlarm()
    outcome_error = None
    try:
        runtime = build_runtime(config, RuntimeMode.HARDWARE_MISSION, event_logger=logger)
        runtime.record_event("test_route_speed_profile", scale=args.speed_scale,
                             translation_speed_scale=config.navigation.translation_speed_scale,
                             patrol_max_m_s=min(.15 * config.navigation.translation_speed_scale,
                                                config.drive.max_linear_speed_m_s),
                             turn_max_rad_s=config.drive.max_angular_speed_rad_s,
                             lookahead_base_m=config.navigation.lookahead_m,
                             lookahead_speed_reference_m_s=config.navigation.lookahead_speed_reference_m_s,
                             lookahead_speed_gain_s=config.navigation.lookahead_speed_gain_s,
                             lookahead_max_extension_m=config.navigation.lookahead_max_extension_m,
                             lookahead_short_segment_m=config.navigation.lookahead_short_segment_m)
        print(f"[route] SPEED scale={args.speed_scale:g}; "
              f"patrol_max={min(.15 * config.navigation.translation_speed_scale, config.drive.max_linear_speed_m_s):g}m/s; "
              f"turn_max={config.drive.max_angular_speed_rad_s:g}rad/s", flush=True)
        park_servo(servo, args.servo_angle_deg)
        runtime.record_event("test_route_camera_hold", angle_deg=args.servo_angle_deg, pulse_us=servo.pulse_us)
        print(f"[route] SERVO HOLD {args.servo_angle_deg:g}deg, {servo.pulse_us}us", flush=True)
        if alarm is not None:
            alarm.initialize(active=False)
        vision.start()
        run_route(runtime, vision, alarm,
                  abort=lambda: aborted.is_set() or stop_file.exists(),
                  max_seconds=args.max_seconds)
        (args.log_dir / "result.txt").write_text("FINISHED\n")
        return 0
    except BaseException as exc:
        outcome_error = f"{type(exc).__name__}: {exc}"
        if runtime is not None:
            try:
                runtime.drive.stop()
            except Exception as stop_error:
                print(f"[route] cleanup failed: {stop_error}", file=sys.stderr)
        (args.log_dir / "result.txt").write_text(f"FAILED: {type(exc).__name__}: {exc}\n")
        print(f"[route] FAILED: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        cleanup_error = None
        for label, cleanup in (
                ("drive_stop",runtime.drive.stop if runtime is not None else lambda: None),
                ("alarm_off",alarm.off if alarm is not None and alarm.is_initialized else lambda: None),
                ("runtime_close",runtime.close if runtime is not None else lambda: None),
                ("vision_close",vision.close),
                ("servo_close",lambda: servo.close(hold=True) if servo.is_running else None)):
            try:
                if cleanup() is False:
                    raise RuntimeError(f"{label} was not confirmed")
            except Exception as exc:
                cleanup_error = cleanup_error or exc
                print(f"[route] {label} cleanup failed: {exc}",file=sys.stderr,flush=True)
                try:
                    logger.emit({"type":"route_cleanup_failed","resource":label,"reason":str(exc)},priority=True)
                except Exception:
                    pass
        try:
            logger.close()
        except Exception as exc:
            cleanup_error = cleanup_error or exc
        if cleanup_error is not None and outcome_error is None:
            (args.log_dir / "result.txt").write_text(f"FAILED: cleanup: {cleanup_error}\n")
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
