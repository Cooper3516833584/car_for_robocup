#!/usr/bin/env python3
"""Explicit hardware test: 260cm, left90, 380cm, left90, 220cm; YOLO stop/beep."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))

from center_target_route import MODEL_PATH, YoloVision, run_route
from components.diagnostics_log import JsonlEventLogger
from components.sound_light_alarm import SoundLightAlarm
from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_factory import build_servo
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import build_runtime
from hal.pwm import PWMBackendError


def park_servo(servo, angle, *, clock=time.monotonic, sleep=time.sleep):
    """Allow udev's group permissions to settle after the first PWM export."""
    deadline = clock() + 2.0
    while True:
        try:
            servo.start(home=False)
            break
        except PWMBackendError as exc:
            if not isinstance(exc.__cause__, PermissionError) or clock() >= deadline:
                raise
            sleep(0.05)
    servo.set_angle(angle, settle=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/robocup_diffdrive.toml"))
    parser.add_argument("--weights", type=Path, default=MODEL_PATH)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--servo-angle-deg", type=float, default=90,
                        help="repository calibrated angle: 0=mid pulse, +90=2500us")
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--check-vision", action="store_true",
                        help="check camera/model freshness without opening motion, GPIO or servo")
    parser.add_argument("--max-seconds", type=float, default=300)
    parser.add_argument("--confirm-motor-test", action="store_true")
    args = parser.parse_args(argv)
    if args.check_vision:
        vision = YoloVision(args.weights, args.camera)
        try:
            vision.start()
            deadline = time.monotonic() + 60
            while not vision.ready and time.monotonic() < deadline:
                time.sleep(0.05)
            entry = vision.poll(time.monotonic())
            print(f"[vision] READY; central_target={entry}", flush=True)
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
    if config.relay.enabled:
        parser.error("payload relay must be disabled for this test")
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
    vision = YoloVision(args.weights, args.camera)
    alarm = SoundLightAlarm()
    try:
        runtime = build_runtime(config, RuntimeMode.HARDWARE_MISSION, event_logger=logger)
        # Park at the requested calibrated angle and leave PWM enabled throughout.
        park_servo(servo, args.servo_angle_deg)
        print(f"[route] SERVO HOLD {args.servo_angle_deg:g}deg, {servo.pulse_us}us", flush=True)
        alarm.initialize(active=False)
        vision.start()
        run_route(runtime, vision, alarm,
                  abort=lambda: aborted.is_set() or stop_file.exists(),
                  max_seconds=args.max_seconds)
        (args.log_dir / "result.txt").write_text("FINISHED\n")
        return 0
    except BaseException as exc:
        if runtime is not None:
            runtime.drive.stop()
        (args.log_dir / "result.txt").write_text(f"FAILED: {type(exc).__name__}: {exc}\n")
        print(f"[route] FAILED: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if runtime is not None:
            runtime.drive.stop()
        try:
            if alarm.is_initialized:
                alarm.off()
        finally:
            try:
                if runtime is not None:
                    runtime.close()
            finally:
                vision.close()
                if servo.is_running:
                    servo.close(hold=True)
                logger.close()


if __name__ == "__main__":
    raise SystemExit(main())
