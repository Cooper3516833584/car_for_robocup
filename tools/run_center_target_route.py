#!/usr/bin/env python3
"""280/420/250cm patrol; YOLO-triggered left-side payload detour or legacy beep."""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
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
from components.relay_lcus import DEFAULT_RELAY_PORT, FakeLCUSRelay
from components.payload_task import prepare_payload
from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_factory import build_payload_demo_session, build_servo, configure_payload_relay
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import build_runtime
from hal.pwm import PWMBackendError
from payload_detour import DetourSettings
from payload_demo import run_payload_demo


def payload_config(config, *, action, release_mode="simulate", relay_port=None, relay_channels=None):
    """Validate explicit relay settings before any camera/servo/motor opens."""
    if action == "beep":
        if config.relay.enabled or relay_port is not None or relay_channels is not None:
            raise ValueError("beep mode requires the payload relay to be disabled")
        return config
    relay = config.relay
    if release_mode == "simulate":
        if relay_port is not None:
            raise ValueError("--relay-port requires explicit --release-mode relay")
        return replace(config, relay=replace(relay, enabled=False, channel_count=4,
                                             disconnect_on_shutdown=True))
    if relay_port is not None:
        if not relay_port.strip():
            raise ValueError("--relay-port must not be empty")
        relay = replace(relay, enabled=True, port=relay_port)
    if relay_channels is not None:
        relay = replace(relay, channel_count=relay_channels)
    if not relay.enabled:
        raise ValueError("drop mode requires an enabled relay in config or explicit --relay-port")
    if relay.channel_count < 4:
        raise ValueError("drop mode requires relay channels 2, 3 and 4")
    # Exiting a program must disconnect latched contacts even if a profile opted out.
    return configure_payload_relay(replace(config, relay=relay))


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


def startup_countdown(relay, alarm, slot, seconds, *, verify=True, abort=lambda: False,
                      clock=time.monotonic, sleep=time.sleep):
    """Hold only the selected magnet, sound a bounded alarm, leave motion unopened."""
    if not math.isfinite(seconds) or not 0 < seconds <= 60:
        raise ValueError("startup alarm must be between 0 and 60 seconds")
    try:
        relay.open()
        if relay.all_off(verify=verify) is False or not prepare_payload(relay, slot, verify=verify):
            raise RuntimeError("startup relay state was not confirmed")
        alarm.initialize(active=False)
        try:
            if abort():
                raise RuntimeError("startup countdown interrupted")
            alarm.on()
            if not alarm.is_active:
                raise RuntimeError("startup alarm active level was not confirmed")
            deadline = clock() + seconds
            while clock() < deadline:
                if abort():
                    raise RuntimeError("startup countdown interrupted")
                sleep(min(.1, max(0, deadline - clock())))
            if abort():
                raise RuntimeError("startup countdown interrupted")
        finally:
            if alarm.is_initialized:
                alarm.off()
        if alarm.is_active:
            raise RuntimeError("startup alarm inactive level was not confirmed")
    except BaseException:
        if relay.connected:
            relay.all_off(verify=verify)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "configs/robocup_diffdrive.toml"))
    parser.add_argument("--weights", type=Path, default=MODEL_PATH)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--target-action", choices=("drop", "beep"), default="drop",
                        help="default: payload detour; beep retains the original stop/1s alarm")
    parser.add_argument("--target-region", choices=("frame", "center"),
                        help="default: entire frame for drop, middle 50%% for beep")
    parser.add_argument("--payload-slot", type=int, choices=(1, 2, 3),
                        help="1=right-front CH2, 2=middle CH3, 3=left-front CH4; powered hold, OFF release")
    parser.add_argument("--release-hold-s", type=float, default=0.5)
    parser.add_argument("--release-mode", choices=("simulate", "relay"),
                        help="default: simulate; explicit filming mode defaults to real relay release")
    parser.add_argument("--demo-no-position-checks", action="store_true",
                        help="temporary filming mode: command/time travel without T265/SLAM position checks")
    parser.add_argument("--demo-speed-cm-s", type=float, default=15)
    parser.add_argument("--demo-turn-rad-s", type=float, default=0.4)
    parser.add_argument("--target-speed-cm-s", type=float, default=8,
                        help="patrol speed cap while a target is visible; normal acceleration limits apply")
    parser.add_argument("--drop-center-width-ratio", type=float, default=0.1,
                        help="horizontal trigger band, default middle 10%%; vertical position is unrestricted")
    parser.add_argument("--relay-port", help="explicit, confirmed LCUS device path; never auto-detected")
    parser.add_argument("--relay-channels", type=int, choices=(4, 8),
                        help="actual fitted LCUS board channel count; default from config")
    parser.add_argument("--servo-angle-deg", type=float, default=90,
                        help="repository calibrated angle: 0=mid pulse, +90=2500us")
    parser.add_argument("--log-dir", type=Path)
    parser.add_argument("--check-vision", action="store_true",
                        help="check camera/model freshness without opening motion, GPIO or servo")
    parser.add_argument("--max-seconds", type=float, default=300)
    parser.add_argument("--startup-alarm-seconds", type=float, default=0,
                        help="drop mode: hold selected magnet, alarm for this duration, then start (0=disabled)")
    parser.add_argument("--confirm-motor-test", action="store_true")
    args = parser.parse_args(argv)
    if args.demo_no_position_checks and args.target_action != "drop":
        parser.error("--demo-no-position-checks requires --target-action drop")
    args.payload_slot = args.payload_slot or (2 if args.demo_no_position_checks else 1)
    args.release_mode = args.release_mode or ("relay" if args.demo_no_position_checks else "simulate")
    if args.demo_no_position_checks and args.release_mode == "relay" and args.relay_port is None:
        args.relay_port = DEFAULT_RELAY_PORT
    if not math.isfinite(args.startup_alarm_seconds) or not 0 <= args.startup_alarm_seconds <= 60:
        parser.error("--startup-alarm-seconds must be between 0 and 60")
    if args.startup_alarm_seconds and args.target_action != "drop":
        parser.error("--startup-alarm-seconds requires --target-action drop")
    region = args.target_region or ("frame" if args.target_action == "drop" else "center")
    if args.check_vision:
        vision = YoloVision(args.weights, args.camera, target_region=region,
                            drop_center_width_ratio=args.drop_center_width_ratio,
                            target_class_name="yellow" if args.demo_no_position_checks else None)
        try:
            vision.start()
            deadline = time.monotonic() + 60
            while not vision.ready and time.monotonic() < deadline:
                time.sleep(0.05)
            entry = vision.poll(time.monotonic())
            print(f"[vision] READY; region={region}; target={entry}", flush=True)
            if args.target_action == "drop":
                _, visible, centered = vision.observe_drop(time.monotonic())
                print(f"[vision] visible={visible}; horizontal_center={centered}; "
                      f"center_width_ratio={args.drop_center_width_ratio:g}", flush=True)
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
        config = payload_config(config, action=args.target_action,
                                release_mode=args.release_mode,
                                relay_port=args.relay_port, relay_channels=args.relay_channels)
        detour = (DetourSettings(payload_slot=args.payload_slot, release_hold_s=args.release_hold_s,
                                 verify_relay=config.relay.verify_writes,
                                 patrol_slow_speed_m_s=args.target_speed_cm_s / 100)
                  if args.target_action == "drop" else None)
        if args.demo_no_position_checks:
            if not math.isfinite(args.demo_speed_cm_s) or not 0 < args.demo_speed_cm_s / 100 <= config.drive.max_linear_speed_m_s:
                raise ValueError("demo speed must be positive and within the configured drive limit")
            if not math.isfinite(args.demo_turn_rad_s) or not 0 < args.demo_turn_rad_s <= config.drive.max_angular_speed_rad_s:
                raise ValueError("demo turn speed must be positive and within the configured drive limit")
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
                        target_class_name="yellow" if args.demo_no_position_checks else None)
    alarm = SoundLightAlarm() if args.target_action == "beep" or args.startup_alarm_seconds else None
    try:
        runtime = (build_payload_demo_session(config, event_logger=logger)
                   if args.demo_no_position_checks else
                   build_runtime(config, RuntimeMode.HARDWARE_MISSION, event_logger=logger))
        if detour is not None and args.release_mode == "simulate":
            runtime.relay = FakeLCUSRelay(4)
        if detour is not None:
            runtime.record_event("test_route_release_mode", mode=args.release_mode, slot=args.payload_slot)
            print(f"[route] RELEASE MODE {args.release_mode}; slot={args.payload_slot}", flush=True)
        if args.startup_alarm_seconds:
            runtime.record_event("startup_countdown_start", slot=args.payload_slot, seconds=args.startup_alarm_seconds)
            print(f"[route] STARTUP: hold slot={args.payload_slot}, alarm {args.startup_alarm_seconds:g}s", flush=True)
            startup_countdown(runtime.relay, alarm, args.payload_slot, args.startup_alarm_seconds,
                              verify=config.relay.verify_writes,
                              abort=lambda: aborted.is_set() or stop_file.exists())
            runtime.record_event("startup_countdown_done", slot=args.payload_slot)
            print("[route] STARTUP DONE; alarm OFF", flush=True)
        # Park at the requested calibrated angle and leave PWM enabled throughout.
        park_servo(servo, args.servo_angle_deg)
        print(f"[route] SERVO HOLD {args.servo_angle_deg:g}deg, {servo.pulse_us}us", flush=True)
        if alarm is not None:
            alarm.initialize(active=False)
        vision.start()
        if args.demo_no_position_checks:
            print("[demo] YELLOW ONLY; POSITION CHECKS OFF; travel is estimated from command/time", flush=True)
            run_payload_demo(runtime, vision, detour,
                             abort=lambda: aborted.is_set() or stop_file.exists(),
                             max_seconds=args.max_seconds, speed_m_s=args.demo_speed_cm_s / 100,
                             turn_rad_s=args.demo_turn_rad_s)
        else:
            run_route(runtime, vision, alarm,
                      abort=lambda: aborted.is_set() or stop_file.exists(),
                      max_seconds=args.max_seconds, detour=detour)
        (args.log_dir / "result.txt").write_text("FINISHED\n")
        return 0
    except BaseException as exc:
        if runtime is not None:
            try:
                runtime.drive.stop()
            except Exception as stop_error:
                print(f"[route] cleanup failed: {stop_error}", file=sys.stderr)
        (args.log_dir / "result.txt").write_text(f"FAILED: {type(exc).__name__}: {exc}\n")
        print(f"[route] FAILED: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        for cleanup in (runtime.drive.stop if runtime is not None else lambda: None,
                        alarm.off if alarm is not None and alarm.is_initialized else lambda: None,
                        runtime.close if runtime is not None else lambda: None,
                        vision.close, lambda: servo.close(hold=True) if servo.is_running else None):
            try:
                cleanup()
            except Exception as exc:
                print(f"[route] cleanup failed: {exc}", file=sys.stderr, flush=True)
        logger.close()



if __name__ == "__main__":
    raise SystemExit(main())
