#!/usr/bin/env python3
"""280/420/250cm patrol; YOLO-triggered left-side payload detour or legacy beep."""

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
from components.relay_lcus import FakeLCUSRelay
from components.payload_task import prepare_payload
from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_factory import build_servo
from config.mission_profile import payload_config, route_speed_config
from components.servo_positioning import park_servo
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import build_runtime
from payload_detour import DetourSettings


def startup_countdown(relay, alarm, slot, seconds, *, verify=True, abort=lambda: False,
                      clock=time.monotonic, sleep=time.sleep):
    """Hold selected slot(s), sound a bounded alarm, leave motion unopened."""
    if not math.isfinite(seconds) or not 0 < seconds <= 60:
        raise ValueError("startup alarm must be between 0 and 60 seconds")
    try:
        relay.open()
        slots = (slot,) if isinstance(slot, int) else tuple(slot)
        if not slots or relay.all_off(verify=verify) is False:
            raise RuntimeError("startup relay state was not confirmed")
        for selected in slots:
            if not prepare_payload(relay, selected, verify=verify):
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
    parser.add_argument("--payload-sequence", type=int, nargs="+", choices=(1, 2, 3),
                        help="drop slots in order, default 1 2 3; mutually exclusive with --payload-slot")
    parser.add_argument("--release-hold-s", type=float, default=0.5)
    parser.add_argument("--release-mode", choices=("simulate", "relay"),
                        help="default: simulate; explicit filming mode defaults to real relay release")
    parser.add_argument("--target-class-name", default=None)
    parser.add_argument("--target-speed-cm-s", type=float, default=8,
                        help="patrol speed cap while a target is visible; normal acceleration limits apply")
    parser.add_argument("--speed-scale", type=float, default=1,
                        help="scale patrol, short moves, visible-target speed and turn targets; default 1")
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
                        help="drop mode: hold selected magnets, alarm for this duration, then start (0=disabled)")
    parser.add_argument("--confirm-motor-test", action="store_true")
    args = parser.parse_args(argv)
    if args.payload_slot is not None and args.payload_sequence is not None:
        parser.error("use either --payload-slot or --payload-sequence")
    slots = tuple(args.payload_sequence or ((args.payload_slot,) if args.payload_slot else (1, 2, 3)))
    if len(set(slots)) != len(slots):
        parser.error("--payload-sequence must contain distinct slots")
    args.payload_slot = slots[0]
    args.release_mode = args.release_mode or "simulate"
    if not math.isfinite(args.startup_alarm_seconds) or not 0 <= args.startup_alarm_seconds <= 60:
        parser.error("--startup-alarm-seconds must be between 0 and 60")
    if args.startup_alarm_seconds and args.target_action != "drop":
        parser.error("--startup-alarm-seconds requires --target-action drop")
    region = args.target_region or ("frame" if args.target_action == "drop" else "center")
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
        config = route_speed_config(config, args.speed_scale)
        config = payload_config(config, action=args.target_action,
                                release_mode=args.release_mode,
                                relay_port=args.relay_port, relay_channels=args.relay_channels)
        detour = (DetourSettings(payload_slot=args.payload_slot, release_hold_s=args.release_hold_s,
                                 verify_relay=config.relay.verify_writes,
                                 patrol_slow_speed_m_s=args.target_speed_cm_s * args.speed_scale / 100)
                  if args.target_action == "drop" else None)
    except ValueError as exc:
        parser.error(str(exc))
    servo = build_servo(config)
    if servo is None:
        parser.error("camera servo must be enabled in the measured config")
    servo.pulse_us_for(args.servo_angle_deg)  # Validate before touching hardware.
    if detour is not None and args.servo_angle_deg != 90:
        parser.error("horizontal alignment requires --servo-angle-deg 90")
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
    alarm = SoundLightAlarm() if args.target_action == "beep" or args.startup_alarm_seconds else None
    outcome_error = None
    try:
        runtime = build_runtime(config, RuntimeMode.HARDWARE_MISSION, event_logger=logger)
        runtime.record_event("test_route_speed_profile", scale=args.speed_scale,
                             translation_speed_scale=config.navigation.translation_speed_scale,
                             patrol_max_m_s=min(.15 * config.navigation.translation_speed_scale,
                                                config.drive.max_linear_speed_m_s),
                             target_max_m_s=detour.patrol_slow_speed_m_s if detour else None,
                             turn_max_rad_s=config.drive.max_angular_speed_rad_s,
                             lookahead_base_m=config.navigation.lookahead_m,
                             lookahead_speed_reference_m_s=config.navigation.lookahead_speed_reference_m_s,
                             lookahead_speed_gain_s=config.navigation.lookahead_speed_gain_s,
                             lookahead_max_extension_m=config.navigation.lookahead_max_extension_m,
                             lookahead_short_segment_m=config.navigation.lookahead_short_segment_m)
        print(f"[route] SPEED scale={args.speed_scale:g}; "
              f"patrol_max={min(.15 * config.navigation.translation_speed_scale, config.drive.max_linear_speed_m_s):g}m/s; "
              f"turn_max={config.drive.max_angular_speed_rad_s:g}rad/s", flush=True)
        if detour is not None and args.release_mode == "simulate":
            runtime.relay = FakeLCUSRelay(4)
        if detour is not None:
            runtime.record_event("test_route_release_mode", mode=args.release_mode, slots=slots)
            print(f"[route] RELEASE MODE {args.release_mode}; slots={slots}", flush=True)
        # Camera must be parked before the magnets/countdown, and stays at +90.
        park_servo(servo, args.servo_angle_deg)
        runtime.record_event("test_route_camera_hold", angle_deg=args.servo_angle_deg, pulse_us=servo.pulse_us)
        print(f"[route] SERVO HOLD {args.servo_angle_deg:g}deg, {servo.pulse_us}us", flush=True)
        if args.startup_alarm_seconds:
            runtime.record_event("startup_countdown_start", slots=slots, seconds=args.startup_alarm_seconds)
            print(f"[route] STARTUP: hold slots={slots}, alarm {args.startup_alarm_seconds:g}s", flush=True)
            startup_countdown(runtime.relay, alarm, slots, args.startup_alarm_seconds,
                              verify=config.relay.verify_writes,
                              abort=lambda: aborted.is_set() or stop_file.exists())
            runtime.record_event("startup_countdown_done", slots=slots)
            print("[route] STARTUP DONE; alarm OFF", flush=True)
        if alarm is not None:
            alarm.initialize(active=False)
        vision.start()
        run_route(runtime, vision, alarm,
                  abort=lambda: aborted.is_set() or stop_file.exists(),
                  max_seconds=args.max_seconds, detour=detour,
                  payload_slots=slots if detour is not None else None,
                  trigger_anywhere=detour is not None)
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
                ("relay_all_off",lambda: runtime.relay.all_off(verify=True)
                 if runtime is not None and runtime.relay is not None and runtime.relay.connected else None),
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
