#!/usr/bin/env python3
"""Standalone CH3 test: +90 search drive, visual drop, fused return, exit.

Flow: hold the selected magnet -> camera servo to the search angle (+90) ->
drive forward along the current fused heading while YOLO looks for the target ->
the horizontally centred target stops the car and starts the shared fused
payload detour (7 cm advance, left 90, one measured side line, release, straight
reverse into the lane, road heading restored) -> stop, release the relay, exit.

No task board, no HC radio and no competition lane: the search line is derived
from the pose the operator placed the car at, not from the mission route.

    py -3 tools/run_ch3_drop_test.py --confirm-motor-test --release-mode simulate
        --drop-safe-m 0.47 --search-distance-m 3
        --log-dir logs/ch3-drop-test

Complete runs are real motor tests. Default release-mode simulate uses an
in-memory relay; relay mode holds before motion and releases at the drop pose.
--vision-preview opens just servo/camera to save stationary evidence.
"""

from __future__ import annotations

import argparse
import logging
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))

from competition_task import (perform_payload_detour, wait_for_fused_localization,
                              fine_center_on_road, DROP_SAFE_M, _open_yellow_camera)
from components.diagnostics_log import JsonlEventLogger
from components.payload_task import prepare_payload
from components.relay_lcus import FakeLCUSRelay
from components.servo_positioning import park_servo
from config.mission_profile import payload_config, route_speed_config
from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_factory import build_servo
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from mission_control import task_scope
from robocup_runtime import build_runtime
from target_patrol import MODEL_PATH, YoloVision, find_centered_target


def camera_value(value):
    return int(value) if str(value).isdigit() else value


def search_endpoints(runtime, distance_m):
    """Forward search line in the current fused frame, along the current yaw."""
    pose = wait_for_fused_localization(runtime).estimate.pose
    start = (pose.x_m, pose.y_m)
    return start, (start[0] + distance_m * math.cos(pose.yaw_rad),
                   start[1] + distance_m * math.sin(pose.yaw_rad))


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(ROOT / "configs/robocup_diffdrive.toml"))
    parser.add_argument("--weights", type=Path, default=MODEL_PATH)
    parser.add_argument("--camera", type=camera_value, default=0,
                        help="camera index or stable device path used at both angles")
    parser.add_argument("--target-class-name", default="yellow")
    parser.add_argument("--drop-center-width-ratio", type=float, default=0.1,
                        help="horizontal trigger band, default middle 10%%; vertical position is unrestricted")
    parser.add_argument("--payload-slot", type=int, choices=(1, 2, 3), default=2,
                        help="1=right-front CH2, 2=middle CH3 (default), 3=left-front CH4")
    parser.add_argument("--release-mode", choices=("simulate", "relay"), default="simulate",
                        help="simulate uses the in-memory relay; relay needs --relay-port or an enabled config")
    parser.add_argument("--relay-port", help="explicit, confirmed LCUS device path; never auto-detected")
    parser.add_argument("--relay-channels", type=int, choices=(4, 8),
                        help="actual fitted LCUS board channel count; default from config")
    parser.add_argument("--search-distance-m", type=float, default=3.0,
                        help="forward distance along the current fused heading while searching")
    parser.add_argument("--search-speed-cm-s", type=float, default=8,
                        help="speed cap while the target is visible; normal limits apply otherwise")
    parser.add_argument("--servo-angle-deg", type=float, default=90,
                        help="search angle: repository calibration, 0=mid pulse, +90=2500us")
    parser.add_argument("--speed-scale", type=float, default=1,
                        help="scale motion targets for this test; default 1")
    parser.add_argument("--startup-hold-s", type=float, default=0,
                        help="hold the magnet for this long before moving, to attach the payload")
    parser.add_argument("--log-dir", type=Path,
                        help="evidence directory; default logs/ch3-drop-test-<timestamp>")
    parser.add_argument("--max-seconds", type=float, default=180,
                        help="whole-test time limit, including the nested drop and return")
    parser.add_argument("--confirm-motor-test", action="store_true")
    parser.add_argument("--vision-preview", action="store_true",
                        help="stationary servo-0 snapshots only; no runtime, drive or relay")
    parser.add_argument("--drop-safe-m", type=float, default=DROP_SAFE_M,
                        help="measured safe fused travel from post-left-turn pivot")
    return parser


def vision_preview(args, parser):
    """Open just the borrowed servo/camera; do not construct a robot runtime."""
    from components import drop_target_vision as vision
    import json
    config = load_v2_config(args.config)
    servo = build_servo(config)
    if servo is None:
        parser.error("camera servo must be enabled for the stationary preview")
    log_dir = args.log_dir or Path("logs") / f"ch3-preview-{time.strftime('%Y%m%d-%H%M%S')}"
    log_dir.mkdir(parents=True, exist_ok=False)
    capture, owned = None, False
    try:
        park_servo(servo, 0)
        capture, owned = _open_yellow_camera(args.camera)
        observations = []
        for index in range(3):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("preview camera frame unavailable")
            observations.append(vision.save_observation(
                frame, log_dir / f"preview_{index}", args.target_class_name))
        feature = vision.stable_feature(observations)
        (log_dir / "preview.json").write_text(json.dumps(
            {"feature": feature, "observations": observations, "chosen_action": "observe_only",
             "yaw": None, "fused_x": None, "fused_y": None,
             "road_projection": None, "side_projection": None}, indent=2), encoding="utf-8")
        print(f"[ch3] stationary preview: {feature}; evidence={log_dir}", flush=True)
        return 0
    finally:
        try:
            if owned and capture is not None:
                capture.release()
        finally:
            servo.close(hold=True)


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.vision_preview:
        return vision_preview(args, parser)
    if not args.confirm_motor_test:
        parser.error("pass --confirm-motor-test for this explicitly requested hardware test")
    if not args.weights.is_file():
        parser.error(f"YOLO weights missing: {args.weights}")
    if args.drop_safe_m is None or not math.isfinite(args.drop_safe_m) or args.drop_safe_m <= 0:
        parser.error("provide the measured safe pivot-to-drop distance with --drop-safe-m")
    if args.servo_angle_deg != 90:
        parser.error("complete CH3 test requires the calibrated +90 search angle")
    if not math.isfinite(args.search_distance_m) or not 0.1 <= args.search_distance_m <= 20:
        parser.error("--search-distance-m must be between 0.1 and 20")
    if not math.isfinite(args.search_speed_cm_s) or args.search_speed_cm_s <= 0:
        parser.error("--search-speed-cm-s must be finite and positive")
    if not math.isfinite(args.startup_hold_s) or not 0 <= args.startup_hold_s <= 60:
        parser.error("--startup-hold-s must be between 0 and 60")
    if not math.isfinite(args.max_seconds) or not 1 <= args.max_seconds <= 600:
        parser.error("--max-seconds must be between 1 and 600")
    if not math.isfinite(args.drop_center_width_ratio) or not 0 < args.drop_center_width_ratio <= 1:
        parser.error("--drop-center-width-ratio must be in (0, 1]")
    try:
        config = accepted_relative_slam_profile(load_v2_config(args.config))
        config = route_speed_config(config, args.speed_scale)
        config = payload_config(config, action="drop", release_mode=args.release_mode,
                                relay_port=args.relay_port, relay_channels=args.relay_channels)
    except (ValueError, FileNotFoundError, NotImplementedError) as exc:
        parser.error(str(exc))
    servo = build_servo(config)
    if servo is None:
        parser.error("camera servo must be enabled in the measured config")
    servo.pulse_us_for(args.servo_angle_deg)  # Validate the angle before any hardware opens.
    log_dir = args.log_dir or Path("logs") / f"ch3-drop-test-{time.strftime('%Y%m%d-%H%M%S')}"
    try:
        log_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        parser.error(f"log directory already exists: {log_dir}")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    stop_file = log_dir / "STOP"
    aborted = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: aborted.set())
    (log_dir / "pid").write_text(str(os.getpid()))
    logger = JsonlEventLogger(log_dir / "events.jsonl")
    slow_speed_m_s = args.search_speed_cm_s / 100 * config.navigation.translation_speed_scale
    runtime, vision = None, None
    outcome_error = None
    try:
        runtime = build_runtime(config, RuntimeMode.HARDWARE_MISSION, event_logger=logger)
        if args.release_mode == "simulate":
            runtime.relay = FakeLCUSRelay(4)
        runtime.record_event("ch3_drop_test_start", payload_slot=args.payload_slot,
                             camera=str(args.camera), release_mode=args.release_mode,
                             target_class_name=args.target_class_name,
                             search_distance_m=args.search_distance_m,
                             search_speed_m_s=slow_speed_m_s, speed_scale=args.speed_scale,
                             servo_angle_deg=args.servo_angle_deg)
        print(f"[ch3] RELEASE MODE {args.release_mode}; slot={args.payload_slot}", flush=True)
        wait_for_fused_localization(runtime)  # Starts sensors, fusion and the relay port.
        if runtime.relay is None:
            raise RuntimeError("payload relay unavailable for this release mode")
        if not prepare_payload(runtime.relay, args.payload_slot, verify=config.relay.verify_writes):
            raise RuntimeError("payload holding state was not confirmed")
        runtime.record_event("payload_hold_ready", slot=args.payload_slot)
        print(f"[ch3] HOLD slot={args.payload_slot} ready", flush=True)
        park_servo(servo, args.servo_angle_deg)
        print(f"[ch3] SERVO {args.servo_angle_deg:g}deg, {servo.pulse_us}us", flush=True)

        deadline = runtime.clock() + args.max_seconds

        def stop_requested():
            return aborted.is_set() or stop_file.exists()

        def guard():
            if stop_requested():
                raise RuntimeError("ch3 drop test interrupted")
            if runtime.clock() >= deadline:
                raise RuntimeError("ch3 drop test time limit expired")

        if args.startup_hold_s:
            print(f"[ch3] STARTUP HOLD {args.startup_hold_s:g}s; attach the payload", flush=True)
            hold_until = runtime.clock() + args.startup_hold_s
            while runtime.clock() < hold_until:
                guard()
                time.sleep(min(0.1, max(0.0, hold_until - runtime.clock())))
        start_xy, end_xy = search_endpoints(runtime, args.search_distance_m)
        runtime.record_event("ch3_drop_test_search", start_xy=start_xy, end_xy=end_xy,
                             pose_reference="fused")
        print(f"[ch3] SEARCH {start_xy[0]:.3f},{start_xy[1]:.3f} -> "
              f"{end_xy[0]:.3f},{end_xy[1]:.3f} at {slow_speed_m_s:g}m/s when visible", flush=True)
        vision = YoloVision(args.weights, args.camera, target_region="frame",
                            target_class_name=args.target_class_name or None,
                            drop_center_width_ratio=args.drop_center_width_ratio)
        with task_scope(guard):
            guard()
            vision.start()
            box = find_centered_target(runtime, vision, start_xy, end_xy,
                                       slow_speed_m_s=slow_speed_m_s, abort=stop_requested,
                                       max_seconds=args.max_seconds, clock=runtime.clock,
                                       sleep=time.sleep)
            if box is not None:
                box = fine_center_on_road(runtime, vision, box)
            # The 0 deg refinement below opens the same camera: release it first.
            vision.close()
            if box is None:
                runtime.record_event("ch3_drop_test_not_found", start_xy=start_xy, end_xy=end_xy)
                print("[ch3] TARGET NOT FOUND on the search line", file=sys.stderr, flush=True)
                (log_dir / "result.txt").write_text("NOT_FOUND\n")
                return 1
            runtime.record_event("ch3_drop_test_target", box=box)
            print(f"[ch3] TARGET CENTRED {box}", flush=True)
            runtime.record_event("ch3_drop_test_drop_start", slot=args.payload_slot)
            print("[ch3] DROP: 7cm, left90, 0deg refine, release, fused return", flush=True)
            returned = perform_payload_detour(runtime, args.payload_slot, servo=servo,
                                              camera=args.camera,
                                              color=args.target_class_name or "yellow",
                                              safe_distance_m=args.drop_safe_m)
            runtime.record_event("ch3_drop_test_drop_done", slot=args.payload_slot, pose=returned,
                                 pose_reference="fused")
            print(f"[ch3] DROP DONE AND RETURNED to {returned.x_m:.3f},{returned.y_m:.3f} "
                  f"yaw={returned.yaw_rad:.3f}rad", flush=True)
        (log_dir / "result.txt").write_text("FINISHED\n")
        return 0
    except BaseException as exc:
        outcome_error = f"{type(exc).__name__}: {exc}"
        if runtime is not None:
            try:
                runtime.drive.stop()
            except Exception as stop_error:
                print(f"[ch3] drive stop failed: {stop_error}", file=sys.stderr)
        (log_dir / "result.txt").write_text(f"FAILED: {outcome_error}\n")
        print(f"[ch3] FAILED: {outcome_error}", file=sys.stderr, flush=True)
        return 1
    finally:
        cleanup_error = None
        for label, cleanup in (
                ("drive_stop", runtime.drive.stop if runtime is not None else lambda: None),
                ("relay_all_off", lambda: runtime.relay.all_off(verify=True)
                 if runtime is not None and runtime.relay is not None and runtime.relay.connected
                 else None),
                ("runtime_close", runtime.close if runtime is not None else lambda: None),
                ("vision_close", vision.close if vision is not None else lambda: None),
                ("servo_close", lambda: servo.close(hold=True) if servo.is_running else None)):
            try:
                if cleanup() is False:
                    raise RuntimeError(f"{label} was not confirmed")
            except Exception as exc:
                cleanup_error = cleanup_error or exc
                print(f"[ch3] {label} cleanup failed: {exc}", file=sys.stderr, flush=True)
                try:
                    logger.emit({"type": "ch3_drop_test_cleanup_failed", "resource": label,
                                 "reason": str(exc)}, priority=True)
                except Exception:
                    pass
        try:
            logger.close()
        except Exception as exc:
            cleanup_error = cleanup_error or exc
        if cleanup_error is not None and outcome_error is None:
            (log_dir / "result.txt").write_text(f"FAILED: cleanup: {cleanup_error}\n")
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
