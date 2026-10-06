#!/usr/bin/env python3
"""Run one bounded BasicMotionController action with fused-pose diagnostics.

This is a supervised hardware-mission tool. Importing it opens no devices.
Start the repository's SLAM sidecar separately and verify the physical stop.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import platform
import signal
import subprocess
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
from tools.drive_calibration import accepted_fused_sample  # noqa: E402
from tools.motion_test_common import unwrap_delta  # noqa: E402

PERIOD_S = 0.05
MAX_OFFSET_M = 0.50
MAX_ANGLE_RAD = math.pi / 2.0
MAX_POSE_STEP_M = 0.10
MAX_YAW_STEP_RAD = math.radians(20.0)
MAX_FROM_START_M = 0.75
SETTLE_STEP_M = 0.003
SETTLE_STEP_RAD = math.radians(0.3)
SETTLE_STABLE_S = 0.25
PREFLIGHT_STABLE_S = 10.0
PREFLIGHT_DRIFT_M = 0.01
PREFLIGHT_DRIFT_RAD = math.radians(1.0)


@dataclass(frozen=True)
class ActionRequest:
    name: str
    distance_m: float = 0.0
    angle_rad: float = 0.0
    forward_m: float = 0.0
    left_m: float = 0.0


@dataclass(frozen=True)
class ActionTarget:
    method: str
    args: tuple
    goal_x_m: float | None = None
    goal_y_m: float | None = None
    goal_yaw_rad: float | None = None


def _bounded(value: float, name: str, limit: float) -> float:
    value = float(value)
    if not math.isfinite(value) or not 0.0 < abs(value) <= limit:
        raise ValueError("%s must be nonzero, finite and at most %.3f" % (name, limit))
    return value


def request_from_args(args) -> ActionRequest:
    name = args.action
    if name == "drive-distance":
        return ActionRequest(name, distance_m=_bounded(args.cm / 100.0, "distance_m", MAX_OFFSET_M))
    if name in {"rotate", "rotate-to"}:
        return ActionRequest(name, angle_rad=_bounded(math.radians(args.deg), "angle_rad", MAX_ANGLE_RAD))
    forward = float(args.forward_cm) / 100.0
    left = float(args.left_cm) / 100.0
    if not all(math.isfinite(v) for v in (forward, left)) or not 0.0 < math.hypot(forward, left) <= MAX_OFFSET_M:
        raise ValueError("relative point must be finite, nonzero and within 50 cm")
    angle = 0.0
    if name == "navigate-to-pose":
        angle = _bounded(math.radians(args.yaw_deg), "yaw_delta_rad", MAX_ANGLE_RAD)
    return ActionRequest(name, angle_rad=angle, forward_m=forward, left_m=left)


def target_from_start(pose, request: ActionRequest) -> ActionTarget:
    yaw = pose.yaw_rad
    c, s = math.cos(yaw), math.sin(yaw)
    x = pose.x_m + request.forward_m * c - request.left_m * s
    y = pose.y_m + request.forward_m * s + request.left_m * c
    if request.name == "drive-distance":
        gx = pose.x_m + request.distance_m * c
        gy = pose.y_m + request.distance_m * s
        return ActionTarget("drive_distance", (request.distance_m,), gx, gy)
    if request.name == "rotate":
        gyaw = normalize_angle_rad(yaw + request.angle_rad)
        return ActionTarget("rotate", (request.angle_rad,), pose.x_m, pose.y_m, gyaw)
    if request.name == "rotate-to":
        gyaw = normalize_angle_rad(yaw + request.angle_rad)
        return ActionTarget("rotate_to", (gyaw,), pose.x_m, pose.y_m, gyaw)
    if request.name == "face-point":
        gyaw = math.atan2(y - pose.y_m, x - pose.x_m)
        return ActionTarget("face_point", (x, y), pose.x_m, pose.y_m, gyaw)
    if request.name == "drive-to":
        return ActionTarget("drive_to", (x, y), x, y)
    if request.name == "follow-segment":
        return ActionTarget("follow_segment", ((pose.x_m, pose.y_m), (x, y)), x, y)
    if request.name == "navigate-to":
        return ActionTarget("navigate_to", (x, y), x, y)
    if request.name == "navigate-to-pose":
        gyaw = normalize_angle_rad(yaw + request.angle_rad)
        return ActionTarget("navigate_to_pose", (x, y, gyaw), x, y, gyaw)
    raise ValueError("unknown motion action: %s" % request.name)


def _sample_dict(sample) -> dict:
    return {"x_m": sample.x_m, "y_m": sample.y_m, "yaw_rad": sample.yaw_rad,
            "t265_age_s": sample.t265_age_s, "slam_age_s": sample.slam_age_s,
            "t265_confidence": sample.t265_confidence, "source_flags": sample.source_flags}


def _check_step(previous, current, start) -> None:
    if previous is not None:
        if (math.hypot(current.x_m - previous.x_m, current.y_m - previous.y_m) > MAX_POSE_STEP_M
                or abs(unwrap_delta(previous.yaw_rad, current.yaw_rad)) > MAX_YAW_STEP_RAD):
            raise RuntimeError("fused pose jumped during the action")
    if math.hypot(current.x_m - start.x_m, current.y_m - start.y_m) > MAX_FROM_START_M:
        raise RuntimeError("vehicle left the bounded test area")


def measurements(start, action_end, settled, samples, request: ActionRequest, target: ActionTarget) -> dict:
    c, s = math.cos(start.yaw_rad), math.sin(start.yaw_rad)
    dx, dy = settled.x_m - start.x_m, settled.y_m - start.y_m
    longitudinal = dx * c + dy * s
    lateral = -dx * s + dy * c
    accumulated_yaw = sum(unwrap_delta(a.yaw_rad, b.yaw_rad) for a, b in zip(samples, samples[1:]))
    result = {
        "start": _sample_dict(start), "action_end": _sample_dict(action_end),
        "settled": _sample_dict(settled), "longitudinal_m": longitudinal,
        "lateral_m": lateral, "relative_yaw_rad": accumulated_yaw,
        "center_shift_m": math.hypot(dx, dy),
        "settle_shift_m": math.hypot(settled.x_m - action_end.x_m, settled.y_m - action_end.y_m),
        "settle_yaw_rad": unwrap_delta(action_end.yaw_rad, settled.yaw_rad),
    }
    if target.goal_x_m is not None and target.goal_y_m is not None:
        result["goal_distance_m"] = math.hypot(target.goal_x_m - settled.x_m, target.goal_y_m - settled.y_m)
    if target.goal_yaw_rad is not None:
        result["yaw_error_rad"] = normalize_angle_rad(settled.yaw_rad - target.goal_yaw_rad)
    if request.name == "drive-distance":
        result["distance_error_m"] = longitudinal - request.distance_m
    if request.name == "rotate":
        result["rotation_error_rad"] = accumulated_yaw - request.angle_rad
    if request.name == "drive-distance" and math.isclose(abs(request.distance_m), 0.50, abs_tol=1e-9):
        result["fused_precision_met"] = abs(result["distance_error_m"]) <= 0.01 + 1e-9
    elif request.name == "rotate" and math.isclose(abs(request.angle_rad), math.pi / 2, abs_tol=1e-9):
        result["fused_precision_met"] = abs(result["rotation_error_rad"]) <= math.radians(1.0) + 1e-9
    else:
        result["fused_precision_met"] = None
    result["precision_source"] = "fused_pose"
    result["precision_met"] = result["fused_precision_met"]  # Deprecated compatibility alias.
    return result


def _checked_step(runtime, config, *, allow_pending=False):
    step = runtime.step()
    if step.error or step.mission_state in {RobocupMissionState.ERROR, RobocupMissionState.SAFE_STOP}:
        raise RuntimeError(step.error or "mission entered %s" % step.mission_state.value)
    sample = accepted_fused_sample(step.estimate, config, step.now_s,
                                   allow_pending=allow_pending)
    return step, sample


def run_one(runtime, config, request: ActionRequest, *, abort, max_s: float,
            preflight_s: float, settle_s: float, preflight_stable_s: float = PREFLIGHT_STABLE_S,
            sleep=time.sleep, clock=time.monotonic, on_started=None) -> dict:
    runtime.start()
    if on_started is not None:
        on_started(runtime)
    deadline = clock() + preflight_s
    ready = None
    origin = None
    healthy_since = None
    while clock() < deadline:
        if abort():
            raise RuntimeError("operator aborted during preflight")
        step, sample = _checked_step(runtime, config)
        if sample is not None and step.mission_state is RobocupMissionState.READY:
            now = clock()
            if (origin is None
                    or math.hypot(sample.x_m - origin.x_m, sample.y_m - origin.y_m) > PREFLIGHT_DRIFT_M
                    or abs(unwrap_delta(origin.yaw_rad, sample.yaw_rad)) > PREFLIGHT_DRIFT_RAD):
                origin = sample
                healthy_since = now
            elif now - healthy_since >= preflight_stable_s:
                ready = sample
                break
        else:
            origin = None
            healthy_since = None
        sleep(PERIOD_S)
    if ready is None:
        raise TimeoutError("stationary T265+SLAM fused pose did not stay healthy for %.1f seconds" % preflight_stable_s)
    target = target_from_start(ready, request)
    getattr(runtime.motion, target.method)(*target.args)
    action_started = clock()
    deadline = action_started + max_s
    samples = []
    action_end = None
    while clock() < deadline:
        if abort():
            raise RuntimeError("operator aborted during action")
        step, sample = _checked_step(runtime, config, allow_pending=True)
        if sample is None:
            raise RuntimeError("healthy fused pose lost during action")
        _check_step(samples[-1] if samples else None, sample, samples[0] if samples else sample)
        samples.append(sample)
        if step.motion is not None and step.motion.state is MotionActionState.SUCCEEDED:
            action_end = sample
            break
        if step.motion is not None and step.motion.state in {
                MotionActionState.BLOCKED, MotionActionState.POSE_LOST,
                MotionActionState.SAFE_STOPPED, MotionActionState.ERROR}:
            raise RuntimeError("motion ended in %s" % step.motion.state.value)
        sleep(PERIOD_S)
    if action_end is None:
        raise TimeoutError("single action exceeded its deadline")
    if runtime.drive.is_running:
        runtime.drive.stop()
    settled = action_end
    previous = action_end
    still_since = None
    deadline = clock() + settle_s
    while clock() < deadline:
        if abort():
            raise RuntimeError("operator aborted while settling")
        sleep(PERIOD_S)
        step, sample = _checked_step(runtime, config)
        if sample is None:
            raise RuntimeError("healthy fused pose lost while settling")
        _check_step(previous, sample, samples[0])
        samples.append(sample)
        moved = math.hypot(sample.x_m - previous.x_m, sample.y_m - previous.y_m)
        turned = abs(unwrap_delta(previous.yaw_rad, sample.yaw_rad))
        if moved < SETTLE_STEP_M and turned < SETTLE_STEP_RAD:
            if still_since is None:
                still_since = clock()
            elif clock() - still_since >= SETTLE_STABLE_S:
                settled = sample
                break
        else:
            still_since = None
        previous = sample
    else:
        raise TimeoutError("vehicle did not settle within the bounded window")
    return {"target": target.__dict__, "metrics": measurements(samples[0], action_end, settled, samples, request, target),
            "action_duration_s": clock() - action_started, "sample_count": len(samples)}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="this car's measured schema-v2 TOML")
    parser.add_argument("--output", required=True, help="new log directory outside the Git checkout")
    parser.add_argument("--confirm-motor-test", action="store_true")
    parser.add_argument("--confirm-area-clear", action="store_true")
    parser.add_argument("--confirm-estop-ready", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=30.0)
    actions = parser.add_subparsers(dest="action", required=True)
    distance = actions.add_parser("drive-distance")
    distance.add_argument("--cm", type=float, required=True, help="signed 1..50 cm")
    for name in ("rotate", "rotate-to"):
        turn = actions.add_parser(name)
        turn.add_argument("--deg", type=float, required=True, help="signed 1..90 degrees")
    for name in ("face-point", "drive-to", "follow-segment", "navigate-to", "navigate-to-pose"):
        point = actions.add_parser(name)
        point.add_argument("--forward-cm", type=float, required=True)
        point.add_argument("--left-cm", type=float, required=True)
        if name == "navigate-to-pose":
            point.add_argument("--yaw-deg", type=float, required=True, help="final yaw relative to start")
    return parser


CRITICAL_SOURCE_FILES = (
    "code/robocup_runtime.py",
    "code/components/basic_motion_controller.py",
    "code/components/differential_drive.py",
    "code/components/differential_kinematics.py",
    "code/components/c10b_diff_backend.py",
    "code/components/rear_motor.py",
    "code/components/t265_driver.py",
    "code/components/t265_pose_adapter.py",
    "code/components/pose_fusion.py",
    "code/components/slam_bridge.py",
    "code/components/slam_scan_source.py",
    "tools/closed_loop_motion.py",
)


def _git_text(*args: str) -> str:
    result = subprocess.run(["git", *args], cwd=str(ROOT), capture_output=True,
                            text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def _atomic_manifest_write(path: Path, manifest: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def _manifest_source_sha256() -> dict[str, str]:
    return {relative: hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
            for relative in CRITICAL_SOURCE_FILES if (ROOT / relative).is_file()}


def _runtime_serial(runtime) -> str:
    source = getattr(runtime, "t265_source", None)
    serial = getattr(source, "serial", None)
    return str(serial) if serial else "unknown"


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        request = request_from_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    if sys.platform != "linux":
        parser.error("live closed-loop motion requires the car's Linux host")
    if not all((args.confirm_motor_test, args.confirm_area_clear, args.confirm_estop_ready)):
        parser.error("explicit motor, clear-area and physical-estop confirmations are required")
    if not math.isfinite(args.max_seconds) or not 1.0 <= args.max_seconds <= 60.0:
        parser.error("--max-seconds must be between 1 and 60")
    config_path = Path(args.config).resolve()
    if config_path == DEFAULT_V2_CONFIG.resolve():
        parser.error("example configuration is not a measured car profile")
    output = Path(args.output).resolve()
    if output == ROOT or ROOT in output.parents or output.exists():
        parser.error("--output must name a new directory outside the repository")
    config = load_v2_config(config_path)
    if config.relay.enabled:
        parser.error("disable the payload relay for this motion test")
    config = accepted_relative_slam_profile(config)
    limits = runtime_constraints(config, RuntimeMode.HARDWARE_PROBE)
    config = replace(config, drive=replace(config.drive,
        max_linear_speed_m_s=min(config.drive.max_linear_speed_m_s, limits.max_linear_speed_m_s),
        max_angular_speed_rad_s=min(config.drive.max_angular_speed_rad_s, limits.max_angular_speed_rad_s),
        max_wheel_speed_m_s=min(config.drive.max_wheel_speed_m_s, limits.max_wheel_speed_m_s)))
    output.mkdir(parents=True, exist_ok=False)
    commit = _git_text("rev-parse", "HEAD")
    git_status = _git_text("status", "--porcelain=v1")
    manifest = {"action": request.__dict__, "config_path": str(config_path),
                "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
                "commit": commit, "git_status_porcelain": git_status,
                "git_dirty": bool(git_status),
                "critical_source_sha256": _manifest_source_sha256(),
                "mode": RuntimeMode.HARDWARE_MISSION.value,
                "effective_protocol_mode": config.drive.protocol_mode,
                "effective_drive_config": asdict(config.drive),
                "effective_navigation_config": asdict(config.navigation),
                "effective_t265_mount": asdict(config.t265_mount),
                "localization_backend": config.localization.backend,
                "relative_slam_profile_applied": bool(config.localization.slam.relative_goals_only),
                "t265_serial": "unknown", "slam_session_id": "unknown",
                "sidecar_identity": "unknown",
                "python": sys.version, "platform": platform.platform(),
                "speed_caps": {"linear_m_s": config.drive.max_linear_speed_m_s,
                               "angular_rad_s": config.drive.max_angular_speed_rad_s},
                "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    manifest_path = output / "manifest.json"
    _atomic_manifest_write(manifest_path, manifest)
    logger = JsonlEventLogger(output / "events.jsonl")
    runtime = None
    aborted = [False]
    previous_handlers = {}
    for signum in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if signum is not None:
            previous_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, lambda _signum, _frame: aborted.__setitem__(0, True))
    summary = {"valid": False, "action": request.name, "error": None}

    def update_started_runtime(started_runtime) -> None:
        manifest["t265_serial"] = _runtime_serial(started_runtime)
        _atomic_manifest_write(manifest_path, manifest)

    try:
        runtime = build_runtime(config, RuntimeMode.HARDWARE_MISSION, event_logger=logger)
        summary.update(run_one(runtime, config, request, abort=lambda: aborted[0], max_s=args.max_seconds,
                               preflight_s=30.0, settle_s=3.0,
                               on_started=update_started_runtime))
        summary["valid"] = True
    except BaseException as exc:
        summary["error"] = "%s: %s" % (type(exc).__name__, exc)
        if runtime is not None:
            runtime.mission.request_safe_stop(summary["error"])
    finally:
        if runtime is not None:
            runtime.close()
        else:
            logger.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        summary["dropped_events"] = logger.dropped_events
        summary["log_write_error"] = logger.write_error
        if logger.dropped_events or logger.write_error:
            summary["valid"] = False
            summary["error"] = summary["error"] or "event log incomplete"
        metrics = summary.get("metrics", {})
        summary["fused_precision_met"] = metrics.get(
            "fused_precision_met", metrics.get("precision_met")
        )
        summary["precision_source"] = "fused_pose"
        summary["precision_met"] = summary["fused_precision_met"]  # Deprecated compatibility alias.
        (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print("summary: %s" % (output / "summary.json"))
    if summary["error"]:
        print("ERROR: %s" % summary["error"])
    elif summary["fused_precision_met"] is False:
        print("FUSED-POSE PRECISION FAIL: action completed but its fused settled error exceeded the target")
    return 0 if summary["valid"] and summary["fused_precision_met"] is not False else (3 if summary["valid"] else 1)


if __name__ == "__main__":
    raise SystemExit(main())
