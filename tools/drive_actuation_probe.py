#!/usr/bin/env python3
"""Single-action, supervised C10B drive-chain probe for wheels-off-ground.

This tool intentionally exposes only four fixed chassis actions. Run each
action as a separate process so stop frames and the hardware lock are released
between observations.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from core.types import Twist2D  # noqa: E402

MAX_DURATION_S = 3.0
MIN_DURATION_S = 0.5
MAX_LINEAR_M_S = 0.05
MAX_ANGULAR_RAD_S = 0.25
ACTIONS = ("forward", "backward", "rotate-left", "rotate-right")


def _backend_diagnostics(drive) -> dict:
    backend = drive.backend
    encoded = getattr(backend, "last_encoded_wheel_speeds_m_s", None)
    chassis = getattr(backend, "last_c10b_chassis_command", None)
    return {
        "canonical_limited_twist": {
            "vx_m_s": drive.last_limited_twist.linear_x_m_s,
            "omega_rad_s": drive.last_limited_twist.angular_z_rad_s,
        },
        "canonical_wheel_targets_m_s": (
            None if encoded is None else list(encoded)
        ),
        "c10b_quantized": (
            None if chassis is None else {
                "vx_mm_s": chassis.linear_mm_s,
                "vz_mrad_s": chassis.angular_mrad_s,
                "requested_left_mm_s": chassis.requested.left_mm_s,
                "requested_right_mm_s": chassis.requested.right_mm_s,
            }
        ),
    }


def _write_event(path: Path, event: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(event, ensure_ascii=False, sort_keys=True) + "\n")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="explicit vehicle TOML profile")
    parser.add_argument("--duration-s", type=float, default=2.0,
                        help="duration for this one action, 0.5..3.0 seconds")
    parser.add_argument("--output", required=True, type=Path,
                        help="new JSONL path outside the repository")
    parser.add_argument("--confirm-wheels-off-ground", action="store_true", required=True)
    parser.add_argument("--confirm-estop-ready", action="store_true", required=True)
    parser.add_argument("--confirm-area-clear", action="store_true", required=True)
    parser.add_argument("action", choices=ACTIONS)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration_s) or not MIN_DURATION_S <= args.duration_s <= MAX_DURATION_S:
        parser.error("--duration-s must be in [0.5, 3.0]")
    output = args.output.resolve()
    if output == ROOT or ROOT in output.parents:
        parser.error("--output must be outside the repository")
    if output.exists():
        parser.error("--output must be a new file")

    from config.v2_factory import build_differential_drive
    from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config
    from config.v2_runtime import RuntimeMode, runtime_constraints

    config_path = Path(args.config).resolve(strict=True)
    if config_path == DEFAULT_V2_CONFIG.resolve():
        parser.error("example profile contains unmeasured placeholders")
    config = load_v2_config(config_path)
    if config.relay.enabled:
        parser.error("disable the payload relay for this motor probe")
    limits = runtime_constraints(config, RuntimeMode.HARDWARE_PROBE)
    linear_m_s = min(MAX_LINEAR_M_S, limits.max_linear_speed_m_s)
    angular_rad_s = min(MAX_ANGULAR_RAD_S, limits.max_angular_speed_rad_s)
    if args.action.startswith("rotate") and not config.drive.allow_in_place_rotation:
        parser.error("this profile disables in-place rotation")

    twists = {
        "forward": Twist2D(linear_m_s, 0.0),
        "backward": Twist2D(-linear_m_s, 0.0),
        "rotate-left": Twist2D(0.0, angular_rad_s),
        "rotate-right": Twist2D(0.0, -angular_rad_s),
    }
    drive = build_differential_drive(config)
    period_s = min(0.02, drive.command_timeout_s / 3.0)
    event_base = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "action": args.action,
        "duration_s": args.duration_s,
        "git_head": __import__("subprocess").run(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True,
            capture_output=True, check=False,
        ).stdout.strip(),
        "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "protocol_mode": config.drive.protocol_mode,
        "limits": {"linear_m_s": linear_m_s, "angular_rad_s": angular_rad_s},
    }
    aborted = False

    def on_signal(signum, _frame):
        nonlocal aborted
        aborted = True
        print("SIGNAL %d: requesting stop" % signum, flush=True)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, on_signal)

    status = 0
    started = None
    try:
        with drive:
            started = time.monotonic()
            deadline = started + args.duration_s
            next_send = started
            while time.monotonic() < deadline:
                if aborted:
                    raise RuntimeError("operator requested abort")
                now = time.monotonic()
                if now >= next_send:
                    drive.command(twists[args.action])
                    next_send = now + period_s
                time.sleep(0.005)
            drive.stop()
            event = dict(event_base)
            event.update({
                "result": "completed",
                "elapsed_s": round(time.monotonic() - started, 4),
                "watchdog_stop_count": drive.watchdog_stop_count,
                **_backend_diagnostics(drive),
            })
            _write_event(output, event)
    except Exception as exc:  # noqa: BLE001 - context exit closes/stops the backend
        status = 1
        event = dict(event_base)
        event.update({
            "result": "error",
            "error_type": type(exc).__name__,
            "error": str(exc),
            "elapsed_s": None if started is None else round(time.monotonic() - started, 4),
            **_backend_diagnostics(drive),
        })
        try:
            _write_event(output, event)
        except OSError as write_exc:
            print("could not write probe event: %s" % write_exc, file=sys.stderr)
        print("ERROR: %s: %s" % (type(exc).__name__, exc), file=sys.stderr)
    print("probe %s: %s; log=%s" % (args.action, "stopped" if status == 0 else "failed", output))
    return status


if __name__ == "__main__":
    raise SystemExit(main())
