#!/usr/bin/env python3
"""Write a read-only software/configuration fingerprint for a RoboCup car."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))

from config.v2_loader import load_v2_config  # noqa: E402

SOURCE_FILES = (
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


def _git(*args: str) -> str:
    result = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def snapshot(config_path: Path) -> dict:
    config_path = config_path.resolve(strict=True)
    config = load_v2_config(config_path)
    status = _git("status", "--porcelain=v1")
    return {
        "git_head": _git("rev-parse", "HEAD"),
        "git_branch": _git("branch", "--show-current"),
        "git_status_porcelain": status,
        "git_dirty": bool(status),
        "config_path": str(config_path),
        "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "protocol_mode": config.drive.protocol_mode,
        "c10b_diff_firmware_verified": config.calibration.c10b_diff_firmware_verified,
        "drive_track_width_m": config.geometry.drive_track_width_m,
        "firmware_track_width_m": config.drive.firmware_track_width_m,
        "position_tolerance_m": config.navigation.position_tolerance_m,
        "yaw_tolerance_rad": config.navigation.yaw_tolerance_rad,
        "max_linear_speed_m_s": config.drive.max_linear_speed_m_s,
        "max_angular_speed_rad_s": config.drive.max_angular_speed_rad_s,
        "max_linear_accel_m_s2": config.drive.max_linear_accel_m_s2,
        "max_angular_accel_rad_s2": config.drive.max_angular_accel_rad_s2,
        "t265_mount": asdict(config.t265_mount),
        "effective_drive_config": asdict(config.drive),
        "effective_navigation_config": asdict(config.navigation),
        "localization_backend": config.localization.backend,
        "source_sha256": {
            relative: hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
            for relative in SOURCE_FILES if (ROOT / relative).is_file()
        },
        "python": sys.version,
        "platform": platform.platform(),
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path,
                        help="JSON destination outside the repository")
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if output == ROOT or ROOT in output.parents:
        parser.error("--output must be outside the repository")
    payload = snapshot(args.config)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
