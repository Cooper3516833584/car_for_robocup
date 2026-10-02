"""Read-only T265 pose probe. This tool never opens the motor controller."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "code"))

from components.t265_driver import RealSenseT265PoseSource, T265UnavailableError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="", help="T265 serial number (empty selects the first device)")
    parser.add_argument("--config", help="schema-v2 profile for mount-corrected base_link pose")
    parser.add_argument("--seconds", type=float, help="positive run duration; default: until Ctrl+C")
    args = parser.parse_args(argv)
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")

    adapter = None
    serial = args.serial
    if args.config:
        from config.v2_loader import load_v2_config
        from components.t265_pose_adapter import T265PoseAdapter

        config = load_v2_config(args.config)
        if not config.t265.enabled:
            parser.error("config disables T265")
        adapter = T265PoseAdapter(
            config.t265_mount,
            min_tracker_confidence=config.fusion.t265_min_tracker_confidence,
            max_age_s=config.fusion.t265_max_age_s,
        )
        serial = serial or config.t265.serial

    source = RealSenseT265PoseSource(serial)
    try:
        source.start()
        print(f"T265 serial: {source.serial or '(not reported by SDK)'}")
        deadline = None if args.seconds is None else time.monotonic() + args.seconds
        while deadline is None or time.monotonic() < deadline:
            sample = source.read()
            if sample is not None:
                base = ""
                if adapter is not None:
                    update = adapter.adapt(sample, now_s=time.monotonic())
                    if update.pose is None:
                        base = f" base_link=invalid:{update.reason}"
                    else:
                        base = (
                            f" base_x_cm={update.pose.x_m * 100:.2f}"
                            f" base_y_cm={update.pose.y_m * 100:.2f}"
                            f" yaw_ccw_deg={math.degrees(update.pose.yaw_rad):.2f}"
                        )
                print(
                    f"host_monotonic={sample.host_monotonic_s:.3f} "
                    f"device_ms={sample.device_timestamp_ms!r} "
                    f"xyz={sample.translation_xyz!r} "
                    f"xyzw={sample.quaternion_xyzw!r} "
                    f"tracker={sample.tracker_confidence} mapper={sample.mapper_confidence}"
                    f"{base}"
                )
            time.sleep(0.2)
        return 0
    except KeyboardInterrupt:
        return 0
    except T265UnavailableError as exc:
        parser.exit(2, f"T265 unavailable: {exc}\n")
    finally:
        source.stop()


if __name__ == "__main__":
    raise SystemExit(main())
