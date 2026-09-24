"""Read-only T265 pose probe. This tool never opens the motor controller."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.t265_driver import RealSenseT265PoseSource, T265UnavailableError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="", help="T265 serial number (empty selects the first device)")
    args = parser.parse_args(argv)
    source = RealSenseT265PoseSource(args.serial)
    try:
        source.start()
        print(f"T265 serial: {source.serial or '(not reported by SDK)'}")
        while True:
            sample = source.read()
            if sample is not None:
                print(
                    f"host_monotonic={sample.host_monotonic_s:.3f} "
                    f"device_ms={sample.device_timestamp_ms!r} "
                    f"xyz={sample.translation_xyz!r} "
                    f"xyzw={sample.quaternion_xyzw!r} "
                    f"tracker={sample.tracker_confidence} mapper={sample.mapper_confidence}"
                )
            time.sleep(0.2)
    except KeyboardInterrupt:
        return 0
    except T265UnavailableError as exc:
        parser.exit(2, f"T265 unavailable: {exc}\n")
    finally:
        source.stop()


if __name__ == "__main__":
    raise SystemExit(main())
