#!/usr/bin/env python3
"""Film the yellow-only payload route with fused/T265 closed-loop localization."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.run_center_target_route import main as route_main
from components.relay_lcus import DEFAULT_RELAY_PORT


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    return route_main(["--target-action", "drop", "--payload-slot", "2",
                       "--target-class-name", "yellow", "--release-mode", "relay",
                       "--relay-port", DEFAULT_RELAY_PORT, *args])


if __name__ == "__main__":
    raise SystemExit(main())
