#!/usr/bin/env python3
"""Film the yellow-only payload route with temporary position checks disabled."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.run_center_target_route import main as route_main


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    return route_main(["--demo-no-position-checks", *args])


if __name__ == "__main__":
    raise SystemExit(main())
