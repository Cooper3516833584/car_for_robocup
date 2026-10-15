#!/usr/bin/env python3
"""Film the yellow-only payload route with fused/T265 closed-loop localization."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.run_ch3_drop_test import main as route_main


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    return route_main(["--payload-slot", "0", "--target-class-name", "yellow", *args])


if __name__ == "__main__":
    raise SystemExit(main())
