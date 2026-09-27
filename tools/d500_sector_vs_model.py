#!/usr/bin/env python3
"""Compare what the D500 actually sees per direction against the wall model.

Read-only.  Prints the nearest boundary echo in each 10 degree body-frame sector
next to the range at which the configured rectangle predicts a wall, so a
"which edges are visible" claim can be checked against raw geometry instead of
trusted.

Usage:
    python3 tools/d500_sector_vs_model.py --field-width-m 5 --field-height-m 4 \
        --start-x-m 2.5 --start-y-m 2.0 --start-yaw-deg 0
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.radar_driver import (  # noqa: E402
    D500SerialDriver,
    RadarMount,
    RadarScanAssembler,
    scan_points_in_body,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyS6")
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--field-width-m", type=float, default=5.0)
    parser.add_argument("--field-height-m", type=float, default=4.0)
    parser.add_argument("--start-x-m", type=float, default=2.5)
    parser.add_argument("--start-y-m", type=float, default=2.0)
    parser.add_argument("--start-yaw-deg", type=float, default=0.0)
    parser.add_argument("--sector-deg", type=float, default=10.0)
    args = parser.parse_args()

    width = args.field_width_m * 100.0
    height = args.field_height_m * 100.0
    x0, y0 = args.start_x_m * 100.0, args.start_y_m * 100.0
    yaw_cw = args.start_yaw_deg

    scans = []
    assembler = RadarScanAssembler()

    def on_packet(packet):
        scans.extend(assembler.feed(packet))

    driver = D500SerialDriver(on_packet=on_packet, port=args.port)
    driver.start()
    if not driver.wait_connected(5.0):
        print("FAIL: D500 not connected")
        return 2
    time.sleep(args.seconds)
    driver.close()
    print("scans: %d" % len(scans))
    if not scans:
        print("FAIL: no complete revolution")
        return 3

    # Nearest echo per sector, aggregated over all scans (sparse net returns).
    width_deg = args.sector_deg
    buckets: dict[int, list[float]] = {}
    for scan in scans:
        for x, y in scan_points_in_body(scan, RadarMount()):
            r = math.hypot(x, y)
            if r <= 0.0:
                continue
            ang = math.degrees(math.atan2(y, x))          # body frame, +Y left
            key = int(math.floor((ang + 180.0) / width_deg)) * int(width_deg) - 180
            buckets.setdefault(key, []).append(r)

    def model_range(ang_deg: float):
        """Distance to the rectangle along a body-frame bearing, if it hits."""

        # body -> wall frame (yaw is clockwise-positive)
        a = math.radians(yaw_cw)
        c, s = math.cos(a), math.sin(a)
        # rotate_cw(body_dir, +yaw) gives the wall-frame direction
        dx = c * math.cos(math.radians(ang_deg)) + s * math.sin(math.radians(ang_deg))
        dy = -s * math.cos(math.radians(ang_deg)) + c * math.sin(math.radians(ang_deg))
        hits = []
        if abs(dx) > 1e-9:
            for wall_x in (0.0, width):
                t = (wall_x - x0) / dx
                if t > 0:
                    yy = y0 + t * dy
                    if -1e-6 <= yy <= height + 1e-6:
                        hits.append(t)
        if abs(dy) > 1e-9:
            for wall_y in (0.0, height):
                t = (wall_y - y0) / dy
                if t > 0:
                    xx = x0 + t * dx
                    if -1e-6 <= xx <= width + 1e-6:
                        hits.append(t)
        return min(hits) if hits else None

    print()
    print("%8s %10s %10s %10s" % ("sector", "model_cm", "nearest", "pts"))
    print("-" * 44)
    for key in sorted(buckets):
        values = buckets[key]
        near = min(values)
        model = model_range(key + width_deg / 2.0)
        flag = ""
        if model is not None:
            # A near echo far below the model range is interior clutter.
            if near < model - 100.0:
                flag = "  <- closer than the wall (interior object?)"
            elif near > model + 200.0:
                flag = "  <- no return near the wall"
        else:
            flag = "  <- model says no wall on this bearing"
        print("%5d..%4d %10s %9.0f %10d%s"
              % (key, key + width_deg,
                 "-" if model is None else "%.0f" % model,
                 near, len(values), flag))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
