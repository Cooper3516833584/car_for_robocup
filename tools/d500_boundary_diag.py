#!/usr/bin/env python3
"""Read-only assessment of the D500 rectangle-boundary observability.

Uses the production ``WallLineLocalizer`` rather than a second implementation,
so what this tool reports is what the runtime will see.  It answers one
question:

    Can the D500 actually observe the safety-net rectangle edges well enough to
    anchor the map?

The safety net is the real field boundary, so it is expected that some rays
pass through the mesh and return from far away.  The metrics below therefore
report per-edge inlier counts and robust residuals, not a single number over
a full revolution.

Nothing here touches the motors.

Usage:
    python3 tools/d500_boundary_diag.py --start-x-m 0 --start-y-m 0 --start-yaw-deg 0
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
    DroneGlobalAlignment,
    Pose2D,
    RadarMount,
    RadarScanAssembler,
    RectangularWallReference,
    WallLineConfig,
    WallLineLocalizer,
)


def collect(port: str, seconds: float):
    assembler = RadarScanAssembler()
    scans = []

    def on_packet(packet):
        for scan in assembler.feed(packet):
            scans.append(scan)

    driver = D500SerialDriver(on_packet=on_packet, port=port)
    driver.start()
    if not driver.wait_connected(5.0):
        raise SystemExit("FAIL: D500 not connected on %s" % port)
    time.sleep(seconds)
    driver.close()
    return scans


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyS6")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--field-width-m", type=float, default=5.0)
    parser.add_argument("--field-height-m", type=float, default=5.0)
    parser.add_argument("--start-x-m", type=float, default=0.0,
                        help="car x in wall frame at scan start")
    parser.add_argument("--start-y-m", type=float, default=0.0,
                        help="car y in wall frame at scan start")
    parser.add_argument("--start-yaw-deg", type=float, default=0.0,
                        help="car yaw (CCW positive) in wall frame at start")
    args = parser.parse_args()

    # The wall frame is a rectangle spanning [0, W] x [0, H]; the car starts at
    # an explicitly given pose inside it.  Never assume the car is at the
    # centre, and never assume the local origin is the field origin.
    width_cm = args.field_width_m * 100.0
    height_cm = args.field_height_m * 100.0
    reference = RectangularWallReference(
        wall_to_global=DroneGlobalAlignment(0.0, 0.0, 0.0),
        back_wall_x_cm=0.0,
        right_wall_y_cm=0.0,
        front_wall_x_cm=width_cm,
        left_wall_y_cm=height_cm,
    )
    localizer = WallLineLocalizer(
        reference, mount=RadarMount(), config=WallLineConfig()
    )

    print("=== D500 rectangle-boundary assessment ===")
    print("  wall frame: x in [0, %.0f] cm, y in [0, %.0f] cm"
          % (width_cm, height_cm))
    print("  start pose: (%.2f, %.2f) m, yaw %.1f deg"
          % (args.start_x_m, args.start_y_m, args.start_yaw_deg))
    print("  WallLineConfig: points>=%d span>=%.0f cm rms<=%.2f cm gate=%.0f cm"
          % (WallLineConfig().min_points_per_wall,
             WallLineConfig().min_line_span_cm,
             WallLineConfig().max_line_rms_cm,
             WallLineConfig().association_gate_cm))
    print()

    scans = collect(args.port, args.seconds)
    print("  scans: %d" % len(scans))
    if not scans:
        return 3

    start_pose = Pose2D(
        args.start_x_m * 100.0,
        args.start_y_m * 100.0,
        -args.start_yaw_deg,   # radar Pose2D yaw is clockwise-positive
    )

    attempts = 0
    usable = 0
    two_axis = 0
    one_axis = 0
    no_boundary = 0
    xs, ys, yaws = [], [], []
    edge_ok = {"back": 0, "front": 0, "right": 0, "left": 0}
    edge_pts = {"back": [], "front": [], "right": [], "left": []}

    for scan in scans:
        attempts += 1
        try:
            obs = localizer.observe(scan, start_pose)
        except Exception as exc:  # noqa: BLE001
            print("    observe() raised: %s: %s" % (type(exc).__name__, exc))
            no_boundary += 1
            continue

        has_x = obs.x_cm is not None
        has_y = obs.y_cm is not None
        has_yaw = obs.yaw_cw_deg is not None
        # back/right point counts are what WallPoseObservation carries.
        if obs.back_wall_points:
            edge_ok["back"] += 1
            edge_pts["back"].append(obs.back_wall_points)
        if obs.right_wall_points:
            edge_ok["right"] += 1
            edge_pts["right"].append(obs.right_wall_points)

        if has_x or has_y:
            usable += 1
        if has_x and has_y:
            two_axis += 1
        elif has_x or has_y:
            one_axis += 1
        else:
            no_boundary += 1
        if has_x and obs.x_cm is not None:
            xs.append(obs.x_cm)
        if has_y and obs.y_cm is not None:
            ys.append(obs.y_cm)
        if has_yaw and obs.yaw_cw_deg is not None:
            yaws.append(obs.yaw_cw_deg)

    print()
    print("=== results ===")
    print("  boundary usable scans: %d/%d (%.1f%%)"
          % (usable, attempts, 100.0 * usable / max(1, attempts)))
    for name in ("back", "front", "right", "left"):
        counts = edge_pts[name]
        med = int(statistics.median(counts)) if counts else 0
        print("  %-5s visibility: %2d/%d, median inliers=%d"
              % (name, edge_ok[name], attempts, med))
    print("  2-axis pose scans:    %d" % two_axis)
    print("  single-axis pose scans: %d" % one_axis)
    print("  no-boundary scans:    %d" % no_boundary)

    def spread(values, scale=1.0):
        if len(values) < 2:
            return None
        return (max(values) - min(values)) * scale

    print()
    if xs:
        print("  x candidate std %.2f cm, peak-to-peak %.2f cm"
              % (statistics.pstdev(xs), spread(xs)))
    if ys:
        print("  y candidate std %.2f cm, peak-to-peak %.2f cm"
              % (statistics.pstdev(ys), spread(ys)))
    if yaws:
        print("  yaw candidate std %.2f deg, peak-to-peak %.2f deg"
              % (statistics.pstdev(yaws), spread(yaws)))

    print()
    print("  Reminder: some rays passing through the mesh and returning from far")
    print("  away is expected; the question is whether enough stable near-boundary")
    print("  echoes remain per edge.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
