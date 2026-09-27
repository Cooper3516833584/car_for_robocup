#!/usr/bin/env python3
"""Read-only D500 field survey: what can the lidar actually measure here?

Prints per-scan statistics, a range histogram and per-sector ranges, then
extracts *individual* straight edges from the aggregated cloud.

A single principal axis over a full 360 deg revolution is deliberately NOT used
as a wall measure.  A rectangular field scanned all round is made of four
different edges, so one axis always shows a huge residual even when every edge
is perfectly clean; comparing that residual against ``max_line_rms_cm`` (which
describes one wall's local inliers) is meaningless.  Use
``tools/d500_boundary_diag.py`` for the production per-wall assessment.

Read-only on the radar; never touches the motors.

Usage:
    python3 tools/d500_diag.py [--port /dev/ttyS6] [--seconds 5]
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.radar_diagnostics import (  # noqa: E402
    aggregate_polar_bins,
    extract_lines,
)
from components.radar_driver import (  # noqa: E402
    D500SerialDriver,
    RadarMount,
    RadarScanAssembler,
    scan_points_in_body,
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
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--sectors", type=int, default=12)
    args = parser.parse_args()

    print("=== D500 field survey (%s, %.1f s) ===" % (args.port, args.seconds))
    scans = collect(args.port, args.seconds)
    print("  complete scans: %d" % len(scans))
    if not scans:
        print("FAIL: no complete revolution assembled")
        return 3

    counts = []
    ranges = []
    for scan in scans:
        pts = scan_points_in_body(scan, RadarMount())
        counts.append(len(pts))
        ranges.extend(math.hypot(x, y) for x, y in pts)
    print("  points per scan: median %d, range %d..%d"
          % (int(statistics.median(counts)), min(counts), max(counts)))

    print()
    print("  range histogram (cm):")
    bins = [0] * 14
    for r in ranges:
        b = int(r // 50.0)
        if 0 <= b < len(bins):
            bins[b] += 1
    total = max(1, len(ranges))
    for i, c in enumerate(bins):
        if c:
            print("    %4d-%4d : %6d pts (%5.1f%%)  %s"
                  % (i * 50, (i + 1) * 50, c, 100.0 * c / total,
                     "#" * int(50.0 * c / total)))
    print("  farthest %.1f cm, nearest %.1f cm" % (max(ranges), min(ranges)))

    print()
    print("  last-scan ranges by %d deg sector (body frame, +X forward):"
          % (360 // args.sectors))
    pts = scan_points_in_body(scans[-1], RadarMount())
    width = 360.0 / args.sectors
    sectors = {}
    for x, y in pts:
        ang = math.degrees(math.atan2(y, x))
        sec = int((ang + 180.0) // width) * width - 180.0
        sectors.setdefault(sec, []).append(math.hypot(x, y))
    for sec in sorted(sectors):
        rs = sectors[sec]
        print("    %+6.0f..%+6.0f deg: %3d pts, %.0f..%.0f cm (median %.0f)"
              % (sec, sec + width, len(rs), min(rs), max(rs),
                 statistics.median(rs)))
    print("    NOTE: a large range spread inside one sector is normal for a")
    print("          rectangle (a sector can span a corner). It is not by")
    print("          itself evidence that the boundary is unusable.")

    # --- per-edge extraction over the aggregated cloud -------------------
    print()
    print("  per-edge line extraction (aggregated boundary candidates):")
    polar = aggregate_polar_bins(scans, bin_deg=1.0)
    cloud = []
    for b in polar:
        near = b.near_quantile_cm(0.25, min_hits=2)
        if near is None:
            continue
        # Radar angles grow clockwise, so body y is -r*sin(angle).
        a = math.radians(b.angle_cw_deg)
        cloud.append((near * math.cos(a), -near * math.sin(a)))
    print("    aggregated boundary candidates: %d" % len(cloud))
    if len(cloud) < 8:
        print("    too few aggregated candidates for line extraction")
        return 0

    fits = extract_lines(cloud, inlier_gate_cm=5.0, min_points=8,
                         min_span_cm=40.0, max_lines=8)
    if not fits:
        print("    no line met the point/span gate")
        return 0
    for i, (fit, _) in enumerate(fits):
        print("    edge %d: angle %+7.1f deg  pts %3d  span %6.1f cm  "
              "rms %5.2f cm  max %5.2f cm  offset %+8.1f cm"
              % (i, fit.angle_deg, fit.points, fit.span_cm, fit.rms_cm,
                 fit.max_abs_residual_cm, fit.offset_cm))

    angles = [f.angle_deg % 180.0 for f, _ in fits]
    orthogonal = 0
    for i in range(len(angles)):
        for j in range(i + 1, len(angles)):
            delta = abs(angles[i] - angles[j]) % 180.0
            if abs(delta - 90.0) < 8.0:
                orthogonal += 1
    print("    orthogonal edge pairs within 8 deg: %d" % orthogonal)
    print("    (a usable rectangle should show two orthogonal families)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
