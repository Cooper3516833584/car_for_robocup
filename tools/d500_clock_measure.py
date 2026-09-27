#!/usr/bin/env python3
"""Measure the D500 raw packet clock: resolution, rate and real rollover.

Phase 3.6 requires the timestamp modulus to come from measurement, not from the
assumption that the field is a uint16 millisecond counter.  Raw packets are
recorded before the assembler and before ``DeviceClockMapper``, so the measured
rollover is a property of the device rather than of our own unwrapping.

Reads through the verified ``D500SerialDriver`` rather than re-implementing the
UART setup: an earlier version of this probe hard-coded 115200 baud while the
D500 runs at 230400 8N1, which decodes zero packets.

Read-only on the radar; never touches the motors.

Usage:
    python3 tools/d500_clock_measure.py [--port /dev/ttyS6] [--seconds 300]
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.radar_driver import (  # noqa: E402
    DEFAULT_D500_BAUDRATE,
    D500SerialDriver,
)

# A D500 revolution is roughly 140 ms.  The point of the run is to see the raw
# counter wrap, so a very short capture can only produce weak evidence.
MIN_MEANINGFUL_SECONDS = 120.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyS6")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_D500_BAUDRATE)
    parser.add_argument("--seconds", type=float, default=300.0)
    parser.add_argument("--out", default="/tmp/d500_clock.csv")
    args = parser.parse_args()

    if args.seconds < MIN_MEANINGFUL_SECONDS:
        print("WARNING: %.0f s may not contain a rollover; results will be weak"
              % args.seconds)

    rows: list[tuple[int, int, float]] = []

    def on_packet(packet):
        rows.append((int(packet.timestamp_ms),
                     int(packet.rotation_speed_deg_s),
                     time.monotonic()))

    driver = D500SerialDriver(on_packet=on_packet, port=args.port,
                              baudrate=args.baudrate)
    driver.start()
    if not driver.wait_connected(5.0):
        print("FAIL: D500 not connected on %s" % args.port)
        return 2
    print("capturing raw D500 packets for %.0f s on %s @ %d 8N1 ..."
          % (args.seconds, args.port, args.baudrate))

    started = time.monotonic()
    try:
        while time.monotonic() - started < args.seconds:
            time.sleep(0.2)
    finally:
        driver.close()

    if len(rows) < 200:
        print("FAIL: only %d packets captured" % len(rows))
        return 3

    host_span = rows[-1][2] - rows[0][2]
    stamps = [r[0] for r in rows]
    rate = len(rows) / host_span

    print()
    print("=== raw capture ===")
    print("  packets         : %d over %.1f s" % (len(rows), host_span))
    print("  packet rate     : %.1f Hz" % rate)
    print("  raw min / max   : %d / %d" % (min(stamps), max(stamps)))
    print("  distinct values : %d" % len(set(stamps)))

    deltas = [b - a for a, b in zip(stamps, stamps[1:])]
    negative = [d for d in deltas if d < 0]
    print()
    print("=== per-packet raw delta ===")
    print("  min / median / max : %d / %.0f / %d"
          % (min(deltas), statistics.median(deltas), max(deltas)))
    print("  negative deltas    : %d" % len(negative))
    for value in sorted(set(negative))[:10]:
        print("     %4d x delta=%d" % (negative.count(value), value))

    # A rollover appears as a large negative jump: the counter resets from near
    # its maximum.  The value just before the jump bounds the modulus below.
    print()
    print("=== rollovers ===")
    wraps = []
    for index, delta in enumerate(deltas):
        if delta < -max(1000.0, 0.25 * max(stamps)):
            wraps.append((index, stamps[index], stamps[index + 1], delta))
    if not wraps:
        print("  none observed; the counter did not wrap in this window")
    for index, before, after, delta in wraps[:12]:
        print("  packet %6d: %6d -> %6d  (delta %d)  pre-wrap value %d"
              % (index, before, after, delta, before))

    print()
    print("=== modulus bounds ===")
    print("  highest raw value seen : %d" % max(stamps))
    if wraps:
        lows = [w[1] for w in wraps]
        highs = [w[2] for w in wraps]
        print("  smallest pre-wrap value: %d" % min(lows))
        print("  largest post-wrap value: %d" % max(highs))
        print("  => modulus in (%d, %d]" % (max(highs), min(lows) + 1))
    print("  a 65536 ms modulus would wrap every 65.5 s")

    span_raw = stamps[-1] - stamps[0]
    if wraps:
        span_raw += 65536 * len(wraps)
    print()
    print("=== unit check ===")
    print("  unwrapped raw span (assuming uint16) : %d" % span_raw)
    print("  host span                            : %.1f s" % host_span)
    if host_span > 0:
        ticks = span_raw / host_span
        print("  raw ticks per host second            : %.1f" % ticks)
        if 900.0 <= ticks <= 1100.0:
            print("  => consistent with a millisecond counter")
        else:
            print("  => NOT milliseconds; scale is %.4f ms per tick"
                  % (1000.0 / ticks))

    try:
        with open(args.out, "w", encoding="utf-8") as handle:
            handle.write("raw_ms,speed_deg_s,monotonic_s\n")
            for raw, speed, now in rows:
                handle.write("%d,%d,%.6f\n" % (raw, speed, now))
        print()
        print("  raw CSV written to %s (%d rows)" % (args.out, len(rows)))
    except OSError as exc:
        print("  could not write %s: %s" % (args.out, exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
