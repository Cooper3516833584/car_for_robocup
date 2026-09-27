#!/usr/bin/env python3
"""Measure the D500 raw packet clock: resolution, rate and real rollover.

Phase 3.6 requires the timestamp modulus to come from measurement, not from the
assumption that the field is a uint16 millisecond counter.  This records raw
packets directly, so it is independent of the assembler and of
``DeviceClockMapper``.

Read-only on the radar; never touches the motors.

Usage:
    python3 d500_clock_measure.py [--port /dev/ttyS6] [--seconds 300]
"""

from __future__ import annotations

import argparse
import collections
import math
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.radar_driver import D500PacketParser  # noqa: E402

SERIAL_BAUD = 230400


def open_serial(device: str) -> int:
    import os
    import termios

    fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
    settings = termios.tcgetattr(fd)
    settings[0] = termios.IGNPAR
    settings[1] = 0
    settings[2] = termios.B115200 | termios.CS8 | termios.CREAD | termios.CLOCAL
    settings[3] = 0
    settings[4] = termios.B115200
    settings[5] = termios.B115200
    settings[6][termios.VMIN] = 0
    settings[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, settings)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyS6")
    parser.add_argument("--seconds", type=float, default=300.0)
    parser.add_argument("--out", default="/tmp/d500_clock.csv")
    args = parser.parse_args()

    fd = open_serial(args.port)
    decoder = D500PacketParser()
    rows = []
    monotonic = []
    print("capturing raw D500 packets for %.0f s on %s ..." % (args.seconds, args.port))
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.seconds:
            chunk = __import__("os").read(fd, 4096)
            if not chunk:
                continue
            for packet in decoder.feed(chunk):
                now = time.monotonic()
                rows.append((packet.timestamp_ms, packet.rotation_speed_deg_s, now))
                monotonic.append(now)
    finally:
        import os

        os.close(fd)

    if len(rows) < 50:
        print("FAIL: only %d packets captured" % len(rows))
        return 2

    stamps = [r[0] for r in rows]
    print()
    print("=== raw capture ===")
    print("  packets            : %d over %.1f s" % (len(rows), monotonic[-1] - monotonic[0]))
    print("  packet rate        : %.1f Hz" % (len(rows) / (monotonic[-1] - monotonic[0])))
    print("  raw min / max      : %d / %d" % (min(stamps), max(stamps)))
    print("  distinct values    : %d" % len(set(stamps)))

    # Per-packet increment statistics.
    deltas = [b - a for a, b in zip(stamps, stamps[1:])]
    negative = [d for d in deltas if d < 0]
    print()
    print("=== per-packet delta ===")
    print("  min / median / max : %d / %.0f / %d"
          % (min(deltas), statistics.median(deltas), max(deltas)))
    print("  negative deltas    : %d" % len(negative))
    for d in sorted(set(negative))[:10]:
        print("     %d x delta=%d" % (negative.count(d), d))

    # Detect wraps: a large negative jump followed by a small positive jump.
    print()
    print("=== candidate rollovers ===")
    wraps = []
    for index, delta in enumerate(deltas):
        if delta < -1000:
            before = stamps[index]
            after = stamps[index + 1]
            wraps.append((index, before, after, delta))
    if not wraps:
        print("  none observed in this window")
    for index, before, after, delta in wraps[:10]:
        implied_modulus = before - after + delta if False else (before + (-delta - before + after))
        print("  at packet %d: %d -> %d (delta %d)" % (index, before, after, delta))
        print("     implied modulus from (before + modulus == after): %d"
              % (before + (after - before) + (-delta)))

    # Highest observed value bounds the modulus from below.
    print()
    print("=== modulus bounds ===")
    print("  max raw observed   : %d" % max(stamps))
    print("  if uint16 ms, wrap every %.1f s at this rate"
          % (65536.0 / (len(rows) / (monotonic[-1] - monotonic[0]))))

    # Is the counter advancing in real milliseconds?
    span_raw = stamps[-1] - stamps[0]
    if wraps:
        span_raw += 65536 * len(wraps)
    span_s = monotonic[-1] - monotonic[0]
    print("  raw span (unwrapped, assuming uint16): %d" % span_raw)
    print("  host span                            : %.1f s" % span_s)
    if span_s > 0:
        print("  raw ticks per host second            : %.1f"
              % (span_raw / span_s))
        print("  -> if this is ~1000 the unit really is milliseconds")

    with open(args.out, "w", encoding="utf-8") as handle:
        handle.write("raw_ms,speed,monotonic_s\n")
        for raw, speed, now in rows:
            handle.write("%d,%d,%.6f\n" % (raw, speed, now))
    print()
    print("  raw CSV written to %s" % args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
