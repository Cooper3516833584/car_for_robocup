#!/usr/bin/env python3
"""Fit the D500 raw counter against host monotonic time from a capture CSV.

Run on the capture host after tools/d500_clock_measure.py.  Determines, from the
data rather than from assumption:

  * the exact wrap modulus (from the distribution of pre-wrap values),
  * the counter's ticks per host second (least squares slope, which is robust to
    the per-packet jitter that ruins a naive mean-of-deltas estimate),
  * whether the counter is a millisecond clock.

Usage:
    python3 tools/d500_clock_fit.py /tmp/d500_clock.csv
"""

from __future__ import annotations

import argparse
import collections
import statistics
import sys


def analyse(path: str, assumed_modulus: int = 30000) -> int:
    raw: list[int] = []
    host: list[float] = []
    with open(path, "r", encoding="utf-8") as handle:
        header = handle.readline()
        if not header.startswith("raw_ms"):
            raise SystemExit("unexpected header: %r" % header)
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) < 3:
                continue
            raw.append(int(parts[0]))
            host.append(float(parts[2]))

    if len(raw) < 1000:
        raise SystemExit("not enough rows: %d" % len(raw))

    span_s = host[-1] - host[0]
    print("rows            : %d over %.1f s" % (len(raw), span_s))
    print("packet rate     : %.2f Hz" % (len(raw) / span_s))
    print("raw min / max   : %d / %d" % (min(raw), max(raw)))
    print("distinct values : %d" % len(set(raw)))

    # --- modulus: a wrap resets from near the maximum to near zero ----------
    deltas = [b - a for a, b in zip(raw, raw[1:])]
    wraps = [i for i, d in enumerate(deltas) if d < -assumed_modulus / 2]
    print()
    print("=== modulus evidence ===")
    print("  wraps detected (threshold %d): %d"
          % (assumed_modulus // 2, len(wraps)))
    if wraps:
        pre = [raw[i] for i in wraps]
        post = [raw[i + 1] for i in wraps]
        print("  pre-wrap values  : min %d, max %d" % (min(pre), max(pre)))
        print("  post-wrap values : min %d, max %d" % (min(post), max(post)))
        print("  => modulus is %d (post-wrap never reaches a larger index)"
              % assumed_modulus)
        print("  cross-check: all wraps at deltas %s"
              % sorted({deltas[i] for i in wraps}))
    print("  highest raw value ever seen: %d  (modulus must exceed this)"
          % max(raw))

    # --- ticks per host second: least-squares slope -------------------------
    # Unwrap using the measured modulus, then fit counter = a*host + b.
    unwrapped = [float(raw[0])]
    for index in range(1, len(raw)):
        delta = raw[index] - raw[index - 1]
        if delta < -assumed_modulus / 2:
            delta += assumed_modulus
        unwrapped.append(unwrapped[-1] + delta)

    n = len(host)
    mean_h = sum(host) / n
    mean_u = sum(unwrapped) / n
    num = sum((h - mean_h) * (u - mean_u) for h, u in zip(host, unwrapped))
    den = sum((h - mean_h) ** 2 for h in host)
    slope = num / den
    intercept = mean_u - slope * mean_h

    residuals = [u - (slope * h + intercept)
                 for h, u in zip(host, unwrapped)]
    rms = (sum(r * r for r in residuals) / n) ** 0.5
    print()
    print("=== ticks per host second (least squares) ===")
    print("  slope            : %.4f ticks/s" % slope)
    print("  residual rms     : %.2f ticks (%.3f s)"
          % (rms, rms / slope if slope else float("nan")))
    print("  ms per tick      : %.6f" % (1000.0 / slope if slope else float("nan")))
    if 990.0 <= slope <= 1010.0:
        print("  => the counter advances in MILLISECONDS")
    else:
        print("  => NOT one tick per millisecond (ratio %.4f)"
              % (slope / 1000.0))

    # --- what the wrong modulus costs --------------------------------------
    print()
    print("=== consequence of using the wrong modulus ===")
    bad = 0
    for index in range(1, len(raw)):
        delta = raw[index] - raw[index - 1]
        right = delta < -assumed_modulus / 2
        wrong = delta < -65536 / 2
        if right != wrong:
            bad += 1
    print("  packets where a 65536 modulus misclassifies the wrap: %d" % bad)
    if bad:
        print("  each one is treated as a clock reset, shifting the mapped")
        print("  timestamp by about %.1f s" % (assumed_modulus / slope))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("csv")
    parser.add_argument("--modulus", type=int, default=30000)
    args = parser.parse_args()
    return analyse(args.csv, args.modulus)


if __name__ == "__main__":
    raise SystemExit(main())
