#!/usr/bin/env python3
"""Read-only C10B telemetry probe: aligned frames, battery voltage, motor flag.

The WHEELTEC C10B firmware streams 24-byte telemetry frames on the same port as
the velocity commands.  This probe opens that port read-only and never writes or
flushes, so it can run alongside the rear-motor command writer.

Use it to answer "is the controller actually driving?" without moving anything:
run it while a supervised pulse is commanding a known Vx/Vz and compare the
telemetry fields.  A field that never follows the command means the firmware is
not receiving or not accepting the frame, which is a hardware/firmware problem
rather than a tuning problem.
"""

from __future__ import annotations

import argparse
import collections
import json
import os
import select
import struct
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "code"))

from components.battery_voltage_monitor import (  # noqa: E402
    BATTERY_HIGH_INDEX,
    BATTERY_LOW_INDEX,
    TELEMETRY_CHECKSUM_INDEX,
    TELEMETRY_HEADER,
    TELEMETRY_LENGTH,
    TELEMETRY_TAIL,
    _open_c10b_telemetry_port,
    _xor_checksum,
    decode_stock_c10b_voltage_v,
)


def _aligned_frames(buffer: bytearray) -> list[bytes]:
    """Pop complete, checksum-verified 24-byte telemetry frames from *buffer*."""
    frames: list[bytes] = []
    while True:
        start = buffer.find(TELEMETRY_HEADER)
        if start < 0:
            buffer.clear()
            return frames
        if start:
            del buffer[:start]
        if len(buffer) < TELEMETRY_LENGTH:
            return frames
        candidate = bytes(buffer[:TELEMETRY_LENGTH])
        if (candidate[-1] != TELEMETRY_TAIL
                or _xor_checksum(candidate[:TELEMETRY_CHECKSUM_INDEX]) != candidate[TELEMETRY_CHECKSUM_INDEX]):
            del buffer[0]
            continue
        frames.append(candidate)
        del buffer[:TELEMETRY_LENGTH]


def telemetry_row(frame: bytes, voltage_v: float | None, *, t_mono: float, t_s: float) -> dict:
    """One JSON-ready trace row for a checksum-verified 24-byte telemetry frame."""
    return {
        "t_mono": round(t_mono, 6),
        "t_s": round(t_s, 4),
        "motor_disabled": bool(frame[1]),
        "int16_be_fields": [
            struct.unpack_from(">h", frame, offset)[0] for offset in range(2, 20, 2)],
        "voltage_v": round(voltage_v, 3) if voltage_v is not None else None,
        "hex": frame.hex(),
    }


def probe(device: str, seconds: float, max_distinct: int,
          trace_path: str | None = None) -> dict:
    fd = _open_c10b_telemetry_port(device)
    buffer = bytearray()
    distinct: collections.OrderedDict = collections.OrderedDict()
    frames = 0
    volts: list[float] = []
    disabled: collections.Counter = collections.Counter()
    # The per-frame trace is what makes a stall diagnosable: the distinct-frame
    # summary collapses time, so it cannot say *when* the firmware stopped
    # following the command.  Read-only, same port, one JSON object per line.
    trace = None
    if trace_path:
        trace = open(trace_path, "w", encoding="utf-8")
    started = time.monotonic()
    try:
        while time.monotonic() - started < seconds:
            readable, _, _ = select.select([fd], [], [], 0.2)
            if not readable:
                continue
            try:
                buffer.extend(os.read(fd, 1024))
            except BlockingIOError:
                continue
            for frame in _aligned_frames(buffer):
                frames += 1
                disabled[bool(frame[1])] += 1
                try:
                    volts.append(decode_stock_c10b_voltage_v(
                        (frame[BATTERY_HIGH_INDEX] << 8) | frame[BATTERY_LOW_INDEX]))
                except Exception:  # noqa: BLE001 - an undecodable field is itself a finding
                    pass
                if trace is not None:
                    now = time.monotonic()
                    trace.write(json.dumps(telemetry_row(
                        frame, volts[-1] if volts else None,
                        t_mono=now, t_s=now - started)) + "\n")
                key = frame.hex()
                if key in distinct:
                    distinct[key]["count"] += 1
                elif len(distinct) < max_distinct:
                    distinct[key] = {"count": 1, "first_seen_s": round(time.monotonic() - started, 3)}
    finally:
        if trace is not None:
            trace.close()
        os.close(fd)
    elapsed = time.monotonic() - started
    return {
        "device": device,
        "seconds": round(elapsed, 2),
        "frames": frames,
        "frame_hz": round(frames / elapsed, 2) if elapsed > 0 else None,
        "trace_path": trace_path,
        "motor_disabled_counts": {str(key): value for key, value in disabled.items()},
        "voltage_last_v": round(volts[-1], 2) if volts else None,
        "voltage_min_v": round(min(volts), 2) if volts else None,
        "voltage_max_v": round(max(volts), 2) if volts else None,
        "distinct_frames": [
            {
                "count": entry["count"],
                "first_seen_s": entry["first_seen_s"],
                "hex": " ".join("%02X" % value for value in bytes.fromhex(key)),
                "byte_1_motor_flag": bytes.fromhex(key)[1],
                "int16_be_fields": {str(offset): struct.unpack_from(">h", bytes.fromhex(key), offset)[0]
                                    for offset in range(2, 20, 2)},
            }
            for key, entry in distinct.items()
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--max-distinct", type=int, default=12)
    parser.add_argument("--trace", help="write one JSON object per received frame to this path")
    args = parser.parse_args()
    if not 0.5 <= args.seconds <= 120.0:
        parser.error("--seconds must be between 0.5 and 120")
    if not 1 <= args.max_distinct <= 64:
        parser.error("--max-distinct must be between 1 and 64")
    print(json.dumps(probe(args.device, args.seconds, args.max_distinct, args.trace), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
