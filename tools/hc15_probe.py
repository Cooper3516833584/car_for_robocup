#!/usr/bin/env python3
"""Bounded HC-15 UART receive probe; sends no bytes or radio AT commands."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.serial_communication import DEFAULT_HC15_PORT, HC15SerialDriver


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_HC15_PORT)
    parser.add_argument("--baudrate", type=int, choices=(9600, 115200), default=115200)
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--connect-timeout-s", type=float, default=3.0)
    args = parser.parse_args(argv)
    for name, value, limit in (
        ("duration-s", args.duration_s, 60.0),
        ("connect-timeout-s", args.connect_timeout_s, 30.0),
    ):
        if not math.isfinite(value) or not 0 < value <= limit:
            parser.error(f"--{name} must be finite and in (0, {limit}]")

    received_bytes = 0
    lock = threading.Lock()

    def receive(data: bytes) -> None:
        nonlocal received_bytes
        with lock:
            received_bytes += len(data)
        print(f"RX bytes={len(data)} hex={data[:32].hex()}", flush=True)

    try:
        with HC15SerialDriver(
            on_bytes=receive, port=args.port, baudrate=args.baudrate,
            bridge_envelope=False,
        ) as link:
            if not link.wait_connected(args.connect_timeout_s):
                print(f"Cannot open {args.port}: {link.last_error}", file=sys.stderr)
                print("Check UART4-M2 overlay, dialout permissions and port ownership.", file=sys.stderr)
                return 1
            print(f"Opened {args.port} {args.baudrate} 8N1; receive only", flush=True)
            deadline = time.monotonic() + args.duration_s
            while time.monotonic() < deadline:
                if not link.connected:
                    print(f"Link disconnected: {link.last_error}", file=sys.stderr)
                    return 1
                time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        return 130
    print(f"Port check complete; received_bytes={received_bytes}. "
          "Opening the port does not verify the radio peer or air link.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
