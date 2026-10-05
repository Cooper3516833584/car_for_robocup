#!/usr/bin/env python3
"""Bounded end-to-end check of the C10B low-voltage -> sound/light alarm chain.

The production threshold (11.0 V) can only be triggered by a genuinely flat
pack, so this probe forces the same production code path instead: it reads the
real battery voltage, then runs ``LowBatteryMonitor`` with a threshold above the
pack voltage and a single required low reading, and checks that the GPIO line
physically goes active (active-low wiring: raw 0) before silencing it again.

The alarm sounds for ``--alarm-seconds`` (default 1.5 s) and is silenced on every
exit path, including failures and Ctrl-C.  This writes only the alarm line; it
never touches the motors or the C10B transmit direction (the telemetry port is
opened read-only).

Run on the board as root, because the sysfs line is root-owned:

    sudo PYTHONPATH=/home/radxa/car/code python3 tools/battery_alarm_chain_test.py

Usage:
    sudo PYTHONPATH=/home/radxa/car/code python3 tools/battery_alarm_chain_test.py \
        [--device /dev/ttyACM0] [--alarm-seconds 1.5]
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.battery_voltage_monitor import (  # noqa: E402
    C10BBatteryVoltageReader,
    LowBatteryMonitor,
)
from components.sound_light_alarm import SoundLightAlarm  # noqa: E402

RAW_ACTIVE = 0  # verified ROCK 5A wiring: the alarm is active-low
RAW_INACTIVE = 1


def read_pin(alarm: SoundLightAlarm) -> int:
    """Read the raw sysfs level, independent of the active-low abstraction."""

    path = Path(f"/sys/class/gpio/gpio{alarm.gpio_number}/value")
    return int(path.read_text(encoding="ascii").strip())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument(
        "--force-threshold-v",
        type=float,
        default=20.0,
        help="threshold used to force the low-voltage branch (above any real pack)",
    )
    parser.add_argument("--alarm-seconds", type=float, default=1.5)
    parser.add_argument("--read-timeout-s", type=float, default=2.0)
    args = parser.parse_args()

    if args.alarm_seconds <= 0.0:
        print("--alarm-seconds must be positive", file=sys.stderr)
        return 2

    alarm = SoundLightAlarm().initialize(active=False)
    print(f"[chain] GPIO{alarm.gpio_number} initialized silent (raw={read_pin(alarm)})")
    if read_pin(alarm) != RAW_INACTIVE:
        print(
            "[chain] FAIL: the alarm line is stuck active before the test",
            file=sys.stderr,
        )
        return 1

    try:
        with C10BBatteryVoltageReader(args.device) as reader:
            sample = reader.read_sample(args.read_timeout_s)
            print(
                f"[chain] {args.device} reports {sample.voltage_v:.2f} V "
                f"(raw=0x{sample.raw_value:04X}); production threshold is 11.00 V"
            )

            monitor = LowBatteryMonitor(
                reader,
                alarm,
                threshold_v=args.force_threshold_v,
                required_consecutive_low=1,
                read_timeout_s=args.read_timeout_s,
            )
            monitor.poll_once()
            if not monitor.alarm_active:
                print(
                    "[chain] FAIL: the monitor did not activate the alarm below "
                    f"{args.force_threshold_v:.2f} V",
                    file=sys.stderr,
                )
                return 1

            raw = read_pin(alarm)
            print(
                f"[chain] forced low-voltage trigger: alarm_active={monitor.alarm_active} "
                f"raw={raw}"
            )
            if raw != RAW_ACTIVE:
                print(
                    "[chain] FAIL: the GPIO did not go low, so the alarm cannot sound "
                    "(check polarity/active_low and the 40-pin connection)",
                    file=sys.stderr,
                )
                return 1

            time.sleep(args.alarm_seconds)
    finally:
        alarm.off()
        print(f"[chain] silenced (raw={read_pin(alarm)})")

    if read_pin(alarm) != RAW_INACTIVE:
        print("[chain] FAIL: the alarm did not return to the silent level", file=sys.stderr)
        return 1

    print("[chain] PASS: C10B telemetry -> threshold -> GPIO alarm, then silenced")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
