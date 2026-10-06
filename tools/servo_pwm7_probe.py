#!/usr/bin/env python3
"""Supervised probe for the PWM hobby servo on ROCK 5A PWM7_IR_M0 (Pin 28).

What it does, in order:

1. resolves the PWM chip that owns the servo pin and prints the matching device
   node, so a wrong or missing overlay is visible before anything moves;
2. drives the axis to ``--home`` degrees (default ``0``, the mid pulse) and holds;
3. optionally sweeps ``--sweep`` angles;
4. releases the pulse train (or holds with ``--keep-enabled``).

The probe never touches the motors, the C10B port, or any other actuator: it
only writes one PWM channel. It requires an explicit ``--confirm`` before it
writes hardware, and it releases the channel on every exit path including
Ctrl-C and failures, so a failed run cannot leave the servo buzzing.

Run on the board, as root (the sysfs channel is root-owned unless a udev rule
grants the ``gpio`` group):

    sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_pwm7_probe.py --confirm

The same component is importable from an interactive session::

    from components.servo_axis import MG90S_TRAVEL, ServoAxis
    from hal.pwm import LinuxSysfsPWMOutput

See ``docs/SERVO_PWM7_M0.md``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.servo_axis import (  # noqa: E402
    DEFAULT_SWEEP_STEP_DEG,
    MG90S_TRAVEL,
    ROCK5A_PWM7_M0_CHIP,
    ROCK5A_PWM7_M0_OVERLAY,
    ROCK5A_PWM7_M0_PHYSICAL_PIN,
    ServoAxis,
    ServoError,
    ServoRangeError,
    ServoTravel,
)
from hal.pwm import LinuxSysfsPWMOutput, PWMBackendError, resolve_chip  # noqa: E402


def describe_chip(chip: Path) -> str:
    """Return a one-line description of a ``pwmchipN`` entry, or why it failed."""

    try:
        resolved = chip.resolve()
    except OSError as exc:  # pragma: no cover - depends on board state
        return f"{chip} (unresolvable: {exc})"
    try:
        channels = (chip / "npwm").read_text(encoding="ascii").strip()
    except OSError:
        channels = "?"
    return f"{chip} -> {resolved} (npwm={channels})"


def report_chip_selection(selectors: tuple[str, ...]) -> Path | None:
    """Print every PWM chip and which one the selectors pick."""

    root = Path("/sys/class/pwm")
    chips = sorted(root.glob("pwmchip*"))
    print(f"PWM chips under {root}:")
    if not chips:
        print("  none: no PWM controller is exported at all")
    for chip in chips:
        print(f"  {describe_chip(chip)}")
    try:
        selected = resolve_chip(root, selectors)
    except PWMBackendError as exc:
        print(f"\nservo pin {ROCK5A_PWM7_M0_PHYSICAL_PIN} ({ROCK5A_PWM7_M0_CHIP}): NOT available")
        print(f"  {exc}")
        print(f"  expected overlay: {ROCK5A_PWM7_M0_OVERLAY} (docs/SERVO_PWM7_M0.md)")
        return None
    print(f"\nservo selectors {selectors} resolved to {selected}")
    return selected


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--confirm", action="store_true",
                        help="required: actually write the PWM channel")
    parser.add_argument("--home", type=float, default=0.0,
                        help="angle applied first; 0 is the mid pulse (1500 us)")
    parser.add_argument("--sweep", type=float, nargs="*", default=[],
                        help="angles to sweep through at --rate-deg-s")
    parser.add_argument("--rate-deg-s", type=float, default=30.0,
                        help="sweep speed in degrees per second")
    parser.add_argument("--step-deg", type=float, default=DEFAULT_SWEEP_STEP_DEG,
                        help="interpolation step; smaller is smoother, slower to schedule")
    parser.add_argument("--return", dest="return_to_start", action="store_true",
                        help="sweep back to the starting angle at the same rate")
    parser.add_argument("--hold-s", type=float, default=0.5,
                        help="seconds to hold each position before the next move")
    parser.add_argument("--keep-enabled", action="store_true",
                        help="leave the pulse train running at the final angle "
                             "(required to hold an axis that is spring-loaded)")
    parser.add_argument("--release", action="store_true",
                        help="enable, write the pulse, then disable and exit, so the "
                             "arm's free resting angle can be compared with the command")
    parser.add_argument("--chip", action="append", default=None,
                        help="PWM chip selector; repeatable. Defaults to the servo config")
    parser.add_argument("--channel", type=int, default=0, help="PWM channel index")
    parser.add_argument("--pwm-path", default=None,
                        help="explicit /sys/class/pwm/pwmchipN/pwmM channel directory")
    parser.add_argument("--period-us", type=int, default=MG90S_TRAVEL.period_us)
    parser.add_argument("--pulse-min-us", type=int, default=MG90S_TRAVEL.pulse_min_us)
    parser.add_argument("--pulse-max-us", type=int, default=MG90S_TRAVEL.pulse_max_us)
    parser.add_argument("--half-range-deg", type=float, default=MG90S_TRAVEL.half_range_deg)
    parser.add_argument("--settle-s", type=float, default=0.4,
                        help="seconds allowed after each move before reading back")
    args = parser.parse_args()

    selectors = tuple(args.chip) if args.chip else (ROCK5A_PWM7_M0_CHIP,)

    try:
        travel = ServoTravel(
            half_range_deg=args.half_range_deg,
            pulse_min_us=args.pulse_min_us,
            pulse_max_us=args.pulse_max_us,
            period_us=args.period_us,
        )
    except ServoRangeError as exc:
        print(f"invalid calibration: {exc}", file=sys.stderr)
        return 2

    if not args.confirm:
        print("dry run: pass --confirm to write the PWM channel\n")
        print(f"pin            : {ROCK5A_PWM7_M0_PHYSICAL_PIN} ({ROCK5A_PWM7_M0_CHIP})")
        print(f"overlay        : {ROCK5A_PWM7_M0_OVERLAY}")
        print(f"travel         : +/-{travel.half_range_deg:g} deg over "
              f"{travel.pulse_min_us}..{travel.pulse_max_us} us at {travel.period_us} us frame")
        print(f"0 deg pulse    : {travel.pulse_mid_us:g} us")
        print(f"home angle     : {args.home:g} deg -> "
              f"{travel.pulse_mid_us + args.home * travel.us_per_deg:.0f} us")
        report_chip_selection(selectors)
        return 0

    selected = report_chip_selection(selectors)
    if selected is None and not args.pwm_path:
        return 1

    output = LinuxSysfsPWMOutput(
        chip_device_match=selectors,
        channel=args.channel,
        period_ns=travel.period_us * 1000,
        pwm_path=args.pwm_path,
    )
    # The start angle doubles as the servo's home/return target, so the axis is
    # built with it: return_to_start() then goes back to --home by construction.
    axis = ServoAxis(travel, pwm=output, settle_s=args.settle_s,
                     home_angle_deg=args.home)
    exit_code = 0
    try:
        # Record the origin without writing a pulse, so the explicit move below is
        # the first command and is not blocked by the command rate limiter.
        axis.start(home=False, sweep_start=True)
        print(f"enabled  : {describe_chip(selected) if selected else args.pwm_path}")

        pulse = axis.set_angle(args.home, settle=True)
        print(f"home     : {args.home:g} deg -> {pulse} us")

        previous = args.home
        for angle in args.sweep:
            time.sleep(max(args.hold_s, axis.min_command_interval_s + 0.01))
            started = time.monotonic()
            pulse = axis.sweep_to(angle, rate_deg_s=args.rate_deg_s,
                                  step_deg=args.step_deg, settle=True)
            elapsed = time.monotonic() - started
            span = abs(angle - previous)
            print(f"sweep    : {previous:g} -> {angle:g} deg at {args.rate_deg_s:g} deg/s "
                  f"-> {pulse} us (moved {span:g} deg in {elapsed:.2f} s, "
                  f"expected {span / args.rate_deg_s:.2f} s)")
            previous = angle

        if args.return_to_start:
            time.sleep(max(args.hold_s, axis.min_command_interval_s + 0.01))
            started = time.monotonic()
            pulse = axis.return_to_start(rate_deg_s=args.rate_deg_s,
                                        step_deg=args.step_deg, settle=True)
            elapsed = time.monotonic() - started
            arrived = axis.angle_deg
            span = abs(previous - arrived)
            print(f"return   : {previous:g} -> {arrived:g} deg at {args.rate_deg_s:g} deg/s "
                  f"-> {pulse} us (moved {span:g} deg in {elapsed:.2f} s, "
                  f"expected {span / args.rate_deg_s:.2f} s)")
        time.sleep(max(args.hold_s, axis.min_command_interval_s + 0.01))
    except ServoError as exc:
        print(f"servo command failed: {exc}", file=sys.stderr)
        exit_code = 1
    except PWMBackendError as exc:
        print(f"PWM backend failed: {exc}", file=sys.stderr)
        exit_code = 1
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        exit_code = 130
    finally:
        if args.keep_enabled:
            print(f"left enabled at {axis.angle_deg} deg ({axis.pulse_us} us)")
        else:
            axis.release()
            print("pulse train released (the arm is now free)")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
