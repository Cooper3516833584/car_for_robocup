#!/usr/bin/env python3
"""Measure the torque a servo axis must hold against, and where it rests free.

A pan axis that springs away from its commanded angle *after* ``release()`` is
fighting a mechanical return force (a cable bundle, gravity, a rubber band).
This probe separates the three candidate causes by measurement, so the fix is
chosen from data instead of guessed:

1. **Does the axis hold its commanded angle while enabled?** A slow drift while
   the pulse train is on means the servo's own loop or feedback is at fault.
2. **How much current does holding cost?** The C10B telemetry reports pack
   voltage; holding against a spring costs more current than resting, which
   shows up as a lower pack voltage. This is a *relative* instrument: absolute
   volts from this board are not trustworthy, only the comparison between
   phases. **Measured limit:** on the current rig a small servo holding a light
   camera platform moved the reading by 0.01 V, i.e. nothing - the telemetry is
   far too coarse to size a small return force. Treat a flat reading as
   "inconclusive", not as "no load", and judge the mechanism by eye.
3. **Can the command even reach the target?** Stepping the pulse and reading the
   pack voltage says whether the servo is stalling (voltage sags hard) or
   sitting comfortably (voltage flat).

It never claims to measure position: this rig has no encoder on the servo, so
the mechanical angle has to be observed by a person. The probe prints exactly
what to watch for next to each phase.

Run on the board, as root:

    sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_backdrive_probe.py \
        --confirm --angle 0

The C10B telemetry port is opened read-only, and the probe never commands the
motors. The C10B board's own KEY2 enables the motors; leave it off.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from components.servo_axis import (  # noqa: E402
    MG90S_TRAVEL,
    ROCK5A_PWM7_M0_CHIP,
    ServoAxis,
    ServoError,
    ServoRangeError,
    ServoTravel,
)
from hal.pwm import LinuxSysfsPWMOutput, PWMBackendError  # noqa: E402


def pack_voltage_reader(device: str):
    """Return a callable sampling pack voltage, or ``None`` if unavailable.

    ``battery_voltage_monitor`` opens the same C10B port read-only, so this is
    usable alongside (but not instead of) a running mission. Note the board's
    absolute voltage scale is not trustworthy; only the comparison between
    phases below is meaningful.
    """

    try:
        from components.battery_voltage_monitor import C10BBatteryVoltageReader

        return C10BBatteryVoltageReader(device=device).start()
    except Exception:  # noqa: BLE001 - the probe works without telemetry
        return None


def sample_voltage(reader, seconds: float, samples: int = 5) -> float | None:
    """Median pack voltage over a short window, or ``None`` without telemetry."""

    if reader is None:
        return None
    readings: list[float] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline and len(readings) < samples:
        try:
            sample = reader.read_sample(timeout_s=0.5)
        except Exception:  # noqa: BLE001
            return None
        if sample is not None:
            readings.append(float(sample.voltage_v))
    return statistics.median(readings) if readings else None


def volts(value: float | None) -> str:
    return "n/a (no telemetry)" if value is None else f"{value:.3f} V"


def phase(title: str, instruction: str) -> None:
    print(f"\n=== {title} ===")
    print(f"  watch: {instruction}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--confirm", action="store_true",
                        help="required: actually write the PWM channel")
    parser.add_argument("--angle", type=float, default=0.0,
                        help="angle to test holding at; 0 is the mid pulse")
    parser.add_argument("--hold-s", type=float, default=8.0,
                        help="seconds to hold each phase while watching")
    parser.add_argument("--c10b", default="/dev/ttyACM0",
                        help="C10B telemetry port for pack-voltage comparison")
    parser.add_argument("--no-telemetry", action="store_true",
                        help="skip the pack-voltage comparison entirely")
    parser.add_argument("--chip", action="append", default=None,
                        help="PWM chip selector; repeatable")
    parser.add_argument("--channel", type=int, default=0, help="PWM channel index")
    parser.add_argument("--pwm-path", default=None,
                        help="explicit /sys/class/pwm/pwmchipN/pwmM channel directory")
    parser.add_argument("--period-us", type=int, default=MG90S_TRAVEL.period_us)
    parser.add_argument("--pulse-min-us", type=int, default=MG90S_TRAVEL.pulse_min_us)
    parser.add_argument("--pulse-max-us", type=int, default=MG90S_TRAVEL.pulse_max_us)
    parser.add_argument("--half-range-deg", type=float, default=MG90S_TRAVEL.half_range_deg)
    parser.add_argument("--settle-s", type=float, default=0.5)
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

    print("This probe cannot see the arm: it has no position feedback, so the")
    print("angle has to be judged by eye. Each phase below says what to look for.")
    print(f"\ntarget angle : {args.angle:g} deg -> {travel.pulse_mid_us + args.angle * travel.us_per_deg:.0f} us")
    print(f"travel       : +/-{travel.half_range_deg:g} deg over "
          f"{travel.pulse_min_us}..{travel.pulse_max_us} us")

    if not args.confirm:
        print("\ndry run: pass --confirm to write the PWM channel")
        return 0

    reader = None if args.no_telemetry else pack_voltage_reader(args.c10b)
    if not args.no_telemetry:
        state = "available" if reader is not None else "unavailable (voltage comparison skipped)"
        print(f"telemetry    : {state}")

    output = LinuxSysfsPWMOutput(
        chip_device_match=selectors,
        channel=args.channel,
        period_ns=travel.period_us * 1000,
        pwm_path=args.pwm_path,
    )
    axis = ServoAxis(travel, pwm=output, settle_s=args.settle_s)
    exit_code = 0
    try:
        # Phase 0: baseline with the channel off, so the pack is under no servo
        # load at all. This is the reference every other phase is compared to.
        phase("idle, channel disabled", "the arm sits wherever the load leaves it")
        idle_voltage = sample_voltage(reader, args.hold_s)
        print(f"  pack voltage : {volts(idle_voltage)}")
        print("  note the resting angle the arm has settled at (this is the "
              "spring-back target).")

        axis.start(home=False, sweep_start=True)
        pulse = axis.set_angle(args.angle, settle=True)
        phase(f"holding {args.angle:g} deg ({pulse} us), pulse train enabled",
              "does the arm reach and stay at the commanded angle, or does it "
              "drift/buzz?")
        hold_voltage = sample_voltage(reader, args.hold_s)
        print(f"  pack voltage : {volts(hold_voltage)}")

        # Phase 2: same angle, but released. The drop here is the mechanical
        # return force doing work; the servo is no longer resisting it.
        axis.release()
        phase("released (enable=0), same pulse still latched in sysfs",
              "where does the arm end up now? If it moves, the load is pulling it "
              "and the servo was the only thing holding it.")
        released_voltage = sample_voltage(reader, args.hold_s)
        print(f"  pack voltage : {volts(released_voltage)}")

        if idle_voltage is not None and hold_voltage is not None:
            sag = idle_voltage - hold_voltage
            note = "measurable load" if abs(sag) >= 0.05 else "too small to read (inconclusive)"
            print(f"\nholding cost : {sag:+.3f} V versus idle ({note})")
            print("  The C10B telemetry resolves roughly tens of millivolts, so a")
            print("  small servo holding a light platform reads as no change. Only a")
            print("  clear, repeatable sag proves the servo is working hard.")

        print("\ninterpretation:")
        print("  held steady + dropped on release  -> mechanical return force (cable")
        print("                                       bundle, gravity) is the only thing")
        print("                                       the servo is fighting; fix the")
        print("                                       routing, or park under power")
        print("  drifted while enabled             -> servo loop or feedback fault;")
        print("                                       try another servo/pulse range")
        print("  never reached the angle           -> mechanical stop or jam; check")
        print("                                       the horn mounting and travel")
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
        axis.release()
        if reader is not None:
            try:
                reader.close()
            except Exception:  # noqa: BLE001
                pass
        print("\npulse train released (the arm is now free)")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
