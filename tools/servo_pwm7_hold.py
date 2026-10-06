#!/usr/bin/env python3
"""Park one PWM servo axis at an angle and keep holding it.

Why this exists: a servo only resists its mechanical load while its pulse train
is enabled, so "leave it at 0 degrees" needs a process that keeps writing. Two
board behaviours make a foreground command a bad tool for that job:

1. the kernel keeps outputting the last duty cycle after the process exits, so a
   killed holder leaves the axis *enabled wherever it was* - which is safer than
   dropping it, but is not a request you can rely on either way;
2. anything run over SSH without ``setsid`` dies with the session, so the hold
   silently ends and the arm is left latched at whatever the last pulse was.

**Holding is the default**, because a hobby servo has no holding torque at all
once its pulse train stops: a pan axis carrying a camera loses its commanded
angle to the cable's return force, and holding costs only the servo's idle
current (tens of milliamps for a small servo on a light payload). Passing
``--release`` opts out and leaves the axis free.

This tool detaches itself and keeps the axis parked until a deadline, a stop file
appears, or it is killed. It writes a log so the state is inspectable after the
fact.

    # park at 0 degrees (mid pulse) and hold until stopped
    sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_pwm7_hold.py --park 0

    # hold for 30 minutes, then release
    sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_pwm7_hold.py --park 0 --minutes 30

    # move somewhere and deliberately let the axis go free
    sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_pwm7_hold.py --park -30 --release

    # stop a running hold and release the axis
    sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_pwm7_hold.py --stop --release

The C10B port, the motors and every other actuator are untouched.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import os
import signal
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

# Defaults are resolved at run time by runtime_dir(); see its docstring for why a
# fixed /tmp path is wrong when both root and a group member use this tool.
DEFAULT_LOG_NAME = "servo_pwm7_hold.log"
DEFAULT_PID_NAME = "servo_pwm7_hold.pid"
DEFAULT_STOP_NAME = "servo_pwm7_hold.stop"


def log(message: str, log_path: str) -> None:
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}"
    print(line, flush=True)
    try:
        with open(log_path, "a", encoding="utf-8") as stream:
            stream.write(line + "\n")
    except OSError:
        pass


def read_pid(pid_path: str) -> int | None:
    try:
        return int(Path(pid_path).read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return None


def process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def build_axis(args: argparse.Namespace, travel: ServoTravel) -> ServoAxis:
    output = LinuxSysfsPWMOutput(
        chip_device_match=tuple(args.chip),
        channel=args.channel,
        period_ns=travel.period_us * 1000,
        pwm_path=args.pwm_path,
    )
    return ServoAxis(travel, pwm=output, settle_s=args.settle_s, home_angle_deg=args.park)


def runtime_dir() -> Path:
    """First writable directory for the pid/log/stop files.

    These must not collide with an earlier ``sudo`` run: root-owned files in
    ``/tmp`` are not writable by the operator, and a ``PermissionError`` while
    starting a hold would leave the axis in whatever state it was already in.
    The servo's sysfs channel is group ``pwm`` and world-unwritable, so an
    operator in that group does not need root at all and gets a user directory.
    """

    candidates = [
        Path("/run") if os.geteuid() == 0 else None,
        Path("/run/user") / str(os.geteuid()),
        Path("/tmp") / f"servo-pwm7-hold-{os.geteuid()}",
        Path.home() / ".cache" / "servo-pwm7-hold",
    ]
    for candidate in candidates:
        if candidate is None:
            continue
        try:
            candidate.mkdir(parents=True, exist_ok=True)
            probe = candidate / ".write-probe"
            probe.write_text("", encoding="utf-8")
            probe.unlink()
            return candidate
        except OSError:
            continue
    raise SystemExit(
        "no writable directory for the hold tool's runtime files; "
        "pass --log/--pid-file/--stop-file explicitly"
    )


def stop_running(pid_path: str, stop_path: str) -> int:
    """Ask a detached holder to finish and release the axis.

    The holder releases on its way out, which is what "stop" should mean: an
    operator stopping a hold wants the axis free, not latched at the last angle.
    """

    stop = Path(stop_path)
    try:
        stop.write_text("stop\n", encoding="utf-8")
        print(f"stop file written: {stop_path}")
    except OSError as exc:
        print(f"cannot write stop file {stop_path}: {exc}", file=sys.stderr)
        return 1
    if os.path.exists(pid_path):
        pid = read_pid(pid_path)
        if pid:
            for _ in range(30):
                if not process_alive(pid):
                    print(f"holder pid {pid} exited")
                    break
                time.sleep(0.1)
            else:
                print(f"holder pid {pid} did not exit from the stop file; sending SIGTERM")
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    pass
        elif pid is None:
            print(f"pid file {pid_path} is unreadable")
    else:
        print(f"no pid file at {pid_path}; the holder may already be gone")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--park", type=float, default=0.0,
                        help="angle to hold; 0 is the mid pulse (1500 us)")
    parser.add_argument("--minutes", type=float, default=0.0,
                        help="how long to hold before releasing (0 = until --stop)")
    parser.add_argument("--release", action="store_true",
                        help="leave the axis free instead of holding it (default: hold)")
    parser.add_argument("--foreground", action="store_true",
                        help="stay in the foreground (for a shell that owns the session)")
    parser.add_argument("--stop", action="store_true",
                        help="stop a detached holder started earlier and exit")
    parser.add_argument("--log", default=None,
                        help="log file path (default: a writable runtime directory)")
    parser.add_argument("--pid-file", default=None, help="pid file path")
    parser.add_argument("--stop-file", default=None,
                        help="the holder exits when this file appears")
    parser.add_argument("--chip", action="append", default=None,
                        help="PWM chip selector; repeatable")
    parser.add_argument("--channel", type=int, default=0, help="PWM channel index")
    parser.add_argument("--pwm-path", default=None,
                        help="explicit /sys/class/pwm/pwmchipN/pwmM channel directory")
    parser.add_argument("--period-us", type=int, default=MG90S_TRAVEL.period_us)
    parser.add_argument("--pulse-min-us", type=int, default=MG90S_TRAVEL.pulse_min_us)
    parser.add_argument("--pulse-max-us", type=int, default=MG90S_TRAVEL.pulse_max_us)
    parser.add_argument("--half-range-deg", type=float, default=MG90S_TRAVEL.half_range_deg)
    parser.add_argument("--settle-s", type=float, default=0.35)
    args = parser.parse_args()
    args.chip = args.chip or [ROCK5A_PWM7_M0_CHIP]

    directory = runtime_dir()
    args.log = args.log or str(directory / DEFAULT_LOG_NAME)
    args.pid_file = args.pid_file or str(directory / DEFAULT_PID_NAME)
    args.stop_file = args.stop_file or str(directory / DEFAULT_STOP_NAME)

    if args.stop:
        return stop_running(args.pid_file, args.stop_file)

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

    if not args.foreground and not os.environ.get("SERVO_HOLD_CHILD"):
        # Detach into its own session so the holder survives the SSH command that
        # started it, then report and return immediately. The child must let go of
        # the inherited stdout/stderr, or the parent command appears to hang
        # because the channel stays open until the holder exits.
        if os.path.exists(args.stop_file):
            os.unlink(args.stop_file)
        existing = read_pid(args.pid_file)
        if existing is not None and process_alive(existing):
            print(f"warning: holder pid {existing} already looks alive; two holders "
                  "would fight over the same channel. Stop it first with --stop.")
        pid = os.fork()
        if pid > 0:
            Path(args.pid_file).write_text(f"{pid}\n", encoding="utf-8")
            log(f"holder detached pid={pid} log={args.log}", args.log)
            print(f"detached holder pid {pid}")
            print(f"  log      : {args.log}")
            print(f"  stop with: python3 {Path(__file__).name} --stop")
            return 0
        os.setsid()
        os.environ["SERVO_HOLD_CHILD"] = "1"
        devnull = os.open(os.devnull, os.O_RDWR)
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        os.close(devnull)

    pid = os.getpid()
    duration = f"{args.minutes:g} min" if args.minutes > 0 else "until stopped"
    log(f"pid={pid} parking at {args.park:g} deg for {duration} "
        f"(release={args.release})", args.log)

    axis = build_axis(args, travel)
    stop = False
    requested = False

    def request_stop(_signum, _frame):
        nonlocal stop, requested
        stop = True
        requested = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    deadline = None if args.minutes <= 0 else time.monotonic() + args.minutes * 60.0
    exit_code = 0
    try:
        axis.start(home=False, sweep_start=True)
        pulse = axis.set_angle(args.park, settle=True)
        log(f"holding {args.park:g} deg ({pulse} us), channel enabled", args.log)
        while not stop:
            if os.path.exists(args.stop_file):
                log("stop file seen", args.log)
                requested = True
                break
            if deadline is not None and time.monotonic() >= deadline:
                log("hold duration elapsed", args.log)
                break
            time.sleep(0.5)
    except ServoError as exc:
        log(f"servo command failed: {exc}", args.log)
        exit_code = 1
    except PWMBackendError as exc:
        log(f"PWM backend failed: {exc}", args.log)
        exit_code = 1
    finally:
        # A hold that ends on request - or on a signal - releases the axis: when
        # someone says "stop", a latched axis that keeps drawing current is not
        # what they asked for. A hold that ended by itself (duration elapsed)
        # stays enabled, because that is the parked state the operator wanted.
        release = args.release or requested
        if release:
            axis.release()
            log("axis released (free)", args.log)
        else:
            # The kernel keeps emitting the pulse, so the axis stays parked.
            log(f"leaving channel enabled at {axis.angle_deg} deg "
                f"({axis.pulse_us} us)", args.log)
        if os.path.exists(args.pid_file):
            try:
                os.unlink(args.pid_file)
            except OSError:
                pass
        if not requested and os.path.exists(args.stop_file):
            try:
                os.unlink(args.stop_file)
            except OSError:
                pass
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
