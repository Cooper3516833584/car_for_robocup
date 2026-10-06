"""Single-axis PWM hobby servo component (Tower Pro MG90S on ROCK 5A ``PWM7_IR_M0``).

The component is deliberately one axis of one servo: it converts a logical angle
in **degrees** into a pulse width in **microseconds**, pushes that pulse through
the :class:`hal.pwm.PWMOutput` abstraction, and refuses any request outside the
calibrated travel. It never reads TOML and never knows a board's sysfs layout;
the pin, the chip selector and the calibration all arrive as constructor
arguments (in production, from ``build_servo`` in ``config/v2_factory.py``).

Angle convention
----------------
``0`` degrees is the **PWM mid point** (``1500`` us by default), not a pulse
endpoint. This is a deliberate project decision: powering up an uncalibrated
servo at a mid-scale pulse cannot drive the output arm into a mechanical stop.
Travel is therefore symmetric, ``-travel_half_range_deg`` .. ``+travel_half_range_deg``
with ``0`` in the middle, and increasing angle means increasing pulse width:

    0 deg   -> 1500 us
    +90 deg -> 2500 us
    -90 deg ->  500 us

The endpoint pulses (``pulse_min_us`` / ``pulse_max_us``) are **software travel
limits**, not the servo's mechanical range. A continuous-rotation
("360 degree") servo has no position feedback at all: it consumes the same
pulses as a speed command, so ``0`` degrees stops it and anything else spins it.
Set the endpoints from your own calibration before trusting an absolute angle.

Wiring and electrical notes (MG90S)
-----------------------------------
- Signal pin: ROCK 5A physical **Pin 28** = ``PWM7_IR_M0`` = ``GPIO0_D0``. It
  needs the ``rk3588-pwm7-m0`` device-tree overlay, which the board does not
  enable by default; see ``docs/SERVO_PWM7_M0.md``.
- 50 Hz refresh (``period_us=20000``) is the standard analog-servo frame.
- The MG90S is a small 4.8-6.0 V servo (idle current ~10 mA, stall current
  ~1.8-2.5 A). **Do not** power it from the header's 5 V pin: a stall would
  brown out the board. Feed the servo's red wire from a separate 5-6 V BEC with
  enough current headroom, and tie the BEC ground to a header GND pin, or the
  PWM signal will have no return reference.
- A 3.3 V PWM high level drives the MG90S signal input on the bench-verified
  rig, but it is at the edge of some servo input thresholds; a 3.3 V to 5 V
  level shifter is the safe wiring if the servo ever twitches or ignores
  commands.

Timing
------
``set_angle`` sends one pulse width and the PWM channel keeps repeating it at
the frame rate; no software loop is needed to hold a position. Commands are
rate limited to ``min_command_interval_s`` because a servo cannot follow a new
target faster than it physically moves, and every accepted move is followed by
``settle`` so callers that need the axis to have arrived can ask for it
explicitly rather than sleeping by guesswork. ``release()`` stops the pulse
train, which removes holding torque: the arm then drops under gravity, so only
release when the axis is either unloaded or meant to go slack.

``sweep_to`` / ``sweep_by`` move the axis at a requested angular **rate**
(degrees per second) by interpolating steps in software. The firmware has no
trajectory generator, so a rate is only as smooth as the step size: each step
sends a new pulse and then waits, which makes the motion a staircase whose
coarseness is ``step_deg``. A step is never allowed to be shorter than one PWM
frame (``period_us``), and the per-step delay is measured from the monotonic
clock, so a slow host or a long step does not pile up drift beyond the requested
rate this way. ``sweep_to`` deliberately bypasses ``min_command_interval_s``:
the sweep rate already paces the commands, and applying both would either stall
the sweep or distort the rate.

Example::

    from components.servo_axis import ServoAxis, MG90S_TRAVEL
    from hal.pwm import LinuxSysfsPWMOutput

    output = LinuxSysfsPWMOutput(chip_device_match="febd0030.pwm", channel=0)
    axis = ServoAxis(MG90S_TRAVEL, pwm=output, settle_s=0.35)
    axis.start()                                # homes to 0 degrees
    axis.set_angle(-90.0, settle=True)          # go to one end of travel
    axis.sweep_to(90.0, rate_deg_s=30.0)        # cross at 30 deg/s
    axis.sweep_to(-90.0, rate_deg_s=30.0)       # come back at 30 deg/s
    axis.release()
"""

from __future__ import annotations

from dataclasses import dataclass
import argparse
import math
import sys
import time
from typing import Callable, Protocol, runtime_checkable

from hal.pwm import LinuxSysfsPWMOutput, PWMOutput

__all__ = [
    "MG90S_PULSE_MIN_US",
    "MG90S_PULSE_MAX_US",
    "MG90S_TRAVEL",
    "ROCK5A_PWM7_M0_CHIP",
    "ROCK5A_PWM7_M0_PHYSICAL_PIN",
    "ROCK5A_PWM7_M0_OVERLAY",
    "FakeServoAxis",
    "ServoAxis",
    "ServoError",
    "ServoNotStartedError",
    "ServoRangeError",
    "ServoTravel",
    "DEFAULT_SWEEP_STEP_DEG",
    "angle_to_pulse_us",
    "pulse_to_angle_deg",
    "travel_from_endpoints",
]

# ROCK 5A physical Pin 28 / GPIO0_D0, enabled by the rk3588-pwm7-m0 overlay.
# The PWM7 controller lives at device-tree node febd0030.pwm; matching on the
# node name survives probe-order changes that renumber pwmchipN.
ROCK5A_PWM7_M0_CHIP = "febd0030.pwm"
ROCK5A_PWM7_M0_OVERLAY = "rk3588-pwm7-m0"
ROCK5A_PWM7_M0_PHYSICAL_PIN = 28

# MG90S bench defaults: 500..2500 us spans the vendor's +/-90 degree travel.
MG90S_PULSE_MIN_US = 500
MG90S_PULSE_MAX_US = 2500
MG90S_TRAVEL_HALF_RANGE_DEG = 90.0

# A rate sweep sends one pulse per step and waits out the rest of the step, so
# the step must be long enough to hold a full PWM frame without stalling.
DEFAULT_SWEEP_STEP_DEG = 1.0


class ServoError(RuntimeError):
    """The servo axis cannot carry out the request in its current state."""


class ServoNotStartedError(ServoError):
    """A command arrived before :meth:`ServoAxis.start` enabled the output."""


class ServoRangeError(ServoError, ValueError):
    """The requested angle or pulse lies outside the calibrated travel."""


@dataclass(frozen=True, slots=True)
class ServoTravel:
    """Calibrated mapping between logical angle and pulse width.

    ``pulse_min_us`` is the pulse at ``-half_range_deg`` and ``pulse_max_us`` the
    pulse at ``+half_range_deg``; ``0`` degrees always maps to their mid point.
    All pulses must fit inside one frame (``period_us``).
    """

    half_range_deg: float = MG90S_TRAVEL_HALF_RANGE_DEG
    pulse_min_us: int = MG90S_PULSE_MIN_US
    pulse_max_us: int = MG90S_PULSE_MAX_US
    period_us: int = 20_000

    def __post_init__(self) -> None:
        half_range = float(self.half_range_deg)
        if not math.isfinite(half_range) or half_range <= 0.0:
            raise ServoRangeError("half_range_deg must be a positive finite angle")
        for name in ("pulse_min_us", "pulse_max_us"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ServoRangeError(f"{name} must be an integer number of microseconds")
        if isinstance(self.period_us, bool) or not isinstance(self.period_us, int):
            raise ServoRangeError("period_us must be an integer number of microseconds")
        if self.period_us <= 0:
            raise ServoRangeError("period_us must be positive")
        if not 0 <= self.pulse_min_us < self.pulse_max_us < self.period_us:
            # A pulse equal to the frame leaves no low period to separate
            # frames, so the whole frame period is the hard upper limit.
            raise ServoRangeError(
                "pulses must satisfy 0 <= pulse_min_us < pulse_max_us < period_us"
            )
        object.__setattr__(self, "half_range_deg", half_range)

    @property
    def pulse_mid_us(self) -> float:
        return (self.pulse_min_us + self.pulse_max_us) / 2.0

    @property
    def pulse_span_us(self) -> int:
        return self.pulse_max_us - self.pulse_min_us

    @property
    def min_angle_deg(self) -> float:
        return -self.half_range_deg

    @property
    def max_angle_deg(self) -> float:
        return self.half_range_deg

    @property
    def us_per_deg(self) -> float:
        return self.pulse_span_us / (2.0 * self.half_range_deg)

    def contains_angle(self, angle_deg: float) -> bool:
        value = float(angle_deg)
        return math.isfinite(value) and -self.half_range_deg <= value <= self.half_range_deg


# Bench default for the fitted MG90S: +/-90 degrees over 500..2500 us at 50 Hz.
MG90S_TRAVEL = ServoTravel(
    half_range_deg=MG90S_TRAVEL_HALF_RANGE_DEG,
    pulse_min_us=MG90S_PULSE_MIN_US,
    pulse_max_us=MG90S_PULSE_MAX_US,
)


def travel_from_endpoints(
    *,
    pulse_min_us: int,
    pulse_max_us: int,
    min_angle_deg: float,
    max_angle_deg: float,
    period_us: int = 20_000,
) -> ServoTravel:
    """Build a :class:`ServoTravel` from an asymmetric pulse/angle pair.

    ``min_angle_deg``/``max_angle_deg`` are the angles at the two pulses; they
    need not be symmetric and need not be centred on zero. The resulting travel
    is re-centred on zero so that the component's public contract (``0`` degrees
    is the mid pulse) holds for every calibration, and the endpoint angles are
    reported through ``min_angle_deg``/``max_angle_deg``.
    """

    low_angle, high_angle = float(min_angle_deg), float(max_angle_deg)
    if not math.isfinite(low_angle) or not math.isfinite(high_angle):
        raise ServoRangeError("endpoint angles must be finite")
    if high_angle <= low_angle:
        raise ServoRangeError("max_angle_deg must be greater than min_angle_deg")
    return ServoTravel(
        half_range_deg=(high_angle - low_angle) / 2.0,
        pulse_min_us=pulse_min_us,
        pulse_max_us=pulse_max_us,
        period_us=period_us,
    )


def angle_to_pulse_us(travel: ServoTravel, angle_deg: float) -> int:
    """Return the pulse width for ``angle_deg`` without any state or hardware."""

    value = float(angle_deg)
    if not travel.contains_angle(value):
        raise ServoRangeError(
            f"angle {value!r} deg is outside the calibrated travel "
            f"[{travel.min_angle_deg:g}, {travel.max_angle_deg:g}] deg"
        )
    return int(round(travel.pulse_mid_us + value * travel.us_per_deg))


def pulse_to_angle_deg(travel: ServoTravel, pulse_us: float) -> float:
    """Inverse of :func:`angle_to_pulse_us`; useful for calibration read-back."""

    pulse = float(pulse_us)
    if not math.isfinite(pulse):
        raise ServoRangeError("pulse_us must be finite")
    if not travel.pulse_min_us <= pulse <= travel.pulse_max_us:
        raise ServoRangeError(
            f"pulse {pulse!r} us is outside the calibrated travel "
            f"[{travel.pulse_min_us}, {travel.pulse_max_us}] us"
        )
    return (pulse - travel.pulse_mid_us) / travel.us_per_deg


@runtime_checkable
class _ServoOutput(Protocol):
    """The slice of :class:`hal.pwm.PWMOutput` this component actually uses."""

    @property
    def is_running(self) -> bool: ...

    def start(self) -> "_ServoOutput": ...

    def set_pulse_us(self, pulse_us: int) -> None: ...

    def disable(self) -> None: ...

    def close(self) -> None: ...


class ServoAxis:
    """One calibrated PWM servo axis; construction never touches hardware.

    ``start`` is what enables the PWM channel, mirroring the rest of this
    repository: building a component must not move a physical actuator.
    """

    def __init__(
        self,
        travel: ServoTravel | None = None,
        *,
        pwm: _ServoOutput | None = None,
        settle_s: float = 0.35,
        min_command_interval_s: float = 0.02,
        home_angle_deg: float = 0.0,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.travel = ServoTravel() if travel is None else travel
        if pwm is None:
            raise ValueError("a PWM output is required; build one in config/v2_factory.py")
        settle = float(settle_s)
        if not math.isfinite(settle) or settle < 0.0:
            raise ValueError("settle_s must be a non-negative finite duration")
        interval = float(min_command_interval_s)
        if not math.isfinite(interval) or interval < 0.0:
            raise ValueError("min_command_interval_s must be a non-negative finite duration")
        if not self.travel.contains_angle(home_angle_deg):
            raise ServoRangeError(
                f"home_angle_deg {home_angle_deg!r} is outside the calibrated travel"
            )
        self._pwm = pwm
        self._settle_s = settle
        self._min_interval_s = interval
        self._home_angle_deg = float(home_angle_deg)
        self._clock = clock
        self._sleep = sleep
        self._angle_deg: float | None = None
        self._pulse_us: int | None = None
        self._last_command_at: float | None = None
        self._start_angle_deg: float | None = None
        self._started = False

    # -- state -----------------------------------------------------------------

    @property
    def is_running(self) -> bool:
        """True while the pulse train is enabled."""

        return self._started and bool(self._pwm.is_running)

    @property
    def angle_deg(self) -> float | None:
        """Last commanded angle, or ``None`` before the first command."""

        return self._angle_deg

    @property
    def pulse_us(self) -> int | None:
        """Last commanded pulse width, or ``None`` before the first command."""

        return self._pulse_us

    @property
    def home_angle_deg(self) -> float:
        return self._home_angle_deg

    @property
    def start_angle_deg(self) -> float | None:
        """Angle the axis was at when :meth:`start` enabled it, if known.

        ``None`` when homing was suppressed: enabling the channel says nothing
        about where the arm physically is, so the component refuses to invent a
        starting position for a later :meth:`return_to_start`.
        """

        return self._start_angle_deg

    @property
    def settle_s(self) -> float:
        return self._settle_s

    @property
    def min_command_interval_s(self) -> float:
        """Shortest interval :meth:`set_angle` accepts between two commands.

        Sweeps pace themselves and ignore it; callers that mix a direct
        :meth:`set_angle` with a :meth:`sweep_to` need it to space the two.
        """

        return self._min_interval_s

    def commanded_angle_deg(self, default: float | None = None) -> float | None:
        """Last commanded angle, or ``start_angle_deg``/``default`` if never commanded."""

        if self._angle_deg is not None:
            return self._angle_deg
        if self._start_angle_deg is not None:
            return self._start_angle_deg
        return default

    def pulse_us_for(self, angle_deg: float) -> int:
        """Public, side-effect-free angle to pulse conversion."""

        return angle_to_pulse_us(self.travel, angle_deg)

    def angle_deg_for(self, pulse_us: float) -> float:
        """Public, side-effect-free pulse to angle conversion."""

        return pulse_to_angle_deg(self.travel, pulse_us)

    # -- lifecycle -------------------------------------------------------------

    def start(self, *, home: bool = True, settle: bool = False,
              sweep_start: bool = False) -> "ServoAxis":
        """Enable the pulse train and, by default, drive the axis to ``0`` degrees.

        Homing on start makes power-up deterministic: after a reboot the axis
        returns to the known mid pulse instead of wherever it was left. Pass
        ``home=False`` to enable the channel and leave the duty cycle untouched;
        the next :meth:`set_angle` is then always written, even if it matches a
        pulse this axis commanded in an earlier session.

        Either way the angle the axis is known to start from is recorded, so a
        later :meth:`return_to_start` has a defined target. With ``home=True``
        that is the home angle. ``home=False, sweep_start=True`` records the home
        angle as the origin *without* writing a pulse, so a caller that commands
        its first move itself (typically the operator CLI) is not blocked by the
        command rate limiter, and the first command it sends establishes the
        origin physically.
        """

        if self._started:
            return self
        self._pwm.start()
        self._started = True
        if home:
            # A channel restored by the driver can still hold the previous boot's
            # duty cycle, so always push a known pulse when homing.
            self.apply_angle(self._home_angle_deg, settle=settle, force=True)
            self._start_angle_deg = self._home_angle_deg
        elif sweep_start:
            self._start_angle_deg = self._home_angle_deg
            self._pulse_us = None
            self._last_command_at = None
        else:
            # Nothing was written, so forget any stale command bookkeeping and
            # do not claim to know the physical starting angle.
            self._pulse_us = None
            self._last_command_at = None
            self._start_angle_deg = None
        return self

    def apply_angle(self, angle_deg: float, *, settle: bool = False, force: bool = False) -> int:
        """Command ``angle_deg`` now, keeping the raw/segment escape hatch.

        Unlike :meth:`set_angle` this skips the command rate limit; it is what
        :meth:`start` and calibration sweeps use.
        """

        if not self._started:
            raise ServoNotStartedError("call start() before commanding the servo")
        pulse = self.pulse_us_for(angle_deg)
        if not force and pulse == self._pulse_us:
            self._angle_deg = float(angle_deg)
            if settle:
                self._sleep(self._settle_s)
            return pulse
        self._pwm.set_pulse_us(pulse)
        now = self._clock()
        self._angle_deg = float(angle_deg)
        self._pulse_us = pulse
        self._last_command_at = now
        if settle:
            self._sleep(self._settle_s)
        return pulse

    def set_angle(self, angle_deg: float, *, settle: bool = False) -> int:
        """Rotate the axis to ``angle_deg`` (degrees), returning the pulse width.

        The request fails closed with :class:`ServoRangeError` outside the
        calibrated travel, and is rate limited by ``min_command_interval_s``:
        commands issued faster than that raise :class:`ServoError` instead of
        silently queueing motion the servo cannot follow.

        Pass ``settle=True`` when the caller needs the axis to have physically
        arrived (for example before a vision capture) rather than just commanded.
        """

        if not self._started:
            raise ServoNotStartedError("call start() before commanding the servo")
        # Validate the angle even when the pulse equals the current one.
        self.pulse_us_for(angle_deg)
        if self._last_command_at is not None and self._min_interval_s > 0.0:
            elapsed = self._clock() - self._last_command_at
            if elapsed < self._min_interval_s:
                raise ServoError(
                    f"servo command rate limited: {elapsed * 1000.0:.1f} ms since the last "
                    f"command, minimum is {self._min_interval_s * 1000.0:.1f} ms"
                )
        return self.apply_angle(angle_deg, settle=settle)

    def nudge(self, delta_deg: float, *, settle: bool = False) -> int:
        """Rotate by ``delta_deg`` relative to the last commanded angle."""

        if self._angle_deg is None:
            raise ServoNotStartedError("no commanded angle yet; call start() first")
        return self.set_angle(self._angle_deg + float(delta_deg), settle=settle)

    # -- rate-controlled motion -------------------------------------------------

    @staticmethod
    def _sweep_step_s(rate_deg_s: float, step_deg: float, period_us: int) -> float:
        """Delay between two sweep steps, never shorter than one PWM frame."""

        rate = float(rate_deg_s)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ServoError("rate_deg_s must be a positive finite speed in degrees/second")
        step = float(step_deg)
        if not math.isfinite(step) or step <= 0.0:
            raise ServoError("step_deg must be a positive finite angle")
        return max(step / rate, period_us / 1_000_000.0)

    @staticmethod
    def _sweep_plan(start_deg: float, end_deg: float, step_deg: float) -> list[float]:
        """Intermediate plus final angles from ``start_deg`` to ``end_deg``.

        The final target is always included exactly, so a sweep ends on the
        requested angle rather than on the last step boundary.
        """

        span = end_deg - start_deg
        distance = abs(span)
        if distance == 0.0:
            return []
        count = int(math.ceil(distance / step_deg))
        direction = 1.0 if span > 0.0 else -1.0
        plan = [start_deg + direction * distance * (index / count) for index in range(1, count + 1)]
        plan[-1] = end_deg
        return plan

    def sweep_to(self, angle_deg: float, *, rate_deg_s: float,
                 step_deg: float = DEFAULT_SWEEP_STEP_DEG, settle: bool = False) -> int:
        """Move to ``angle_deg`` at ``rate_deg_s`` degrees per second.

        The motion is interpolated in software as ``step_deg``-sized commands,
        each followed by the delay its share of the rate implies (never shorter
        than one PWM frame). ``min_command_interval_s`` is deliberately bypassed
        here: the rate already paces the commands. Returns the final pulse width.

        Raises :class:`ServoRangeError` if either the current or the target angle
        is outside the calibrated travel, so a sweep never clamps silently.
        """

        if not self._started:
            raise ServoNotStartedError("call start() before sweeping the servo")
        target = float(angle_deg)
        if not self.travel.contains_angle(target):
            raise ServoRangeError(
                f"target angle {target!r} deg is outside the calibrated travel "
                f"[{self.travel.min_angle_deg:g}, {self.travel.max_angle_deg:g}] deg"
            )
        start = self.commanded_angle_deg(default=self._home_angle_deg)
        if start is None or not self.travel.contains_angle(start):
            raise ServoNotStartedError(
                "cannot sweep without a known starting angle; call start() first"
            )
        step_s = self._sweep_step_s(rate_deg_s, step_deg, self.travel.period_us)
        plan = self._sweep_plan(float(start), target, float(step_deg))
        # Pace against the monotonic clock, not a running total of the requested
        # delays: a host that oversleeps one step must not push the rest of the
        # sweep into the future, which would make the axis report a command time
        # ahead of reality and block the next command.
        deadline = self._clock()
        for angle in plan:
            self.apply_angle(angle, force=True)
            deadline += step_s
            remaining = deadline - self._clock()
            if remaining > 0.0:
                self._sleep(remaining)
        if settle:
            self._sleep(self._settle_s)
        self._last_command_at = self._clock()
        return self._pulse_us if self._pulse_us is not None else self.pulse_us_for(target)

    def sweep_by(self, delta_deg: float, *, rate_deg_s: float,
                 step_deg: float = DEFAULT_SWEEP_STEP_DEG, settle: bool = False) -> int:
        """Move by ``delta_deg`` at a rate, relative to the last commanded angle."""

        start = self.commanded_angle_deg()
        if start is None:
            raise ServoNotStartedError("no commanded angle yet; call start() first")
        return self.sweep_to(float(start) + float(delta_deg), rate_deg_s=rate_deg_s,
                             step_deg=step_deg, settle=settle)

    def return_to_start(self, *, rate_deg_s: float,
                        step_deg: float = DEFAULT_SWEEP_STEP_DEG,
                        settle: bool = False) -> int:
        """Sweep back to the angle this axis started from.

        The reference is the angle recorded by :meth:`start`, not "wherever the
        axis happens to be now", so a scan that lost track mid-way still returns
        to the position the operator saw at power-up.
        """

        if self._start_angle_deg is None:
            raise ServoNotStartedError(
                "no recorded start angle: enable the axis with start() (home=True or "
                "sweep_start=True) so a return target exists"
            )
        return self.sweep_to(self._start_angle_deg, rate_deg_s=rate_deg_s,
                             step_deg=step_deg, settle=settle)

    def release(self) -> None:
        """Stop the pulse train, removing holding torque (the arm may drop)."""

        if not self._started:
            return
        self._pwm.disable()
        self._started = False

    def hold(self) -> None:
        """Keep the pulse train running so the axis resists its mechanical load.

        Use this on an axis with a return force (a cable bundle, gravity): the
        servo only holds position while it is enabled, so releasing it at the end
        of a move is what lets the arm spring back. The cost is idle current,
        which for a small servo on a light payload is far below its moving draw.
        """

        if not self._started:
            raise ServoNotStartedError(
                "cannot hold: the pulse train is off; call start() first"
            )

    def close(self, *, hold: bool = True) -> None:
        """Park the axis under power (default) or release it, then close the channel.

        **Holding is the default** because a hobby servo has no holding torque
        once its pulse train stops: an axis carrying a load (a camera platform
        pulled by its own cable, anything off-centre) loses its commanded angle
        the moment it is released. Holding costs only the servo's idle current,
        tens of milliamps for a small servo on a light payload, which is well
        below what the same axis draws while moving.

        Pass ``hold=False`` to release the axis and let it go free, which is the
        right choice for an axis that is unloaded, or when the mechanism is meant
        to relax between tasks.
        """

        try:
            if hold:
                self.hold()
            else:
                self.release()
        finally:
            self._pwm.close()

    def __enter__(self) -> "ServoAxis":
        return self.start()

    def __exit__(self, *exc_info: object) -> None:
        # Deliberately holds: leaving a loaded axis free on scope exit is what
        # silently loses the angle the caller just set.
        self.close(hold=True)


class FakeServoAxis(ServoAxis):
    """In-memory axis for dry-run, replay and unit tests.

    It records every pulse it would have written and never opens a device, so it
    can stand in for the real component wherever a profile enables the servo
    without hardware attached.
    """

    def __init__(self, travel: ServoTravel | None = None, **kwargs: object) -> None:
        if "pwm" in kwargs:
            raise ValueError("FakeServoAxis supplies its own in-memory PWM output")
        self.commands: list[int] = []
        self.disabled_count = 0
        self.closed = False
        super().__init__(travel, pwm=_FakePWM(self), **kwargs)  # type: ignore[arg-type]

    def write_pulse(self, pulse_us: int) -> None:
        self.commands.append(pulse_us)

    def _disable(self) -> None:
        self.disabled_count += 1

    def close(self, *, hold: bool = True) -> None:
        try:
            super().close(hold=hold)
        finally:
            self.closed = True


class _FakePWM:
    """PWM stand-in wiring :class:`FakeServoAxis` bookkeeping into the axis."""

    def __init__(self, axis: FakeServoAxis) -> None:
        self._axis = axis
        self._running = False

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> "_FakePWM":
        self._running = True
        return self

    def set_pulse_us(self, pulse_us: int) -> None:
        self._axis.write_pulse(int(pulse_us))

    def disable(self) -> None:
        self._axis._disable()
        self._running = False

    def close(self) -> None:
        self._running = False


def _build_pwm(args: argparse.Namespace) -> PWMOutput:
    return LinuxSysfsPWMOutput(
        chip_device_match=tuple(args.chip),
        channel=args.channel,
        period_ns=int(round(args.travel.period_us * 1000)),
        pwm_chip=args.pwm_chip,
        pwm_path=args.pwm_path,
    )


def _travel_from_args(args: argparse.Namespace) -> ServoTravel:
    if args.min_angle_deg is not None or args.max_angle_deg is not None:
        if args.min_angle_deg is None or args.max_angle_deg is None:
            raise ServoRangeError("--min-angle-deg and --max-angle-deg must be given together")
        return travel_from_endpoints(
            pulse_min_us=args.pulse_min_us,
            pulse_max_us=args.pulse_max_us,
            min_angle_deg=args.min_angle_deg,
            max_angle_deg=args.max_angle_deg,
            period_us=args.period_us,
        )
    return ServoTravel(
        half_range_deg=args.half_range_deg,
        pulse_min_us=args.pulse_min_us,
        pulse_max_us=args.pulse_max_us,
        period_us=args.period_us,
    )


def main(argv: list[str] | None = None) -> int:
    """Supervised single-axis servo test; run with ``sudo`` for sysfs access."""

    parser = argparse.ArgumentParser(
        description="Drive one PWM hobby servo axis and report the pulse written.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--angle", type=float, default=0.0,
                        help="target angle in degrees where 0 is the mid pulse")
    parser.add_argument("--sweep", type=float, nargs="+", metavar="DEG",
                        help="visit several angles in order instead of one target")
    parser.add_argument("--hold-s", type=float, default=0.0,
                        help="seconds to hold the final position before releasing")
    parser.add_argument("--no-settle", action="store_true",
                        help="do not wait for the axis to reach each target")
    parser.add_argument("--keep-enabled", action="store_true",
                        help="leave the pulse train running instead of releasing on exit")
    parser.add_argument("--chip", action="append", default=None,
                        help="PWM chip selector; repeatable for fallbacks")
    parser.add_argument("--pwm-chip", default=None,
                        help="explicit /sys/class/pwm/pwmchipN path, skipping chip lookup")
    parser.add_argument("--pwm-path", default=None,
                        help="explicit /sys/class/pwm/pwmchipN/pwmM channel directory")
    parser.add_argument("--channel", type=int, default=0, help="PWM channel index")
    parser.add_argument("--period-us", type=int, default=20_000, help="PWM frame period")
    parser.add_argument("--pulse-min-us", type=int, default=MG90S_PULSE_MIN_US)
    parser.add_argument("--pulse-max-us", type=int, default=MG90S_PULSE_MAX_US)
    parser.add_argument("--half-range-deg", type=float, default=MG90S_TRAVEL_HALF_RANGE_DEG,
                        help="symmetric travel half range centred on 0 degrees")
    parser.add_argument("--min-angle-deg", type=float, default=None,
                        help="lowest angle; use with --max-angle-deg for a shifted travel")
    parser.add_argument("--max-angle-deg", type=float, default=None,
                        help="highest angle; use with --min-angle-deg for a shifted travel")
    parser.add_argument("--settle-s", type=float, default=0.35,
                        help="seconds to allow after each move")
    parser.add_argument("--dry-run", action="store_true",
                        help="use the in-memory axis; never opens a PWM channel")
    args = parser.parse_args(argv)
    args.chip = args.chip or [ROCK5A_PWM7_M0_CHIP]

    try:
        travel = _travel_from_args(args)
    except ServoRangeError as exc:
        print(f"invalid servo calibration: {exc}", file=sys.stderr)
        return 2

    axis: ServoAxis
    if args.dry_run:
        axis = FakeServoAxis(travel, settle_s=args.settle_s)
    else:
        try:
            axis = ServoAxis(travel, pwm=_build_pwm(args), settle_s=args.settle_s)
        except (ValueError, OSError) as exc:
            print(f"cannot configure PWM output: {exc}", file=sys.stderr)
            return 2

    targets = args.sweep if args.sweep else [args.angle]
    settle = not args.no_settle
    try:
        axis.start(settle=settle)
        print(f"home: {axis.angle_deg:g} deg -> {axis.pulse_us} us")
        for target in targets:
            pulse = axis.set_angle(target, settle=settle)
            source = "recorded" if args.dry_run else "written"
            print(f"angle {target:g} deg -> pulse {pulse} us ({source})")
        if args.hold_s > 0.0:
            time.sleep(args.hold_s)
    except ServoError as exc:
        print(f"servo command failed: {exc}", file=sys.stderr)
        return 1
    except OSError as exc:
        print(f"PWM output error: {exc}", file=sys.stderr)
        return 1
    finally:
        if args.keep_enabled:
            print(f"left enabled at {axis.angle_deg:g} deg ({axis.pulse_us} us)")
        else:
            axis.release()
            print("pulse train released")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
