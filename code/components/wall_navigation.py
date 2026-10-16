"""Wall-driven lateral correction, distance stops and in-place turns.

Input is a :class:`~components.wall_observer.WallSnapshot` plus the current
action; output is the existing :class:`core.types.Twist2D`. This module never
writes serial frames and never creates a second motor sender: the one
``DifferentialDrive`` still owns acceleration limiting, wheel-speed limiting,
the command watchdog and the C10B backend's reverse compensation.

Sign convention (physical chassis, fixed here so the firmware encoding is
handled exactly once, in the backend): ``v > 0`` forward,
``omega > 0`` counter-clockwise / left.

The lateral controller reuses the existing Pure Pursuit curvature idea, but the
line is the wall line measured in the current body frame -- not a stored map
line -- so there is no global line, no ``Pose2D`` and no localization gate.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import logging
import math
import time
from typing import Callable

from components.wall_observer import (
    MIN_SPAN_M,
    WallLine,
    WallSnapshot,
    WallTracker,
    wrap_to_pi,
)
from core.types import Twist2D

LOG = logging.getLogger("wall-navigation")

# Software starting values from the implementation package, not measured field
# numbers. Keep them here instead of scattering command-line switches.
CORNER_TOLERANCE_M = 0.02
DROP_TOLERANCE_M = 0.01
TURN_TOLERANCE_RAD = math.radians(2.0)
TURN_GAIN = 1.0
TURN_MAX_OMEGA_RAD_S = 0.30
# Brake ahead of the measured target to allow for scan/actuator response.
# This prediction never replaces the measured-angle completion gate.
TURN_RESPONSE_S = 0.25
TURN_SETTLED_RATE_RAD_S = math.radians(5.0)
TURN_STABLE_FRAMES = 2
TURN_LOST_TIMEOUT_S = 5.0
WALL_LOST_TIMEOUT_S = 5.0
HOLD_TICK_S = 0.02
# A long wall approximately perpendicular to the right wall supplies the stop
# distance even when an occlusion prevents observing their corner.
TERMINAL_ORTHOGONAL_TOLERANCE_RAD = math.radians(20.0)
TERMINAL_TRACK_ANGLE_RAD = math.radians(30.0)
# A terminal wall must also be a real wall, not a stub: below the observer's own
# minimum supported length it cannot be the end of the field.
MIN_TERMINAL_SPAN_M = MIN_SPAN_M
# How long a terminal wall that was already verified may stay out of view before
# the leg gives up, rather than driving on with a distance that cannot update.
TERMINAL_LOST_TIMEOUT_S = 5.0
TERMINAL_DISTINCT_LINES_M = 0.10
DROP_TERMINAL_RANGE_M = 0.06
DROP_FINE_SPEED_M_S = 0.05
DROP_TERMINAL_SPEED_M_S = 0.012
# Heading hold while translating squarely toward or away from one wall. The band
# is the allowed residual tilt; outside it the correction is proportional with a
# gain that reaches the chassis angular limit at the edge of a 5 deg-plus band.
DROP_HEADING_BAND_RAD = math.radians(5.0)
# Success is judged slightly outside the dead band. If the two were equal, the
# controller would stop correcting exactly at the edge and any residual just
# beyond it would never be corrected again, so the move could sit one
# thousandth of a degree from success forever. The extra half degree is the
# settle margin, not a wider allowance: correction still starts at 5.0.
DROP_HEADING_TOLERANCE_RAD = math.radians(5.5)
# Extra room, in radians, added to a custom dead band so the tolerance always
# sits strictly outside it.
HEADING_SETTLE_MARGIN_RAD = math.radians(0.5)
DROP_HEADING_GAIN = 1.5
DROP_HEADING_SPEED_FLOOR = 0.4
DROP_NORMAL_ANGLE_TOLERANCE_RAD = math.radians(35.0)
TERMINAL_CREEP_SPEED_M_S = 0.008

# Cruise profile derived from the field config, matching the retired patrol
# numbers so the physical speed does not change.
CRUISE_SPEED_FACTOR = 0.15
VISION_SPEED_FACTOR = 0.08
CRUISE_MIN_FACTOR = 0.045

# Pivot bearings: where a tracked wall sits in the body frame.
REAR_BEARING_RAD = math.pi
RIGHT_BEARING_RAD = -math.pi / 2.0
LEFT_BEARING_RAD = math.pi / 2.0
FRONT_BEARING_RAD = 0.0


def clamp(value: float, low: float, high: float) -> float:
    if low > high:
        raise ValueError("clamp bounds are inverted")
    return low if value < low else high if value > high else value


@dataclass(frozen=True, slots=True)
class WallActionLimits:
    """Speed/limit profile read from the live config; never overriding it."""

    follow_speed_m_s: float
    vision_speed_m_s: float
    minimum_speed_m_s: float
    lookahead_m: float
    track_width_m: float
    wheel_speed_m_s: float
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    max_linear_accel_m_s2: float
    max_angular_accel_rad_s2: float

    def __post_init__(self):
        for name, value in (("follow_speed_m_s", self.follow_speed_m_s),
                            ("vision_speed_m_s", self.vision_speed_m_s),
                            ("minimum_speed_m_s", self.minimum_speed_m_s),
                            ("lookahead_m", self.lookahead_m),
                            ("track_width_m", self.track_width_m),
                            ("wheel_speed_m_s", self.wheel_speed_m_s),
                            ("max_linear_speed_m_s", self.max_linear_speed_m_s),
                            ("max_angular_speed_rad_s", self.max_angular_speed_rad_s),
                            ("max_linear_accel_m_s2", self.max_linear_accel_m_s2),
                            ("max_angular_accel_rad_s2", self.max_angular_accel_rad_s2)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")

    @classmethod
    def from_config(cls, config, *, navigation=None, drive=None):
        navigation = config.navigation if navigation is None else navigation
        drive = config.drive if drive is None else drive
        scale = float(navigation.translation_speed_scale)
        follow = min(CRUISE_SPEED_FACTOR * scale, float(drive.max_linear_speed_m_s))
        return cls(
            follow_speed_m_s=follow,
            vision_speed_m_s=min(follow, VISION_SPEED_FACTOR * scale,
                                 float(drive.max_linear_speed_m_s)),
            minimum_speed_m_s=min(CRUISE_MIN_FACTOR * scale, follow),
            lookahead_m=float(navigation.lookahead_m),
            track_width_m=float(config.geometry.drive_track_width_m),
            wheel_speed_m_s=float(drive.max_wheel_speed_m_s),
            max_linear_speed_m_s=float(drive.max_linear_speed_m_s),
            max_angular_speed_rad_s=float(drive.max_angular_speed_rad_s),
            max_linear_accel_m_s2=float(drive.max_linear_accel_m_s2),
            max_angular_accel_rad_s2=float(drive.max_angular_accel_rad_s2),
        )


def is_right_wall(line: WallLine) -> bool:
    """True when the line is approximately alongside the chassis on the right."""
    return line.normal_xy[1] <= -0.5


def is_rear_wall(line: WallLine) -> bool:
    """True when the normal points behind the chassis (the wall is astern)."""
    return line.normal_xy[0] <= -0.5


def is_front_wall(line: WallLine) -> bool:
    """True when the normal points ahead of the chassis."""
    return line.normal_xy[0] >= 0.5


def _same_line(first: WallLine, second: WallLine) -> bool:
    """Whether two observations are plausibly the same physical wall line."""
    normal_dot = abs(first.normal_xy[0] * second.normal_xy[0]
                     + first.normal_xy[1] * second.normal_xy[1])
    if normal_dot < math.cos(math.radians(10.0)):
        return False
    return abs(first.rho_base_m - second.rho_base_m) <= TERMINAL_DISTINCT_LINES_M


def wall_pure_pursuit(right: WallLine, target_distance_m: float, signed_speed_m_s: float,
                      lookahead_m: float) -> tuple[float, float]:
    """Directed-line Pure Pursuit on the instantaneous body-frame wall line.

    ``e = rho - d`` is the signed distance error; the carrot sits on the wall
    tangent ahead of (or behind, when reversing) the foot of the perpendicular.
    A positive error means the target line lies further out along the wall
    normal, which steers the chassis away from the wall while driving forward --
    and the opposite way when reversing. That sign change is not optional.
    """
    if not math.isfinite(target_distance_m) or target_distance_m <= 0:
        raise ValueError("target_distance_m must be finite and positive")
    if not math.isfinite(signed_speed_m_s):
        raise ValueError("signed_speed_m_s must be finite")
    if not is_right_wall(right):
        raise ValueError("reference wall is not approximately alongside the chassis on the right")
    nx, ny = right.normal_xy
    tangent_x, tangent_y = -ny, nx
    if tangent_x < 0.0:
        tangent_x, tangent_y = -tangent_x, -tangent_y
    error = right.rho_base_m - target_distance_m
    carrot_x = error * nx + math.copysign(lookahead_m, signed_speed_m_s) * tangent_x
    carrot_y = error * ny + math.copysign(lookahead_m, signed_speed_m_s) * tangent_y
    denominator = carrot_x * carrot_x + carrot_y * carrot_y
    if denominator < 1e-12:
        return signed_speed_m_s, 0.0
    curvature = 2.0 * carrot_y / denominator
    return signed_speed_m_s, signed_speed_m_s * curvature


def braking_cap(distance_m: float, target_m: float, speed_cap_m_s: float, *,
                deceleration_m_s2: float, tolerance_m: float = CORNER_TOLERANCE_M) -> float:
    """Brake-to-stop speed cap for an approaching terminal wall.

    Returns ``0.0`` once inside tolerance; otherwise the smaller of the incoming
    cap and ``sqrt(2 * a * remaining)``. It is always applied with a final
    ``min()`` so the cruise speed floor can never raise it again, and it never
    asks the chassis to reverse to recover a crossed threshold.
    """
    if not math.isfinite(deceleration_m_s2) or deceleration_m_s2 <= 0:
        raise ValueError("deceleration_m_s2 must be finite and positive")
    if not math.isfinite(distance_m) or not math.isfinite(target_m):
        raise ValueError("finite distances required")
    remaining = distance_m - target_m
    if remaining <= tolerance_m:
        return 0.0
    return min(speed_cap_m_s, math.sqrt(2.0 * deceleration_m_s2 * remaining))


def follow_command(right: WallLine, target_distance_m: float, direction: int, *,
                   limits: WallActionLimits, speed_cap_m_s: float | None = None,
                   terminal_measured_m: float | None = None,
                   terminal_target_m: float | None = None,
                   terminal_tolerance_m: float = CORNER_TOLERANCE_M,
                   terminal_creep_m_s: float = TERMINAL_CREEP_SPEED_M_S,
                   ) -> tuple[Twist2D, dict]:
    """One patrol tick: lateral Pure Pursuit plus terminal braking.

    ``direction`` is ``+1`` forward or ``-1`` reversing with the nose unchanged.
    The terminal wall is a *different* wall from the right wall: closing a
    corner looks at the front wall, reversing along the top edge looks at the
    rear wall. Only its measured perpendicular distance and target are needed
    here, so the caller may keep its own reference identity for that wall.

    Braking is a ``min()`` applied last, so the cruise speed floor can never
    lift it. Inside the stop tolerance the command is exactly zero and the
    action is asked to finish.
    """
    if direction not in (-1, 1) or isinstance(direction, bool):
        raise ValueError("direction must be +1 or -1")
    # A terminal target may be supplied with no measurement: the terminal wall
    # can vanish for a scan or two, and the caller then keeps its last measured
    # remaining distance as a conservative cap and creeps rather than stopping
    # dead outside tolerance. Refusing that combination would turn a momentary
    # dropout into a mission abort.
    if terminal_target_m is None and terminal_measured_m is not None:
        raise ValueError("terminal_measured_m requires terminal_target_m")
    planned_speed = limits.follow_speed_m_s if speed_cap_m_s is None else \
        min(limits.follow_speed_m_s, speed_cap_m_s)
    terminal_fields: dict = {}
    at_terminal = False
    if terminal_target_m is not None:
        terminal_fields = {"terminal_target_m": terminal_target_m}
        if terminal_measured_m is not None:
            terminal_fields["terminal_measured_m"] = terminal_measured_m
            terminal_fields["terminal_remaining_m"] = terminal_measured_m - terminal_target_m
            remaining = terminal_measured_m - terminal_target_m
            if remaining <= terminal_tolerance_m:
                at_terminal = True
            else:
                brakable = braking_cap(terminal_measured_m, terminal_target_m, planned_speed,
                                       deceleration_m_s2=limits.max_linear_accel_m_s2,
                                       tolerance_m=terminal_tolerance_m)
                # Never creep faster than the caller's cap, and never force an
                # abrupt stop outside tolerance just because the braking curve
                # rounds to zero.
                planned_speed = min(max(brakable, min(terminal_creep_m_s, planned_speed)),
                                    planned_speed)
        else:
            # Terminal wall momentarily out of view: bound the approach by the
            # creep speed until a fresh distance is measured again.
            terminal_fields["terminal_measured_m"] = None
            terminal_fields["terminal_remaining_m"] = None
            planned_speed = min(planned_speed, terminal_creep_m_s)
    if at_terminal:
        return Twist2D(0.0, 0.0), {"curvature": 0.0, "carrot_xy": (0.0, 0.0),
                                   "measured_base_m": right.rho_base_m,
                                   "target_base_m": target_distance_m,
                                   "distance_error_m": right.rho_base_m - target_distance_m,
                                   "wall_angle_rad": right.bearing_rad,
                                   "planned_speed_m_s": 0.0, "signed_v_m_s": 0.0,
                                   "at_terminal": True, **terminal_fields}

    signed_speed, omega = wall_pure_pursuit(right, target_distance_m,
                                            direction * planned_speed, limits.lookahead_m)
    curvature = omega / signed_speed if signed_speed else 0.0
    # Curvature slowdown, then the cruise floor, then re-apply the braking cap
    # so the floor can never lift it.
    slowed = max(limits.minimum_speed_m_s, planned_speed / (1.0 + 0.5 * abs(curvature)))
    speed = min(slowed, planned_speed, limits.max_linear_speed_m_s)
    speed = max(0.0, speed)
    signed_speed = direction * speed
    omega = signed_speed * curvature
    # Do not let one wheel reverse while the other drives forward during
    # translated motion; pure in-place turns are exempt from this one limit.
    no_wheel_reversal = 1.8 * abs(signed_speed) / limits.track_width_m
    omega = clamp(omega, -min(limits.max_angular_speed_rad_s, no_wheel_reversal),
                  min(limits.max_angular_speed_rad_s, no_wheel_reversal))
    wheel_peak = abs(signed_speed) + abs(omega) * limits.track_width_m / 2.0
    scale = min(1.0, limits.wheel_speed_m_s / max(wheel_peak, 1e-9))
    signed_speed *= scale
    omega *= scale
    nx, ny = right.normal_xy
    tangent_x, tangent_y = -ny, nx
    if tangent_x < 0.0:
        tangent_x, tangent_y = -tangent_x, -tangent_y
    error = right.rho_base_m - target_distance_m
    lookahead_signed = math.copysign(limits.lookahead_m, direction)
    fields = {
        "measured_base_m": right.rho_base_m,
        "target_base_m": target_distance_m,
        "distance_error_m": error,
        "wall_angle_rad": right.bearing_rad,
        "carrot_xy": (error * nx + lookahead_signed * tangent_x,
                      error * ny + lookahead_signed * tangent_y),
        "curvature": curvature,
        "planned_speed_m_s": planned_speed,
        "signed_v_m_s": signed_speed,
        "at_terminal": False,
        **terminal_fields,
    }
    return Twist2D(signed_speed, omega), fields


def turn_command(reference: WallLine, desired_bearing_rad: float, *,
                 limits: WallActionLimits,
                 tolerance_rad: float = TURN_TOLERANCE_RAD,
                 gain: float = TURN_GAIN,
                 max_omega_rad_s: float = TURN_MAX_OMEGA_RAD_S,
                 measured_omega_rad_s: float = 0.0) -> tuple[float, dict]:
    """In-place turn command from the measured wall bearing.

    ``beta = atan2(n_y, n_x)`` decreases as the chassis turns left; the error is
    wrapped, so a normal crossing +/-pi keeps turning the same way instead of
    flipping the chassis around. In-place turns are deliberately exempt from the
    no-wheel-reversal moving limit.

    Inside ``tolerance_rad`` the command is an exact zero rather than a rate
    proportional to the error, so the chassis can actually stop and the settled
    gate can confirm. Note that with sample-and-hold the *measured* angle
    converges to the band edge, not to the exact target: proportional control of
    a rate has no way to land precisely on a setpoint in one tick. That is why
    the completion test is a tolerance band and not an equality.
    """
    error = wrap_to_pi(reference.bearing_rad - desired_bearing_rad)
    if abs(error) <= tolerance_rad:
        omega = 0.0
    else:
        closing_rate = max(0.0, math.copysign(1.0, error) * measured_omega_rad_s)
        remaining = max(0.0, abs(error) - closing_rate * TURN_RESPONSE_S)
        omega = 0.0 if remaining <= tolerance_rad else math.copysign(min(gain * remaining,
                                  max_omega_rad_s,
                                  limits.max_angular_speed_rad_s,
                                  math.sqrt(2.0 * limits.max_angular_accel_rad_s2 * remaining)),
                              error)
    return omega, {"measured_bearing_rad": reference.bearing_rad,
                   "measured_omega_rad_s": measured_omega_rad_s,
                   "desired_bearing_rad": desired_bearing_rad,
                   "error_rad": error}


def normal_move_command(reference: WallLine, target_distance_m: float, direction: int, *,
                        max_speed_m_s: float,
                        tolerance_m: float = DROP_TOLERANCE_M,
                        gain: float = 1.0,
                        angle_tolerance_rad: float = DROP_NORMAL_ANGLE_TOLERANCE_RAD
                        ) -> tuple[float, dict]:
    """Signed speed toward a normal-distance target on one reference wall.

    Since ``rho_dot = -n_x * v``, the speed that closes the distance error is
    ``k * (rho - target) / n_x``. Inside tolerance, or already past the target,
    the command is zero: the action never starts a corrective move the other
    way, and it never integrates time to pretend a distance was travelled.
    A near-zero ``n_x`` means the chassis is not yet normal to the wall, which
    is a turn problem rather than a divide-by-almost-zero problem.
    """
    if direction not in (-1, 1) or isinstance(direction, bool):
        raise ValueError("direction must be +1 or -1")
    if not math.isfinite(max_speed_m_s) or max_speed_m_s <= 0:
        raise ValueError("max_speed_m_s must be finite and positive")
    nx = reference.normal_xy[0]
    if abs(nx) < math.cos(angle_tolerance_rad):
        raise ValueError("chassis must first face normal to the reference wall")
    error = reference.rho_base_m - target_distance_m
    travel = error / nx
    remaining = direction * travel
    speed = 0.0 if remaining <= tolerance_m else direction * min(max_speed_m_s, gain * remaining)
    return speed, {"measured_base_m": reference.rho_base_m, "target_base_m": target_distance_m,
                   "distance_error_m": error, "normal_x": nx,
                   "remaining_along_direction_m": remaining}


@dataclass
class FreshStopGate:
    """Only scans acquired after zero output can confirm a stopped turn.

    ``zero_commanded_s`` is the instant zero output was *first* commanded and must
    not be re-stamped while the hold continues. A confirming revolution must have
    been acquired entirely after that instant; re-reading one snapshot never
    accumulates.
    """

    required: int = TURN_STABLE_FRAMES
    max_age_s: float = 0.40
    tolerance_rad: float = TURN_TOLERANCE_RAD
    last_seq: int = -1
    stable: int = 0

    def reset(self) -> None:
        self.last_seq = -1
        self.stable = 0

    def observe(self, *, snapshot: WallSnapshot, now_s: float, zero_commanded_s: float,
                error_rad: float) -> bool:
        values = (snapshot.scan_started_s, snapshot.scan_completed_s, now_s,
                  zero_commanded_s, error_rad)
        if not all(math.isfinite(value) for value in values):
            self.stable = 0
            return False
        # The revolution must have been acquired *entirely* after zero output was
        # commanded. One that straddles the command still contains samples taken
        # while the chassis was turning, so counting it would declare the turn
        # settled a whole revolution too early.
        fresh = (zero_commanded_s <= snapshot.scan_started_s
                 <= snapshot.scan_completed_s <= now_s
                 and now_s - (snapshot.scan_started_s + snapshot.scan_completed_s) / 2.0
                 <= self.max_age_s)
        if not fresh or abs(error_rad) > self.tolerance_rad:
            self.stable = 0
            return False
        if snapshot.scan_seq <= self.last_seq:
            return False
        self.last_seq = snapshot.scan_seq
        self.stable += 1
        return self.stable >= self.required


@dataclass
class ActionOutcome:
    """Simple result of one wall action; no ``.estimate.pose`` is required.

    ``waiting`` marks an ordinary "no usable wall right now, holding zero" tick.
    It is an explicit flag rather than a reason string so the outage timer cannot
    be reset by a status the base class does not recognise.
    """

    action: str
    done: bool
    reason: str = ""
    fields: dict = field(default_factory=dict)
    waiting: bool = False

    @property
    def command(self) -> Twist2D:
        return self.fields.get("command", Twist2D(0.0, 0.0))


class WallAction:
    """Base class for the independently callable wall actions.

    A subclass implements ``_tick(snapshot, now_s)``. ``step()`` is called once
    per control tick: a missing or stale wall yields zero speed and waits for a
    fresh observation; it never fabricates a position and never finishes on a
    timer.
    """

    name = "wall-action"

    def __init__(self, *, clock: Callable[[], float] = time.monotonic,
                 max_age_s: float = 0.40,
                 lost_timeout_s: float = WALL_LOST_TIMEOUT_S) -> None:
        self.clock = clock
        self.max_age_s = max_age_s
        self.lost_timeout_s = lost_timeout_s
        self.started_s: float | None = None
        self.zero_since_s: float | None = None
        self.last_error_s: float | None = None
        self.finished = False
        self.outcome: ActionOutcome | None = None

    @property
    def began(self) -> bool:
        return self.started_s is not None

    def begin(self, now_s: float) -> None:
        self.started_s = now_s
        self.zero_since_s = now_s

    def _record_zero(self, now_s: float) -> None:
        if self.zero_since_s is None:
            self.zero_since_s = now_s

    def _record_nonzero(self, now_s: float) -> None:
        self.zero_since_s = None

    def step(self, snapshot: WallSnapshot | None, *, now_s: float | None = None) -> ActionOutcome:
        now = float(self.clock()) if now_s is None else float(now_s)
        if self.started_s is None:
            self.started_s = now
        if self.finished and self.outcome is not None:
            return self.outcome
        if snapshot is None:
            return self._wait(now, "no_wall_observation")
        if not snapshot.is_fresh(now, max_age_s=self.max_age_s):
            return self._wait(now, "stale_wall_observation")
        outcome = self._tick(snapshot, now)
        # Clear the outage timer only once this tick actually found what it was
        # waiting for. Clearing it before ``_tick`` would make a persistently
        # lost reference wall retry forever instead of failing.
        if not outcome.waiting:
            self.last_error_s = None
        return outcome

    def _wait(self, now_s: float, reason: str, **fields) -> ActionOutcome:
        # Stamp the start of the outage once; re-stamping every tick would mean
        # the lost-wall timeout could never elapse.
        if self.last_error_s is None:
            self.last_error_s = now_s
        self._record_zero(now_s)
        if now_s - self.last_error_s > self.lost_timeout_s:
            return self._finish(False, reason)
        return ActionOutcome(self.name, False, reason, dict(fields), waiting=True)

    def _tick(self, snapshot: WallSnapshot, now_s: float) -> ActionOutcome:  # pragma: no cover
        raise NotImplementedError

    def _finish(self, done: bool, reason: str = "", **fields) -> ActionOutcome:
        self.finished = True
        self.outcome = ActionOutcome(self.name, done, reason, dict(fields))
        return self.outcome


class FollowRightWall(WallAction):
    """``follow_right_wall(target_distance_base, direction, speed_cap, terminal)``.

    A terminal wall is a *different* wall from the right wall. The caller passes
    per-leg measured role predicates for it (a front wall while closing a
    corner, a rear wall while reversing along the top edge), and those are
    re-evaluated on every frame rather than frozen at leg start.
    """

    name = "follow_right_wall"

    def __init__(self, *, target_distance_m: float, direction: int,
                 limits: WallActionLimits, speed_cap_m_s: float | None = None,
                 tracker: WallTracker | None = None,
                 terminal_role=None,
                 terminal_target_m: float | None = None,
                 terminal_tolerance_m: float = CORNER_TOLERANCE_M,
                 terminal_creep_m_s: float = TERMINAL_CREEP_SPEED_M_S,
                 terminal_track_angle_rad: float = TERMINAL_TRACK_ANGLE_RAD,
                 terminal_lost_timeout_s: float = TERMINAL_LOST_TIMEOUT_S,
                 lost_timeout_s: float = WALL_LOST_TIMEOUT_S,
                 clock: Callable[[], float] = time.monotonic,
                 max_age_s: float = 0.40) -> None:
        super().__init__(clock=clock, max_age_s=max_age_s, lost_timeout_s=lost_timeout_s)
        if direction not in (-1, 1) or isinstance(direction, bool):
            raise ValueError("direction must be +1 or -1")
        if not callable(terminal_role) and terminal_target_m is not None:
            raise ValueError("a terminal target requires a terminal wall predicate")
        self.target_distance_m = target_distance_m
        self.direction = direction
        self.limits = limits
        self.speed_cap_m_s = speed_cap_m_s
        self.tracker = tracker if tracker is not None else WallTracker()
        self.terminal_role = terminal_role
        self.terminal_target_m = terminal_target_m
        self.terminal_tolerance_m = terminal_tolerance_m
        self.terminal_creep_m_s = terminal_creep_m_s
        self.terminal_track_angle_rad = terminal_track_angle_rad
        self.terminal_lost_timeout_s = terminal_lost_timeout_s
        # The verified terminal wall is handed to the mission layer so the corner
        # turn can reference the wall that was actually measured. Never a boolean.
        self.terminal_wall: WallLine | None = None
        self.terminal_seen_at_s: float | None = None
        self._terminal_reference: WallLine | None = None
        self._terminal_visible = False
        self._terminal_lost_since_s: float | None = None
        self._terminal_candidates: list[dict] = []

    def _pick_right(self, snapshot: WallSnapshot):
        if self.tracker.wall is None:
            candidates = [line for line in snapshot.lines if is_right_wall(line)]
            if not candidates:
                return None
            # Seeding is the one moment a control target may pick the wall: it
            # chooses a fresh observation, it does not sustain a track.
            seed = min(candidates, key=lambda line: abs(line.rho_base_m - self.target_distance_m))
            return self.tracker.set_reference(seed)
        # Association is centred on the last *measured* distance of this wall,
        # never on the distance the chassis is being commanded toward: a target
        # the car is approaching can coincide with a different parallel
        # structure, which would then be accepted as "already arrived".
        return self.tracker.track(snapshot.lines,
                                  expected_bearing_rad=self.tracker.wall.bearing_rad,
                                  expected_distance_m=self.tracker.last_rho_m)

    def _pick_terminal(self, snapshot: WallSnapshot, right: WallLine | None = None):
        """Measure a long perpendicular wall even when its corner is occluded.

        The scan must fit a real long wall in the requested sector. Requiring
        both supports to extend to their intersection can suppress this distance
        while a corner is occluded. The field run first accepted a distance
        40 cm past its stop position; missing distances must instead hold zero.
        """
        if self.terminal_role is None:
            return None
        candidates = [line for line in snapshot.lines if self.terminal_role(line)]
        self._terminal_candidates = [dict(rho_base_m=line.rho_base_m,
                                          bearing_rad=line.bearing_rad, span_m=line.span_m)
                                     for line in candidates]
        right = self.tracker.wall if right is None else right
        if right is not None:
            def usable_distance(line):
                # Retain the measured length, sector and perpendicularity gates;
                # a short pillar or the road wall cannot end the leg.
                if line.span_m < MIN_TERMINAL_SPAN_M:
                    return False
                if _same_line(line, right):
                    return False
                if abs(line.normal_xy[0] * right.normal_xy[0]
                       + line.normal_xy[1] * right.normal_xy[1]) > \
                        math.sin(TERMINAL_ORTHOGONAL_TOLERANCE_RAD):
                    return False
                return True
            candidates = [line for line in candidates if usable_distance(line)]
        if not candidates:
            return None
        if self._terminal_reference is not None:
            reference = self._terminal_reference
            continuous = [line for line in candidates
                          if abs(wrap_to_pi(line.bearing_rad - reference.bearing_rad))
                          <= self.terminal_track_angle_rad
                          and abs(line.rho_base_m - reference.rho_base_m)
                          <= self.tracker.distance_gate_m]
            if not continuous:
                return None
            chosen = min(continuous, key=lambda line: abs(line.rho_base_m - reference.rho_base_m))
        else:
            chosen = min(candidates, key=lambda line: line.rho_base_m)
        self._terminal_reference = chosen
        return chosen

    def _tick(self, snapshot: WallSnapshot, now_s: float) -> ActionOutcome:
        right = self._pick_right(snapshot)
        if right is None:
            return self._wait(now_s, "right_wall_not_visible")
        terminal = self._pick_terminal(snapshot) if self.terminal_role is not None else None
        self._terminal_visible = terminal is not None
        fields: dict = {"terminal_candidates": self._terminal_candidates}
        if self.terminal_role is None:
            # No terminal for this leg: plain wall following.
            terminal_target = None
            terminal_measured = None
        elif terminal is not None:
            # A fresh measurement of the specified terminal wall.
            self.terminal_wall = terminal
            self.terminal_seen_at_s = now_s
            self._terminal_lost_since_s = None
            terminal_target = self.terminal_target_m
            terminal_measured = terminal.rho_base_m
            if terminal_measured < terminal_target - self.terminal_tolerance_m:
                self._record_zero(now_s)
                return self._finish(False, "terminal_distance_overshot",
                                    terminal_measured_m=terminal_measured,
                                    terminal_target_m=terminal_target)
        else:
            # A missing distance never grants permission to advance, including
            # before the first detection. Wait at zero with a bounded timeout.
            reason = ("terminal_wall_unconfirmed" if self.terminal_seen_at_s is None
                      else "terminal_wall_lost")
            if self._terminal_lost_since_s is None:
                self._terminal_lost_since_s = now_s
            lost_for = now_s - self._terminal_lost_since_s
            if lost_for > self.terminal_lost_timeout_s:
                return self._finish(False, reason)
            self._record_zero(now_s)
            fields["terminal_visible"] = False
            fields["terminal_seen"] = self.terminal_seen_at_s is not None
            fields["terminal_lost_for_s"] = lost_for
            return ActionOutcome(self.name, False, reason,
                                 dict(fields, command=Twist2D(0.0, 0.0)), waiting=True)
        command, control = follow_command(
            right, self.target_distance_m, self.direction, limits=self.limits,
            speed_cap_m_s=self.speed_cap_m_s,
            terminal_measured_m=terminal_measured,
            terminal_target_m=terminal_target,
            terminal_tolerance_m=self.terminal_tolerance_m,
            terminal_creep_m_s=self.terminal_creep_m_s)
        fields.update(control)
        fields["terminal_visible"] = self._terminal_visible
        fields["terminal_seen"] = self.terminal_seen_at_s is not None
        fields["right_wall_rho_base_m"] = right.rho_base_m
        fields["right_wall_normal"] = right.normal_xy
        fields["right_wall_span_m"] = right.span_m
        fields["right_wall_rms_m"] = right.rms_m
        if fields.get("at_terminal"):
            self._record_zero(now_s)
            return self._finish(True, "terminal_reached", **fields)
        if abs(command.linear_x_m_s) < 1e-9 and abs(command.angular_z_rad_s) < 1e-9:
            self._record_zero(now_s)
        else:
            self._record_nonzero(now_s)
        return ActionOutcome(self.name, False, "", dict(fields, command=command))


class TurnToWallBearing(WallAction):
    """``turn_to_wall_bearing(reference_wall, desired_bearing, direction)``.

    Only the measured wall angle ends the turn: two fresh scans, acquired after
    zero output, inside the angle tolerance. There is no fallback that
    integrates the commanded rate to 90 degrees.
    """

    name = "turn_to_wall_bearing"

    def __init__(self, *, reference_bearing_rad: float, desired_bearing_rad: float,
                 turn_direction: int, limits: WallActionLimits,
                 tracker: WallTracker | None = None,
                 gain: float = TURN_GAIN,
                 max_omega_rad_s: float = TURN_MAX_OMEGA_RAD_S,
                 tolerance_rad: float = TURN_TOLERANCE_RAD,
                 required_frames: int = TURN_STABLE_FRAMES,
                 clock: Callable[[], float] = time.monotonic,
                 max_age_s: float = 0.40,
                 lost_timeout_s: float = TURN_LOST_TIMEOUT_S) -> None:
        super().__init__(clock=clock, max_age_s=max_age_s, lost_timeout_s=lost_timeout_s)
        if turn_direction not in (-1, 1) or isinstance(turn_direction, bool):
            raise ValueError("turn_direction must be +1 (left) or -1 (right)")
        if not math.isfinite(reference_bearing_rad) or not math.isfinite(desired_bearing_rad):
            raise ValueError("finite bearings required")
        self.reference_bearing_rad = reference_bearing_rad
        self.desired_bearing_rad = desired_bearing_rad
        self.turn_direction = turn_direction
        self.limits = limits
        self.tracker = tracker if tracker is not None else WallTracker()
        self.gain = gain
        self.max_omega_rad_s = max_omega_rad_s
        self.tolerance_rad = tolerance_rad
        self.gate = FreshStopGate(required=required_frames, max_age_s=max_age_s,
                                  tolerance_rad=tolerance_rad)
        self._continuous_bearing_rad: float | None = reference_bearing_rad
        self._holding = False
        self._zero_commanded_s: float | None = None
        self._last_tick_s: float | None = None
        self._last_scan_seq = -1
        self._last_measured_s: float | None = None
        self._last_bearing_rad: float | None = None
        self._measured_omega_rad_s = 0.0

    def _track(self, snapshot: WallSnapshot):
        if self._continuous_bearing_rad is None:
            return None
        wall = self.tracker.track(snapshot.lines,
                                  expected_bearing_rad=self._continuous_bearing_rad,
                                  expected_distance_m=self.tracker.last_rho_m)
        if wall is None:
            # A stale cached line must not keep answering "visible"; clearing it
            # is what lets the lost-wall timeout expire instead of holding the
            # reference forever.
            self.tracker.clear()
        return wall

    def _tick(self, snapshot: WallSnapshot, now_s: float) -> ActionOutcome:
        wall = self._track(snapshot)
        if wall is None:
            # Hold zero and wait for the reference to come back.
            self._last_measured_s = None
            self._last_bearing_rad = None
            self._measured_omega_rad_s = 0.0
            return self._wait(now_s, "reference_wall_lost")
        previous = self._continuous_bearing_rad
        self._continuous_bearing_rad = previous + wrap_to_pi(wall.bearing_rad - previous)
        if snapshot.scan_seq > self._last_scan_seq:
            measured_s = snapshot.observed_at_s
            if self._last_measured_s is not None and measured_s > self._last_measured_s:
                self._measured_omega_rad_s = -wrap_to_pi(
                    wall.bearing_rad - self._last_bearing_rad) / (measured_s - self._last_measured_s)
            self._last_measured_s = measured_s
            self._last_bearing_rad = wall.bearing_rad
            self._last_scan_seq = snapshot.scan_seq
        omega, fields = turn_command(wall, self.desired_bearing_rad, limits=self.limits,
                                     tolerance_rad=self.tolerance_rad, gain=self.gain,
                                     max_omega_rad_s=self.max_omega_rad_s,
                                     measured_omega_rad_s=self._measured_omega_rad_s)
        fields["reference_rho_base_m"] = wall.rho_base_m
        fields["tracked_bearing_rad"] = self._continuous_bearing_rad
        fields["scan_seq"] = snapshot.scan_seq
        if abs(omega) < 1e-9:
            if not self._holding:
                # Entering the hold: record the instant zero output was first
                # commanded. It must NOT be re-stamped on later ticks, or no
                # revolution could ever be newer than it -- and it must not be
                # back-dated to the previous tick either, because that tick could
                # still have carried a non-zero turn command.
                self._holding = True
                self._zero_commanded_s = now_s
                self.zero_since_s = now_s
                self.gate.reset()
            self._record_zero(now_s)
        else:
            self._holding = False
            self._zero_commanded_s = None
            self._record_nonzero(now_s)
        fields["stable_count"] = self.gate.stable
        if self._holding:
            assert self._zero_commanded_s is not None
            if abs(self._measured_omega_rad_s) > TURN_SETTLED_RATE_RAD_S:
                self.gate.reset()
                done = False
            else:
                done = self.gate.observe(snapshot=snapshot, now_s=now_s,
                                         zero_commanded_s=self._zero_commanded_s,
                                         error_rad=fields["error_rad"])
            fields["stable_count"] = self.gate.stable
            if done:
                return self._finish(True, "bearing_reached", **fields)
            return ActionOutcome(self.name, False, "settling",
                                 dict(fields, command=Twist2D(0.0, 0.0)), waiting=True)
        return ActionOutcome(self.name, False, "", dict(fields, command=Twist2D(0.0, omega)))


class MoveNormalToWall(WallAction):
    """``move_normal_to_wall(reference_wall, target_distance_base, direction, cap)``.

    Uses the normal distance to one reference wall and is used by the payload
    detour for the 43/32 cm in-and-out moves. It never claims lateral
    (along-wall) correction: a single wall cannot observe that.
    """

    name = "move_normal_to_wall"

    def __init__(self, *, target_distance_m: float, direction: int,
                 reference_bearing_rad: float, limits: WallActionLimits,
                 tracker: WallTracker | None = None,
                 max_speed_m_s: float = DROP_FINE_SPEED_M_S,
                 terminal_speed_m_s: float = DROP_TERMINAL_SPEED_M_S,
                 terminal_range_m: float = DROP_TERMINAL_RANGE_M,
                 tolerance_m: float = DROP_TOLERANCE_M,
                 heading_gain: float = DROP_HEADING_GAIN,
                 heading_band_rad: float = DROP_HEADING_BAND_RAD,
                 heading_tolerance_rad: float | None = None,
                 settle_margin_rad: float = HEADING_SETTLE_MARGIN_RAD,
                 desired_bearing_rad: float | None = None,
                 lost_timeout_s: float = WALL_LOST_TIMEOUT_S,
                 clock: Callable[[], float] = time.monotonic,
                 max_age_s: float = 0.40) -> None:
        super().__init__(clock=clock, max_age_s=max_age_s, lost_timeout_s=lost_timeout_s)
        if direction not in (-1, 1) or isinstance(direction, bool):
            raise ValueError("direction must be +1 or -1")
        if not math.isfinite(target_distance_m) or target_distance_m <= 0:
            raise ValueError("target_distance_m must be finite and positive")
        if not math.isfinite(heading_gain) or heading_gain < 0:
            raise ValueError("heading_gain must be finite and non-negative")
        if not math.isfinite(heading_band_rad) or heading_band_rad < 0:
            raise ValueError("heading_band_rad must be finite and non-negative")
        self.target_distance_m = target_distance_m
        self.direction = direction
        self.reference_bearing_rad = reference_bearing_rad
        self.limits = limits
        self.tracker = tracker if tracker is not None else WallTracker()
        self.max_speed_m_s = min(max_speed_m_s, limits.max_linear_speed_m_s)
        self.terminal_speed_m_s = min(terminal_speed_m_s, self.max_speed_m_s)
        self.terminal_range_m = terminal_range_m
        self.tolerance_m = tolerance_m
        self.heading_gain = heading_gain
        self.heading_band_rad = heading_band_rad
        # Success is judged a little outside the dead band. The two must not be
        # equal: the controller stops correcting exactly at the band edge, so a
        # residual that settles a nanoradian beyond it would never be corrected
        # again and the move would wait forever just short of success.
        self.heading_tolerance_rad = (max(heading_band_rad + settle_margin_rad,
                                          DROP_HEADING_TOLERANCE_RAD)
                                      if heading_tolerance_rad is None
                                      else heading_tolerance_rad)
        if self.heading_tolerance_rad < heading_band_rad:
            raise ValueError("heading_tolerance_rad must not be below the dead band")
        # The heading this move holds: the reference wall keeps its bearing while
        # the chassis translates square to it.
        self.desired_bearing_rad = (reference_bearing_rad if desired_bearing_rad is None
                                    else desired_bearing_rad)
        self._continuous_bearing_rad: float | None = reference_bearing_rad

    def _track(self, snapshot: WallSnapshot):
        if self._continuous_bearing_rad is None:
            return None
        wall = self.tracker.track(snapshot.lines,
                                  expected_bearing_rad=self._continuous_bearing_rad,
                                  expected_distance_m=self.target_distance_m)
        if wall is None:
            # Drop the cached line. Leaving it in place would let a stale wall
            # keep answering "the reference is visible", so the lost-wall
            # timeout could never fire and the move would keep driving on
            # geometry that is no longer being measured.
            self.tracker.clear()
            return None
        previous = self._continuous_bearing_rad
        self._continuous_bearing_rad = previous + wrap_to_pi(wall.bearing_rad - previous)
        return wall

    def _tick(self, snapshot: WallSnapshot, now_s: float) -> ActionOutcome:
        wall = self._track(snapshot)
        if wall is None:
            return self._wait(now_s, "reference_wall_not_visible")
        try:
            probe_speed, probe_info = normal_move_command(
                wall, self.target_distance_m, self.direction,
                max_speed_m_s=self.max_speed_m_s, tolerance_m=self.tolerance_m)
        except ValueError as exc:
            # Not yet normal to the reference wall. Hold zero and let the shared
            # outage timeout bound it, instead of retrying forever.
            return self._wait(now_s, f"heading_not_normal ({exc})")
        # The remaining distance in METRES comes from the command's own report.
        # Comparing the returned speed (m/s) against terminal_range_m (m) would
        # put the whole 43/32 cm approach into the final 6 cm slow zone.
        remaining_m = probe_info["remaining_along_direction_m"]
        cap = (self.terminal_speed_m_s if remaining_m <= self.terminal_range_m
               else self.max_speed_m_s)
        speed, fields = normal_move_command(wall, self.target_distance_m, self.direction,
                                            max_speed_m_s=cap, tolerance_m=self.tolerance_m)
        fields["speed_cap_m_s"] = cap
        reaching = abs(speed) < 1e-9
        # Hold the chassis square to the same wall while it translates. With the
        # chassis facing the reference wall, the wall normal is parallel to the
        # nose, so the angle between that wall and the radar's forward axis is
        # exactly ``90 deg - heading_error``. Inside the band the heading is good
        # enough and the angular command is an exact zero; outside it the error is
        # corrected while the move continues, which is what makes the 43/32 cm
        # reach land squarely instead of carrying a tilt sideways.
        desired = (self.desired_bearing_rad if self.desired_bearing_rad is not None
                   else wall.bearing_rad)
        heading_error = self._axis_error(wall)
        fields["desired_bearing_rad"] = desired
        fields["heading_error_rad"] = heading_error
        fields["heading_error_deg"] = math.degrees(heading_error)
        fields["heading_band_deg"] = math.degrees(self.heading_band_rad)
        square = abs(heading_error) <= self.heading_tolerance_rad
        fields["heading_within_band"] = square
        if reaching and square:
            self._record_zero(now_s)
            return self._finish(True, "distance_reached", **fields)
        # Reaching the distance is not success on its own. A chassis that arrives
        # 40 degrees off has the right perpendicular distance and the wrong
        # orientation, and calling that done would abandon the correction the
        # operator asked for. So the move keeps working the heading, in place,
        # until it is square; the shared outage timeout bounds it if it cannot.
        if reaching and not square:
            omega = self.heading_command(heading_error)
            fields["corrected_omega_rad_s"] = omega
            fields["speed_cap_m_s"] = self.max_speed_m_s
            fields["squaring_in_place"] = True
            self._record_zero(now_s)
            if self.last_error_s is None:
                self.last_error_s = now_s
            if now_s - self.last_error_s > self.lost_timeout_s:
                return self._finish(False, "squaring_heading", **fields)
            # Rotate in place: the distance is already met, so the only thing
            # left is the orientation the operator asked for.
            return ActionOutcome(self.name, False, "squaring_heading",
                                 dict(fields, command=Twist2D(0.0, omega)),
                                 waiting=True)
        omega = self.heading_command(heading_error)
        fields["corrected_omega_rad_s"] = omega
        self._record_nonzero(now_s)
        return ActionOutcome(self.name, False, "", dict(fields, command=Twist2D(speed, omega)))

    def _axis_error(self, line: WallLine) -> float:
        """Heading error of ``line`` against the axis it was locked on at entry.

        ``rho`` and the normal are both invariant under ``n -> -n``, so a wall
        seen from the far side is indistinguishable in those terms and the
        tracked bearing can flip by pi. Comparing against the storage axis of the
        *entry* bearing keeps a 180 degree flip from reading as a 180 degree
        heading error, without needing to know which side the wall is on.
        """
        storage = wrap_to_pi(self.reference_bearing_rad)
        unwrapped = storage + wrap_to_pi(line.bearing_rad - storage)
        return wrap_to_pi(unwrapped - self.desired_bearing_rad)

    def heading_command(self, heading_error_rad: float) -> float:
        """Angular command that holds the wall square, with a dead band.

        Zero inside ``heading_band_rad`` so a small residual tilt does not make
        the chassis fidget; outside it, a proportional correction bounded by the
        chassis angular limit and by a braking-shaped limit (``sqrt(2*a*error)``)
        so it decelerates into the band instead of overshooting through it.

        In-place rotation is deliberately NOT used here: reversing one wheel
        mid-approach would fight the C10B reverse handling and the wheel-speed
        limit, whereas an angular offset on a forward or reverse move is a plain
        differential and needs no special case.
        """
        if abs(heading_error_rad) <= self.heading_band_rad:
            return 0.0
        bias = math.copysign(self.heading_band_rad, heading_error_rad)
        magnitude = min(self.heading_gain * abs(heading_error_rad - bias),
                        self.limits.max_angular_speed_rad_s,
                        math.sqrt(2.0 * self.limits.max_angular_accel_rad_s2
                                  * abs(heading_error_rad - bias)))
        return math.copysign(magnitude, heading_error_rad)


class HoldStopped(WallAction):
    """``hold_stopped(duration)``: a cancellable zero-output wait."""

    name = "hold_stopped"

    def __init__(self, *, duration_s: float, clock: Callable[[], float] = time.monotonic,
                 tick_s: float = HOLD_TICK_S) -> None:
        super().__init__(clock=clock, max_age_s=math.inf, lost_timeout_s=math.inf)
        if not math.isfinite(duration_s) or duration_s < 0:
            raise ValueError("duration_s must be finite and non-negative")
        self.duration_s = duration_s
        self.tick_s = tick_s

    def step(self, snapshot: WallSnapshot | None, *, now_s: float | None = None) -> ActionOutcome:
        now = float(self.clock()) if now_s is None else float(now_s)
        if self.started_s is None:
            self.started_s = now
        if self.finished and self.outcome is not None:
            return self.outcome
        self._record_zero(now)
        elapsed = now - self.started_s
        if elapsed >= self.duration_s:
            return self._finish(True, "held", elapsed_s=elapsed)
        return ActionOutcome(self.name, False, "", {
            "remaining_s": self.duration_s - elapsed, "command": Twist2D(0.0, 0.0)})


def corner_turn(reference_bearing_rad: float, *, limits: WallActionLimits,
                tracker: WallTracker | None = None, **kwargs) -> TurnToWallBearing:
    """Drive-axis-centre left turn so the wall that was in front becomes the right wall."""
    return TurnToWallBearing(reference_bearing_rad=reference_bearing_rad,
                             desired_bearing_rad=RIGHT_BEARING_RAD,
                             turn_direction=1, limits=limits, tracker=tracker, **kwargs)


def drop_turn(reference_bearing_rad: float, desired_bearing_rad: float, *,
              limits: WallActionLimits, tracker: WallTracker | None = None,
              turn_direction: int = 1, **kwargs) -> TurnToWallBearing:
    """Turn that swings the tracked road wall to a new body bearing."""
    return TurnToWallBearing(reference_bearing_rad=reference_bearing_rad,
                             desired_bearing_rad=desired_bearing_rad,
                             turn_direction=turn_direction, limits=limits,
                             tracker=tracker, **kwargs)
