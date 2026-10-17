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

from dataclasses import dataclass, field, replace
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
from config.mission_profile import WALL_CRUISE_SPEED_M_S

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
TERMINAL_DISTINCT_LINES_M = 0.10
DROP_TERMINAL_RANGE_M = 0.06
DROP_FINE_SPEED_M_S = 0.15
DROP_TERMINAL_SPEED_M_S = 0.012
# Heading hold while translating squarely toward or away from one wall. The band
# is the allowed residual tilt; outside it the correction is proportional with a
# gain that reaches the chassis angular limit at the edge of a 5 deg-plus band.
DROP_HEADING_BAND_RAD = math.radians(5.0)
DROP_HEADING_GAIN = 1.5
TERMINAL_CREEP_SPEED_M_S = 0.008

# Operator-requested cruise target; the runtime profile sets the drive envelope.
CRUISE_SPEED_FACTOR = WALL_CRUISE_SPEED_M_S
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


def _reframe_wall(line: WallLine, road_before: WallLine, road_after: WallLine) -> WallLine:
    rotation = wrap_to_pi(road_after.bearing_rad - road_before.bearing_rad)
    c, s = math.cos(rotation), math.sin(rotation)
    shift = road_after.rho_base_m - road_before.rho_base_m
    dx, dy = (shift * v for v in road_after.normal_xy)
    def rotate(p):
        return c * p[0] - s * p[1], s * p[0] + c * p[1]
    normal = rotate(line.normal_xy)
    def point(p):
        x, y = rotate(p)
        return x + dx, y + dy
    return replace(line, normal_xy=normal,
                   rho_base_m=line.rho_base_m + normal[0] * dx + normal[1] * dy,
                   support_start_xy=point(line.support_start_xy),
                   support_end_xy=point(line.support_end_xy))


def _terminal_support_gap(road: WallLine, terminal: WallLine) -> float:
    a, b = road.normal_xy
    c, d = terminal.normal_xy
    determinant = a * d - b * c
    if abs(determinant) < 1e-9:
        return math.inf
    corner = ((road.rho_base_m * d - b * terminal.rho_base_m) / determinant,
              (a * terminal.rho_base_m - road.rho_base_m * c) / determinant)
    return terminal.distance_to_segment(corner)


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

    Braking is applied last, so the cruise floor cannot lift it. A crossed
    target reverses along the road; only an absolute error inside tolerance
    ends the leg.
    """
    if direction not in (-1, 1) or isinstance(direction, bool):
        raise ValueError("direction must be +1 or -1")
    # No terminal measurement means ordinary right-wall following. An old range
    # is association evidence only, never a current control measurement.
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
            if abs(remaining) <= terminal_tolerance_m:
                at_terminal = True
            else:
                # Recover a crossed stop line along the same right-wall path.
                # ``direction`` still denotes the nominal leg (front or rear).
                if remaining < 0.0:
                    direction = -direction
                brakable = braking_cap(abs(remaining), 0.0, planned_speed,
                                       deceleration_m_s2=limits.max_linear_accel_m_s2,
                                       tolerance_m=terminal_tolerance_m)
                # Never creep faster than the caller's cap, and never force an
                # abrupt stop outside tolerance just because the braking curve
                # rounds to zero.
                planned_speed = min(max(brakable, min(terminal_creep_m_s, planned_speed)),
                                    planned_speed)
        else:
            terminal_fields["terminal_measured_m"] = None
            terminal_fields["terminal_remaining_m"] = None
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
    if abs(nx) < 1e-6:
        raise ValueError("reference wall has no longitudinal projection")
    error = reference.rho_base_m - target_distance_m
    travel = error / nx
    remaining = direction * travel
    speed = 0.0 if remaining <= tolerance_m else direction * min(max_speed_m_s, gain * remaining)
    return speed, {"measured_base_m": reference.rho_base_m, "target_base_m": target_distance_m,
                   "distance_error_m": error, "normal_x": nx,
                   "remaining_along_direction_m": remaining}


def normal_heading_command(error: float, limits: WallActionLimits, *,
                           band_rad: float = DROP_HEADING_BAND_RAD,
                           gain: float = DROP_HEADING_GAIN) -> float:
    if abs(error) <= band_rad:
        return 0.0
    outside = abs(error) - band_rad
    return math.copysign(min(gain * outside, limits.max_angular_speed_rad_s,
                            math.sqrt(2.0 * limits.max_angular_accel_rad_s2 * outside)), error)


def can_follow_wall(wall: WallLine, target_m: float, limits: WallActionLimits) -> bool:
    """Hand off a coarse turn when the existing moving controller is feasible."""
    if not is_right_wall(wall):
        return False
    v, omega = wall_pure_pursuit(wall, target_m, limits.follow_speed_m_s, limits.lookahead_m)
    nx, ny = wall.normal_xy
    carrot_x = (wall.rho_base_m - target_m) * nx - limits.lookahead_m * ny
    return (carrot_x > 0.0 and abs(omega) <= limits.max_angular_speed_rad_s
            and abs(omega) * limits.track_width_m <= 1.8 * abs(v))


def can_move_normal(wall: WallLine, desired_bearing_rad: float,
                    limits: WallActionLimits) -> bool:
    error = wrap_to_pi(wall.bearing_rad - desired_bearing_rad)
    if math.cos(error) <= 0.0 or abs(wall.normal_xy[0]) < 1e-6:
        return False
    omega = normal_heading_command(error, limits)
    speed = min(DROP_FINE_SPEED_M_S, limits.max_linear_speed_m_s)
    return abs(omega) * limits.track_width_m <= 1.8 * speed


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
        # Clear the remembered outage once usable feedback has returned. A
        # missing observation remains a wait and never finishes the action.
        if not outcome.waiting:
            self.last_error_s = None
        return outcome

    def _wait(self, now_s: float, reason: str, **fields) -> ActionOutcome:
        # Missing feedback is a recoverable wait, never a task failure timer.
        if self.last_error_s is None:
            self.last_error_s = now_s
        self._record_zero(now_s)
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
        self._travel_direction = direction
        self.limits = limits
        self.speed_cap_m_s = speed_cap_m_s
        self.tracker = tracker if tracker is not None else WallTracker()
        self.terminal_role = terminal_role
        self.terminal_target_m = terminal_target_m
        self.terminal_tolerance_m = terminal_tolerance_m
        self.terminal_creep_m_s = terminal_creep_m_s
        self.terminal_track_angle_rad = terminal_track_angle_rad
        # The verified terminal wall is handed to the mission layer so the corner
        # turn can reference the wall that was actually measured. Never a boolean.
        self.terminal_wall: WallLine | None = None
        self.terminal_seen_at_s: float | None = None
        self._terminal_reference: WallLine | None = None
        self._terminal_visible = False
        self._terminal_reference_s: float | None = None
        self._terminal_right: WallLine | None = None
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

    def inherit_terminal(self, previous: FollowRightWall, right_before: WallLine | None) -> None:
        """Carry a leg's wall identity across a payload interruption, not its outcome.

        Re-express the prior using the same road wall's measured rotation and
        normal displacement. Along-road detour drift is not observable from one
        wall; the old range remains a prior, never a fresh range for control.
        """
        reference = previous._terminal_reference
        right_after = self.tracker.wall
        if reference is None or right_before is None or right_after is None:
            return
        self._terminal_reference = _reframe_wall(reference, right_before, right_after)
        self._terminal_right = right_after
        self._terminal_reference_s = float(self.clock())
        self.terminal_seen_at_s = previous.terminal_seen_at_s
        self._travel_direction = previous._travel_direction

    def _pick_terminal(self, snapshot: WallSnapshot, right: WallLine, now_s: float):
        if self.terminal_role is None:
            return None
        reference = self._terminal_reference
        if reference is not None and self._terminal_right is not None:
            reference = _reframe_wall(reference, self._terminal_right, right)
            self._terminal_reference = reference
        self._terminal_right = right
        self._terminal_candidates = []
        candidates = []
        # The speed limit only bounds possible association displacement during a
        # dropout. It does not integrate drive commands or invent a measurement.
        elapsed = max(0.0, now_s - self._terminal_reference_s) \
            if self._terminal_reference_s is not None else 0.0
        distance_gate = self.tracker.distance_gate_m + self.limits.max_linear_speed_m_s * elapsed
        for line in snapshot.lines:
            if not self.terminal_role(line):
                continue
            reason = None
            if line.span_m < MIN_TERMINAL_SPAN_M:
                reason = "short_support"
            elif _same_line(line, right):
                reason = "road_wall"
            elif abs(sum(a * b for a, b in zip(line.normal_xy, right.normal_xy))) > \
                    math.sin(TERMINAL_ORTHOGONAL_TOLERANCE_RAD):
                reason = "not_perpendicular_to_road"
            elif reference is not None and abs(wrap_to_pi(line.bearing_rad - reference.bearing_rad)) > \
                    self.terminal_track_angle_rad:
                reason = "different_wall_direction"
            elif reference is not None and abs(line.rho_base_m - reference.rho_base_m) > distance_gate:
                reason = "different_wall_distance"
            gap = _terminal_support_gap(right, line)
            detail = dict(rho_base_m=line.rho_base_m, bearing_rad=line.bearing_rad,
                          span_m=line.span_m, inlier_count=line.inlier_count, rms_m=line.rms_m,
                          support_start_xy=line.support_start_xy, support_end_xy=line.support_end_xy,
                          corner_support_gap_m=gap if math.isfinite(gap) else None,
                          rejection_reason=reason, selected=False)
            self._terminal_candidates.append(detail)
            if reason is None:
                # Continuity is the primary choice once a perimeter wall is
                # known. Initial selection favours supported enclosure geometry
                # and long walls, never simply the closest front-facing object.
                score = ((abs(line.rho_base_m - reference.rho_base_m),
                          abs(wrap_to_pi(line.bearing_rad - reference.bearing_rad)))
                         if reference is not None else (gap / line.span_m, -line.span_m))
                candidates.append((score, line, detail))
        if not candidates:
            return None
        _, chosen, detail = min(candidates, key=lambda item: item[0])
        detail["selected"] = True
        self._terminal_reference = chosen
        self._terminal_reference_s = now_s
        return chosen

    def _tick(self, snapshot: WallSnapshot, now_s: float) -> ActionOutcome:
        right = self._pick_right(snapshot)
        if right is None:
            return self._wait(now_s, "right_wall_not_visible")
        terminal = self._pick_terminal(snapshot, right, now_s) if self.terminal_role is not None else None
        self._terminal_visible = terminal is not None
        fields: dict = {"terminal_candidates": self._terminal_candidates,
                        "scan_seq": snapshot.scan_seq, "scan_point_count": snapshot.point_count,
                        "terminal_selection_reason": ("selected" if terminal is not None else
                            "no_compatible_terminal" if self._terminal_candidates else "no_terminal_model")}
        terminal_target = self.terminal_target_m
        terminal_measured = None
        if terminal is not None:
            self.terminal_wall = terminal
            self.terminal_seen_at_s = now_s
            terminal_measured = terminal.rho_base_m
            self._travel_direction = (-self.direction if terminal_measured <
                                     terminal_target - self.terminal_tolerance_m else self.direction)
        # A missing terminal never holds zero or starts an abort timer. Continue
        # the current road-wall direction, including an ongoing recovery; only
        # fresh terminal ranges can brake or reverse it. The prior is retained
        # for association, never used as a current control range.
        command, control = follow_command(
            right, self.target_distance_m,
            self.direction if terminal is not None else self._travel_direction, limits=self.limits,
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

    Route turns hand off when the next moving controller can take over from
    the measured geometry. Standalone turns without a handoff still use two
    fresh stopped scans inside tolerance. No command rate is integrated into
    a virtual angle.
    """

    name = "turn_to_wall_bearing"

    def __init__(self, *, reference_bearing_rad: float, desired_bearing_rad: float,
                 turn_direction: int, limits: WallActionLimits,
                 tracker: WallTracker | None = None,
                 gain: float = TURN_GAIN,
                 max_omega_rad_s: float = TURN_MAX_OMEGA_RAD_S,
                 tolerance_rad: float = TURN_TOLERANCE_RAD,
                 required_frames: int = TURN_STABLE_FRAMES,
                 handoff: Callable[[WallLine], bool] | None = None,
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
        self.handoff = handoff
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
        if self.handoff is not None and self.handoff(wall):
            self._record_zero(now_s)
            return self._finish(True, "dynamic_handoff", **fields)
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
                                  expected_distance_m=self.tracker.last_rho_m)
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
            _, probe_info = normal_move_command(
                wall, self.target_distance_m, self.direction,
                max_speed_m_s=self.max_speed_m_s, tolerance_m=self.tolerance_m)
        except ValueError:
            # A wall exactly alongside has no longitudinal distance component.
            # Actively turn toward the move axis; do not hold zero waiting for an
            # angle-validity gate that no command is trying to satisfy.
            error = self._axis_error(wall)
            omega = self.heading_command(error)
            self._record_nonzero(now_s)
            return ActionOutcome(self.name, False, "aligning_move_axis",
                                 {"command": Twist2D(0.0, omega), "heading_error_rad": error})
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
        fields["heading_within_band"] = abs(heading_error) <= self.heading_band_rad
        if reaching:
            # The distance ends this move. Residual heading is handled by the
            # next coarse turn or the road controller, never by an in-place
            # precision-angle loop at the end of a translation.
            self._record_zero(now_s)
            return self._finish(True, "distance_reached", **fields)
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
        return normal_heading_command(heading_error_rad, self.limits,
                                      band_rad=self.heading_band_rad, gain=self.heading_gain)



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
