"""Raw D500 scan -> timestamped local wall lines. Observe only; never drives.

This component owns exactly one D500 serial reader and one fitting worker. It
does not know about the robot's global position, the task plan, PWM or the
motors, and it deliberately does not construct ``D500RadarComponent`` (that one
pulls in scan-to-scan ICP, wall-map fusion and a global point map).

Coordinate contract (``AGENTS.md``): ``base_link`` is the midpoint of the two
driven wheel axes, ``+X`` forward, ``+Y`` left, metres and radians. A wall line
is ``n . p = rho`` with a unit normal pointing *toward the wall* and
``rho > 0``; ``rho_base_m`` is therefore the perpendicular drive-axle-centre
distance to the wall, and the observed finite support segment is kept
separately so a gap in the enclosure never becomes imaginary wall length.

The 2-D total-least-squares fit is a closed-form 2x2 symmetric eigenproblem, so
this module has no numpy or scikit-image dependency. The RANSAC + TLS + real
support-segment structure follows the package reference math and the cited
upstream projects (see ``docs/12_REFERENCES_AND_LICENSES.md``); no upstream
source was copied.
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import random
import threading
import time
from typing import Callable, Iterable, Sequence

from components.radar_driver import (
    DEFAULT_D500_BAUDRATE,
    DEFAULT_D500_PORT,
    D500_TIMESTAMP_MODULUS_MS,
    D500SerialDriver,
    RadarScan,
    RadarScanAssembler,
)
from components.sensor_clock import DeviceClockMapper

LOG = logging.getLogger("wall-observer")

# Fitting defaults. These are software starting values from the implementation
# package, not measured field numbers; keep them here rather than spreading a
# pile of command-line switches.
RESIDUAL_THRESHOLD_M = 0.03
MIN_SPAN_M = 0.80
MIN_POINTS = 10
MAX_RMS_M = 0.03
MAX_SUPPORT_GAP_M = 0.60
MAX_MODELS = 8
MAX_TRIALS = 128
MAX_POINTS = 600
MIN_MODEL_BASELINE_M = 0.10
MAX_AGE_S = 0.40

# Candidate association gates. The wide gate seeds a selection; the narrow gate
# keeps one wall across frames so the controller cannot hop between two
# parallel structures.
ANGLE_GATE_RAD = math.radians(35.0)
DISTANCE_GATE_M = 0.65
TRACK_ANGLE_GATE_RAD = math.radians(25.0)
TRACK_DISTANCE_GATE_M = 0.35
# A right wall must be roughly alongside the chassis, and a terminal wall
# roughly across it.
RIGHT_MIN_DOWNWARD_NY = -0.5
TERMINAL_MIN_NX = 0.5
ORTHOGONAL_TOLERANCE_RAD = math.radians(12.0)
MAX_SUPPORT_EXTENSION_M = 0.80

# Sanity limits for a single valid point.
MIN_RANGE_M = 0.05
MAX_RANGE_M = 20.0


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi)``; reject non-finite input."""
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True, slots=True)
class WallLine:
    """One observed wall segment in the instantaneous ``base_link`` frame."""

    normal_xy: tuple[float, float]
    rho_base_m: float
    support_start_xy: tuple[float, float]
    support_end_xy: tuple[float, float]
    inlier_count: int
    span_m: float
    rms_m: float

    def __post_init__(self):
        values = (*self.normal_xy, self.rho_base_m, *self.support_start_xy,
                  *self.support_end_xy, self.span_m, self.rms_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("wall line fields must be finite")
        if self.rho_base_m <= 0 or self.rms_m < 0 or self.span_m < 0 or self.inlier_count < 0:
            raise ValueError("invalid wall line distance or quality")
        if not math.isclose(math.hypot(*self.normal_xy), 1.0, abs_tol=1e-6):
            raise ValueError("wall line normal must be unit length")

    @property
    def bearing_rad(self) -> float:
        """Body-frame direction of the normal, i.e. where the wall sits."""
        return math.atan2(self.normal_xy[1], self.normal_xy[0])

    def distance_to_segment(self, point_xy: tuple[float, float]) -> float:
        ax, ay = self.support_start_xy
        bx, by = self.support_end_xy
        dx, dy = bx - ax, by - ay
        denominator = dx * dx + dy * dy
        if denominator <= 1e-18:
            return math.hypot(point_xy[0] - ax, point_xy[1] - ay)
        u = ((point_xy[0] - ax) * dx + (point_xy[1] - ay) * dy) / denominator
        u = min(1.0, max(0.0, u))
        return math.hypot(point_xy[0] - (ax + u * dx), point_xy[1] - (ay + u * dy))

    def covers_along(self, point_xy: tuple[float, float], *,
                     max_extension_m: float) -> bool:
        """Whether the corner lies inside this support's span, within extension.

        The question a corner has to answer is "is the point beyond the end of
        the wall I actually observed?", measured along the wall's own direction.
        Measuring the perpendicular distance to the segment instead would reject
        a legitimate corner: the other wall's line sits a whole wall-distance
        away perpendicular, so every real corner would fail.
        """
        ax, ay = self.support_start_xy
        dx = self.support_end_xy[0] - ax
        dy = self.support_end_xy[1] - ay
        length = math.hypot(dx, dy)
        if length <= 1e-12:
            return math.hypot(point_xy[0] - ax, point_xy[1] - ay) <= max_extension_m
        tx, ty = dx / length, dy / length
        along = (point_xy[0] - ax) * tx + (point_xy[1] - ay) * ty
        return -max_extension_m <= along <= length + max_extension_m


@dataclass(frozen=True, slots=True)
class WallSnapshot:
    scan_seq: int
    scan_started_s: float
    scan_completed_s: float
    observed_at_s: float
    lines: tuple[WallLine, ...]
    point_count: int = 0
    error: str | None = None

    def is_fresh(self, now_s: float, *, max_age_s: float = MAX_AGE_S) -> bool:
        """Whether this scan is a current, successful measurement.

        "Fresh" is deliberately about time and acquisition success only. A
        current scan that happens to contain no usable line is still fresh: the
        caller's wall *selection* then reports "not visible", which is a
        different condition from "the observation is too old to use".
        """
        if not math.isfinite(now_s) or not math.isfinite(self.observed_at_s):
            return False
        if self.error is not None:
            return False
        return 0.0 <= now_s - self.observed_at_s <= max_age_s


def fit_line_tls(points: Sequence[tuple[float, float]]):
    """Closed-form 2-D orthogonal regression.

    Returns ``(normal, rho, start, end, rms)`` with a unit normal pointing away
    from the origin and ``rho > 0``, or ``None`` when the points are fewer than
    three, coincident, or degenerate.
    """
    data = [(float(x), float(y)) for x, y in points]
    if len(data) < 3:
        return None
    count = len(data)
    mean_x = sum(point[0] for point in data) / count
    mean_y = sum(point[1] for point in data) / count
    sxx = sxy = syy = 0.0
    for x, y in data:
        dx, dy = x - mean_x, y - mean_y
        sxx += dx * dx
        sxy += dx * dy
        syy += dy * dy
    trace = sxx + syy
    if trace <= 1e-12:
        # Every point is at the same place: no direction exists at all.
        return None
    root = math.sqrt(max(0.0, (sxx - syy) ** 2 + 4.0 * sxy * sxy))
    small = 0.5 * (trace - root)
    large = 0.5 * (trace + root)
    if large <= 1e-12:
        return None
    # Only a genuinely coincident set is degenerate. Requiring a minimum
    # *eigenvalue ratio* instead would reject exactly collinear points, whose
    # perpendicular spread is legitimately zero -- and a perfect line is the
    # least ambiguous wall there is. The caller's RMS and span gates judge
    # quality; this function only has to return a well-defined direction.
    if small <= 0.0:
        small = 0.0
    # The eigenvector of the SMALL eigenvalue is the normal direction.
    if abs(sxy) > 1e-12:
        normal = (small - syy, sxy)
    else:
        normal = (1.0, 0.0) if sxx <= syy else (0.0, 1.0)
    norm = math.hypot(*normal)
    if norm <= 1e-12:
        return None
    normal = (normal[0] / norm, normal[1] / norm)
    # The contract is a normal pointing *toward* the wall, so pick the unit
    # normal that is positive along the support's own direction from the origin.
    # Orienting by the sign of ``rho`` alone would point a wall on the robot's
    # right at +x instead of -y.
    if normal[0] * mean_x + normal[1] * mean_y < 0.0:
        normal = (-normal[0], -normal[1])
    rho = normal[0] * mean_x + normal[1] * mean_y
    if rho <= 1e-9:
        return None
    tangent = (-normal[1], normal[0])
    projections = [(x - mean_x) * tangent[0] + (y - mean_y) * tangent[1] for x, y in data]
    lo, hi = min(projections), max(projections)
    start = (mean_x + lo * tangent[0], mean_y + lo * tangent[1])
    end = (mean_x + hi * tangent[0], mean_y + hi * tangent[1])
    residual_sq = 0.0
    for x, y in data:
        residual = x * normal[0] + y * normal[1] - rho
        residual_sq += residual * residual
    return normal, rho, start, end, math.sqrt(residual_sq / count)


def _largest_span_group(points: Sequence[tuple[float, float]],
                        normal: tuple[float, float], *, max_gap_m: float):
    """Split inliers at real gaps; return the longest observed support segment."""
    tangent = (-normal[1], normal[0])
    ordered = sorted(points, key=lambda point: point[0] * tangent[0] + point[1] * tangent[1])
    groups, current = [], [ordered[0]]
    for previous, point in zip(ordered, ordered[1:]):
        gap = (point[0] - previous[0]) * tangent[0] + (point[1] - previous[1]) * tangent[1]
        if gap > max_gap_m:
            groups.append(current)
            current = []
        current.append(point)
    groups.append(current)
    return max(groups, key=len)


def extract_lines(points: Sequence[tuple[float, float]], *, rng: random.Random,
                  residual_threshold_m: float = RESIDUAL_THRESHOLD_M,
                  max_trials: int = MAX_TRIALS, max_models: int = MAX_MODELS,
                  min_points: int = MIN_POINTS, min_span_m: float = MIN_SPAN_M,
                  max_rms_m: float = MAX_RMS_M,
                  max_support_gap_m: float = MAX_SUPPORT_GAP_M) -> tuple[WallLine, ...]:
    """Multi-model RANSAC + TLS with real support segments.

    The caller owns ``rng`` and reuses it across scans. This function never
    decides which line is the field's right/front/rear wall.
    """
    if rng is None:
        raise ValueError("a caller-owned random generator is required")
    for name, value in (("residual_threshold_m", residual_threshold_m),
                        ("min_span_m", min_span_m), ("max_rms_m", max_rms_m),
                        ("max_support_gap_m", max_support_gap_m)):
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be finite and positive")
    if min_points < 3 or max_trials < 1 or max_models < 1:
        raise ValueError("invalid RANSAC counts")

    remaining = [(float(x), float(y)) for x, y in points]
    consumed: set[int] = set()
    output: list[WallLine] = []
    for _ in range(max_models):
        if len(remaining) < min_points:
            break
        best_indices = None
        best_score = None
        for _ in range(max_trials):
            first, second = rng.sample(range(len(remaining)), 2)
            ax, ay = remaining[first]
            bx, by = remaining[second]
            dx, dy = bx - ax, by - ay
            length = math.hypot(dx, dy)
            if length < MIN_MODEL_BASELINE_M:
                continue
            nx, ny = -dy / length, dx / length
            offset = nx * ax + ny * ay
            indices, residual_sq = [], 0.0
            for index, (x, y) in enumerate(remaining):
                residual = x * nx + y * ny - offset
                if abs(residual) <= residual_threshold_m:
                    indices.append(index)
                    residual_sq += residual * residual
            if len(indices) < min_points:
                continue
            score = (len(indices), -residual_sq / len(indices))
            if best_score is None or score > best_score:
                best_score, best_indices = score, indices
        if best_indices is None:
            break
        # Refit an ever-larger inlier set; keep the last usable fit.
        fitted = None
        for _ in range(2):
            candidate = fit_line_tls([remaining[index] for index in best_indices])
            if candidate is None:
                break
            nx, ny, rho = candidate[0][0], candidate[0][1], candidate[1]
            refined = [index for index, (x, y) in enumerate(remaining)
                       if abs(x * nx + y * ny - rho) <= residual_threshold_m]
            if len(refined) < min_points:
                break
            best_indices, fitted = refined, candidate
        consumed |= set(best_indices)
        remaining = [point for index, point in enumerate(remaining) if index not in consumed]
        if fitted is None:
            continue
        normal = fitted[0]
        # Re-select this model's inliers from the untouched point list so a real
        # gap can split the support without losing the model's own evidence.
        inliers = [(float(x), float(y)) for x, y in points
                   if abs(float(x) * normal[0] + float(y) * normal[1] - fitted[1])
                   <= residual_threshold_m]
        if len(inliers) < min_points:
            continue
        group = _largest_span_group(inliers, normal, max_gap_m=max_support_gap_m)
        if len(group) < min_points:
            continue
        fitted = fit_line_tls(group)
        if fitted is None:
            continue
        normal, rho, start, end, rms = fitted
        span = math.dist(start, end)
        if span < min_span_m or rms > max_rms_m:
            continue
        output.append(WallLine(normal_xy=(normal[0], normal[1]), rho_base_m=rho,
                               support_start_xy=start, support_end_xy=end,
                               inlier_count=len(group), span_m=span, rms_m=rms))
    return tuple(output)


def scan_points_base(scan: RadarScan, mount) -> list[tuple[float, float]]:
    """Convert one sensor-frame revolution to base-frame metres, exactly once.

    ``RadarPoint.sensor_xy_cm()`` already encodes the clockwise-angle sign
    convention, so this only applies the measured mount rotation and
    translation. Never call ``scan_points_in_body()`` here as well: that would
    add the mount offset a second time.
    """
    yaw_cw_deg = float(getattr(mount, "yaw_cw_deg", 0.0))
    yaw_rad = -math.radians(yaw_cw_deg)
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    x_m = float(getattr(mount, "x_forward_cm", 0.0)) / 100.0
    y_m = float(getattr(mount, "y_left_cm", 0.0)) / 100.0
    points = []
    for point in scan.points:
        x_cm, y_cm = point.sensor_xy_cm()
        x_l, y_l = x_cm / 100.0, y_cm / 100.0
        if not (math.isfinite(x_l) and math.isfinite(y_l)):
            continue
        if not MIN_RANGE_M <= math.hypot(x_l, y_l) <= MAX_RANGE_M:
            continue
        points.append((cos_yaw * x_l - sin_yaw * y_l + x_m,
                       sin_yaw * x_l + cos_yaw * y_l + y_m))
    return points


def _downsample(points: Sequence[tuple[float, float]], limit: int):
    if limit < 1 or len(points) <= limit:
        return list(points)
    step = len(points) / limit
    return [points[int(index * step)] for index in range(limit)]


def select_candidate(lines: Sequence[WallLine], bearing_rad: float, *,
                     expected_distance_m: float | None = None,
                     angle_gate_rad: float = ANGLE_GATE_RAD,
                     distance_gate_m: float = DISTANCE_GATE_M) -> WallLine | None:
    """Pick the line closest in bearing (and distance) to an expectation.

    This is a single-snapshot helper for seeding a selection. Continuity across
    frames is ``WallTracker``'s job; do not call this every frame with a fixed
    body-relative sector.
    """
    if not math.isfinite(angle_gate_rad) or angle_gate_rad <= 0:
        raise ValueError("angle_gate_rad must be finite and positive")
    if not math.isfinite(distance_gate_m) or distance_gate_m <= 0:
        raise ValueError("distance_gate_m must be finite and positive")
    accepted = []
    for line in lines:
        delta_angle = abs(wrap_to_pi(line.bearing_rad - bearing_rad))
        if delta_angle > angle_gate_rad:
            continue
        delta_distance = 0.0 if expected_distance_m is None else abs(line.rho_base_m - expected_distance_m)
        if expected_distance_m is not None and delta_distance > distance_gate_m:
            continue
        score = (delta_angle / angle_gate_rad + delta_distance / distance_gate_m,
                 -line.span_m, -line.inlier_count)
        accepted.append((score, line))
    return min(accepted, key=lambda item: item[0])[1] if accepted else None


class WallTracker:
    """Keeps one reference wall across frames instead of hopping structures.

    Two different distances matter here and must never be mixed:

    * ``reference_rho_m`` is the distance at which this physical wall was first
      identified. It is the wall's *identity*, and control targets such as
      ``d0 + 0.43`` must never be written into it.
    * ``last_rho_m`` is the most recent *measured* distance of that same wall.
      It, not the control target, is what the next frame's association is
      centred on. Centring association on a target the chassis is driving toward
      makes the tracker accept a different parallel structure that happens to sit
      at the target distance, which then reads as "already arrived".

    ``track()`` never re-picks a different body-relative sector; it follows the
    stored bearing and only re-seeds with a wide gate when the narrow continuity
    gate fails.
    """

    def __init__(self, *, angle_gate_rad: float = TRACK_ANGLE_GATE_RAD,
                 distance_gate_m: float = TRACK_DISTANCE_GATE_M,
                 seed_angle_gate_rad: float = ANGLE_GATE_RAD,
                 seed_distance_gate_m: float = DISTANCE_GATE_M) -> None:
        self.angle_gate_rad = angle_gate_rad
        self.distance_gate_m = distance_gate_m
        self.seed_angle_gate_rad = seed_angle_gate_rad
        self.seed_distance_gate_m = seed_distance_gate_m
        self.wall: WallLine | None = None
        self.reference_bearing_rad: float | None = None
        self.reference_rho_m: float | None = None
        self.last_rho_m: float | None = None
        self.inlier_count = 0

    def set_reference(self, wall: WallLine) -> WallLine:
        """Lock a new reference wall (a corner turn, or an explicit re-acquire)."""
        self.wall = wall
        self.reference_bearing_rad = wall.bearing_rad
        self.reference_rho_m = wall.rho_base_m
        self.last_rho_m = wall.rho_base_m
        self.inlier_count = 0
        return wall

    def clear(self) -> None:
        self.wall = None
        self.reference_bearing_rad = None
        self.reference_rho_m = None
        self.last_rho_m = None
        self.inlier_count = 0

    def track(self, lines: Sequence[WallLine], *, expected_bearing_rad: float,
              expected_distance_m: float | None = None,
              seed_only: bool = False) -> WallLine | None:
        """Follow the reference wall; return ``None`` when it is not visible.

        ``expected_bearing_rad`` is where the *reference* wall should now be in
        the body frame, normally the stored bearing plus the measured body
        rotation. ``expected_distance_m`` should be that wall's most recent
        measured distance (or a prediction of it), never a control target. A
        missing result means the caller must hold zero speed, never that it
        should integrate a command into a virtual angle.
        """
        if not math.isfinite(expected_bearing_rad):
            raise ValueError("expected bearing must be finite")
        narrow = None
        if not seed_only:
            narrow = select_candidate(
                lines, expected_bearing_rad,
                expected_distance_m=expected_distance_m,
                angle_gate_rad=self.angle_gate_rad,
                distance_gate_m=self.distance_gate_m)
        if narrow is None:
            narrow = select_candidate(
                lines, expected_bearing_rad,
                expected_distance_m=expected_distance_m,
                angle_gate_rad=self.seed_angle_gate_rad,
                distance_gate_m=self.seed_distance_gate_m)
        if narrow is None:
            return None
        # Continue the *same* reference: advance its measured distance and
        # bearing, but keep reference_rho_m as the identity distance recorded
        # when this wall was first locked.
        self.wall = narrow
        self.reference_bearing_rad = narrow.bearing_rad
        self.last_rho_m = narrow.rho_base_m
        return narrow


def supported_corner(right: WallLine, terminal: WallLine, *, rear: bool = False,
                     max_angle_error_rad: float = ORTHOGONAL_TOLERANCE_RAD,
                     max_support_extension_m: float = MAX_SUPPORT_EXTENSION_M):
    """Intersect two approximately orthogonal walls, with real support checks.

    Returns the corner ``(x, y)`` in the body frame, or ``None`` when the walls
    are not orthogonal, are parallel, meet in the wrong quadrant, or sit far
    outside either observed support segment. Two infinite lines crossing is not
    a verified corner.
    """
    if not math.isfinite(max_angle_error_rad) or max_angle_error_rad <= 0:
        raise ValueError("max_angle_error_rad must be finite and positive")
    if abs(right.normal_xy[0] * terminal.normal_xy[0]
           + right.normal_xy[1] * terminal.normal_xy[1]) > math.sin(max_angle_error_rad):
        return None
    determinant = (right.normal_xy[0] * terminal.normal_xy[1]
                   - right.normal_xy[1] * terminal.normal_xy[0])
    if abs(determinant) < 1e-9:
        return None
    # Solve n_right . p = rho_right and n_terminal . p = rho_terminal.
    corner_x = (right.rho_base_m * terminal.normal_xy[1]
                - right.normal_xy[1] * terminal.rho_base_m) / determinant
    corner_y = (right.normal_xy[0] * terminal.rho_base_m
                - right.rho_base_m * terminal.normal_xy[0]) / determinant
    if corner_y >= 0.0:
        return None
    if corner_x >= 0.0 if rear else corner_x <= 0.0:
        return None
    for line in (right, terminal):
        if not line.covers_along((corner_x, corner_y),
                                 max_extension_m=max_support_extension_m):
            return None
    return corner_x, corner_y


class WallObserver:
    """Latest complete D500 revolution -> immutable :class:`WallSnapshot`.

    ``start()`` opens the serial port and the fitting worker; constructing the
    object opens nothing. ``latest()`` only reads the published snapshot, so a
    control tick never blocks on the UART or on a fit.
    """

    def __init__(self, *, port: str = DEFAULT_D500_PORT, baudrate: int = DEFAULT_D500_BAUDRATE,
                 mount=None, clock: Callable[[], float] = time.monotonic,
                 max_age_s: float = MAX_AGE_S, seed: int | None = None,
                 seed_source: Callable[[], int] | None = None,
                 assembler: RadarScanAssembler | None = None,
                 serial_driver=None, driver_factory=None) -> None:
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError("max_age_s must be finite and positive")
        self.port = port
        self.baudrate = baudrate
        self.mount = mount
        self.clock = clock
        self.max_age_s = max_age_s
        self._assembler = assembler or RadarScanAssembler()
        self._clock_mapper = DeviceClockMapper(modulus_ms=D500_TIMESTAMP_MODULUS_MS)
        # Production owns one RNG for the whole run. ``seed_source`` re-seeds per
        # scan and exists so a simulated or replayed sequence is reproducible;
        # the live path never uses it.
        self._seed_source = seed_source
        self._rng = random.Random(seed if seed_source is None else seed_source())
        self._lock = threading.Lock()
        self._wake = threading.Condition(self._lock)
        self._pending: tuple[int, RadarScan, float, float, float] | None = None
        self._snapshot: WallSnapshot | None = None
        self._revolutions = 0
        self._seq = 0
        self._scan_seq = 0
        self._packet_started_s: float | None = None
        self._packet_completed_s: float | None = None
        self._worker: threading.Thread | None = None
        self._running = False
        self._closed = False
        self._error: str | None = None
        self._driver_factory = driver_factory or (
            lambda on_packet: D500SerialDriver(on_packet=on_packet, port=self.port,
                                               baudrate=self.baudrate,
                                               on_connected=self._on_connected,
                                               on_disconnected=self._on_disconnected))
        self._serial_driver = serial_driver

    # ---- lifecycle -------------------------------------------------------
    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def revolution_count(self) -> int:
        return self._revolutions

    def start(self) -> None:
        if self._closed:
            raise RuntimeError("WallObserver is closed; build a new one for the next run")
        if self._running:
            return
        with self._lock:
            self._pending = None
            self._snapshot = None
            self._revolutions = 0
            self._scan_seq = 0
            self._seq = 0
            self._packet_started_s = None
            self._packet_completed_s = None
            self._error = None
        self._clock_mapper.reset()
        self._assembler.reset()
        if self._serial_driver is None:
            self._serial_driver = self._driver_factory(self._on_packet)
        self._running = True
        self._worker = threading.Thread(target=self._fit_loop, name="wall-fit", daemon=True)
        self._worker.start()
        self._serial_driver.start()

    def close(self, *, timeout_s: float = 2.0) -> None:
        self._closed = True
        self._running = False
        with self._lock:
            self._wake.notify_all()
        if self._serial_driver is not None:
            try:
                self._serial_driver.close()
            except Exception:
                LOG.exception("D500 serial close failed")
        worker, self._worker = self._worker, None
        if worker is not None:
            worker.join(timeout=timeout_s)

    # ---- publisher side (serial reader thread) ---------------------------
    def _on_connected(self) -> None:
        with self._lock:
            self._assembler.reset()
            self._pending = None
            self._snapshot = None
            self._packet_started_s = None
            self._packet_completed_s = None
            self._error = None

    def _on_disconnected(self, _error=None) -> None:
        self._on_connected()

    def _on_packet(self, packet) -> None:
        now_s = float(self.clock())
        with self._lock:
            if self._packet_started_s is None:
                self._packet_started_s = now_s
            started_s = self._packet_started_s
        try:
            scans = self._assembler.feed(packet)
        except Exception as exc:  # pragma: no cover - driver callback guard
            with self._lock:
                self._error = f"scan assembly failed: {exc}"
            return
        with self._lock:
            self._packet_started_s = now_s
            self._packet_completed_s = now_s
            for scan in scans:
                self._seq += 1
                measured_s = min(
                    self._clock_mapper.map_milliseconds(scan.timestamp_ms, now_s), now_s)
                # The host measurement time is the midpoint of this revolution's
                # own packet boundaries, never the moment the fit finished.
                self._pending = (self._seq, scan, started_s, now_s,
                                 min(measured_s, 0.5 * (started_s + now_s)))
            self._wake.notify_all()

    # ---- consumer side ---------------------------------------------------
    def publish_points(self, points: Sequence[tuple[float, float]], *,
                       observed_at_s: float | None = None,
                       scan_seq: int | None = None) -> WallSnapshot:
        """Fit and publish a snapshot from raw base-frame points.

        Used by the offline tests and the scan-replay tools; the live path goes
        through ``_on_packet`` instead. Supplies the same monotone-sequence
        contract as a real revolution.
        """
        now_s = float(self.clock())
        measured_s = now_s if observed_at_s is None else float(observed_at_s)
        with self._lock:
            if scan_seq is None:
                # A published scan is a new revolution and must carry a strictly
                # increasing sequence, or nothing downstream can tell a fresh
                # observation from a re-read of the previous one.
                self._seq += 1
                self._scan_seq += 1
                seq = self._seq
            else:
                seq = int(scan_seq)
        snapshot = self._fit(seq, points, measured_s, now_s, measured_s)
        with self._lock:
            self._snapshot = snapshot
        return snapshot

    def latest(self) -> WallSnapshot | None:
        with self._lock:
            return self._snapshot

    def fresh(self, *, now_s: float | None = None, max_age_s: float | None = None) -> WallSnapshot | None:
        snapshot = self.latest()
        if snapshot is None:
            return None
        now = float(self.clock()) if now_s is None else float(now_s)
        age_limit = self.max_age_s if max_age_s is None else float(max_age_s)
        return snapshot if snapshot.is_fresh(now, max_age_s=age_limit) else None

    def _fit_loop(self) -> None:
        while True:
            with self._lock:
                while self._running and self._pending is None:
                    self._wake.wait(0.2)
                if not self._running:
                    return
                seq, scan, started_s, completed_s, measured_s = self._pending
                self._pending = None
            try:
                points = scan_points_base(scan, self.mount) if self.mount is not None else \
                    [(x / 100.0, y / 100.0) for x, y in (point.sensor_xy_cm() for point in scan.points)]
                snapshot = self._fit(seq, points, started_s, completed_s, measured_s)
            except Exception as exc:  # pragma: no cover - defensive
                LOG.exception("wall fit failed")
                snapshot = WallSnapshot(scan_seq=seq, scan_started_s=started_s,
                                        scan_completed_s=completed_s,
                                        observed_at_s=measured_s, lines=(),
                                        error=f"fit failed: {exc}")
            with self._lock:
                if self._snapshot is None or seq >= self._snapshot.scan_seq:
                    self._snapshot = snapshot
                self._revolutions += 1

    def _fit(self, seq: int, points: Sequence[tuple[float, float]],
             started_s: float, completed_s: float, measured_s: float) -> WallSnapshot:
        clean = [(float(x), float(y)) for x, y in points
                 if math.isfinite(x) and math.isfinite(y)
                 and MIN_RANGE_M <= math.hypot(x, y) <= MAX_RANGE_M]
        sampled = _downsample(clean, MAX_POINTS)
        error = None
        lines: tuple[WallLine, ...] = ()
        try:
            if self._seed_source is not None:
                self._rng = random.Random(self._seed_source())
            lines = extract_lines(sampled, rng=self._rng)
        except Exception as exc:
            error = f"extraction failed: {exc}"
            LOG.exception("wall extraction failed")
        snapshot = WallSnapshot(scan_seq=seq, scan_started_s=started_s,
                                scan_completed_s=completed_s, observed_at_s=measured_s,
                                lines=lines, point_count=len(clean), error=error)
        LOG.debug("wall_scan seq=%d seq_started=%.3f seq_completed=%.3f age=%.3f points=%d "
                  "candidates=%d error=%s", seq, started_s, completed_s,
                  max(0.0, completed_s - measured_s), len(clean), len(lines), error)
        for line in lines:
            LOG.debug("wall_candidate normal=(%.3f,%.3f) rho_base=%.3f span=%.3f rms=%.3f inliers=%d",
                      line.normal_xy[0], line.normal_xy[1], line.rho_base_m,
                      line.span_m, line.rms_m, line.inlier_count)
        return snapshot

