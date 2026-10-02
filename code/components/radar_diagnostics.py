#!/usr/bin/env python3
"""Shared diagnostics for D500 rectangle-boundary observability.

These helpers exist so the two standalone tools (``tools/d500_diag.py`` and
``tools/d500_boundary_diag.py``) can report *per-wall* fit quality instead of a
single dominant line over a full 360 deg revolution.

Why a single principal axis is the wrong measure
------------------------------------------------
A rectangular field scanned through 360 deg is composed of four different
straight edges, so its points do not lie on one line.  Comparing the residual
about one principal axis against ``WallLineConfig.max_line_rms_cm`` therefore
always fails, even for a perfect rectangle.  Each edge has to be evaluated on
its own.

The safety net is treated as the real field boundary, not as noise: individual
laser rays may pass through the mesh and hit something further away, so the
fitters here are robust (median/MAD trimming, nearest-mode selection) rather
than assuming every point lies on the wall.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .radar_driver import RadarPoint, RadarScan, RadarMount, scan_points_in_body

__all__ = [
    "PolarBin",
    "LineFit",
    "WallModeFit",
    "aggregate_polar_bins",
    "fit_line_pca",
    "extract_lines",
    "fit_axis_mode",
    "MAD_TO_SIGMA",
]

# Median-absolute-deviation to standard-deviation factor for normal data.
MAD_TO_SIGMA = 1.4826


@dataclass(frozen=True, slots=True)
class PolarBin:
    """One angular bin holding the aggregated range hits across many scans."""

    angle_cw_deg: float
    ranges_cm: tuple[float, ...]

    @property
    def hits(self) -> int:
        return len(self.ranges_cm)

    def quantile_cm(self, fraction: float) -> float | None:
        """Return the ``fraction`` quantile of the aggregated ranges."""

        if not self.ranges_cm:
            return None
        ordered = sorted(self.ranges_cm)
        fraction = min(1.0, max(0.0, float(fraction)))
        index = int(round(fraction * (len(ordered) - 1)))
        return ordered[index]

    def near_quantile_cm(self, fraction: float, min_hits: int) -> float | None:
        """Lower quantile, but only when the bin was hit often enough.

        A single ray slipping through the mesh must not define the boundary, so
        bins with too few hits return ``None``.
        """

        if self.hits < max(1, int(min_hits)):
            return None
        return self.quantile_cm(fraction)


def aggregate_polar_bins(
    scans,
    *,
    bin_deg: float = 2.0,
    min_distance_mm: int = 100,
    max_distance_mm: int = 12000,
    min_confidence: int = 30,
) -> list[PolarBin]:
    """Aggregate several scans into angular bins of range hits.

    Per ray, some hits pass through the net and land far away.  Pooling many
    revolutions per angular bin lets a low quantile recover the stable near
    boundary instead of the far background.
    """

    if bin_deg <= 0.0:
        raise ValueError("bin_deg must be positive")
    if not 0 <= min_distance_mm < max_distance_mm:
        raise ValueError("min_distance_mm must be below max_distance_mm")

    bins: dict[int, list[float]] = {}
    for scan in scans:
        for point in scan.points:
            if not isinstance(point, RadarPoint):
                continue
            if not min_distance_mm <= point.distance_mm <= max_distance_mm:
                continue
            if point.confidence < min_confidence:
                continue
            index = int(math.floor((point.angle_cw_deg + 180.0) / bin_deg))
            bins.setdefault(index, []).append(point.distance_mm / 10.0)

    result = []
    for index in sorted(bins):
        angle = index * bin_deg + bin_deg / 2.0 - 180.0
        result.append(PolarBin(angle, tuple(sorted(bins[index]))))
    return result


@dataclass(frozen=True, slots=True)
class LineFit:
    """A robust straight-line fit in a local 2-D frame (centimetres)."""

    points: int
    angle_deg: float
    span_cm: float
    rms_cm: float
    max_abs_residual_cm: float
    offset_cm: float

    @property
    def usable(self) -> bool:
        return self.points > 0 and self.span_cm > 0.0


def _pca_angle_deg(xs, ys) -> float:
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    sxx = sum((x - mean_x) ** 2 for x in xs)
    syy = sum((y - mean_y) ** 2 for y in ys)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    return 0.5 * math.atan2(2.0 * sxy, sxx - syy)


def _robust_inliers(values, tolerance_cm: float):
    """Trim values whose deviation from the median exceeds the tolerance."""

    ordered = sorted(values)
    median = ordered[len(ordered) // 2]
    residuals = [value - median for value in values]
    ordered_res = sorted(abs(r) for r in residuals)
    mad = ordered_res[len(ordered_res) // 2]
    limit = max(2.5 * MAD_TO_SIGMA * mad, tolerance_cm)
    keep = [i for i, r in enumerate(residuals) if abs(r) <= limit]
    return median, keep


def fit_line_pca(points, *, trim_tolerance_cm: float = 4.0) -> LineFit:
    """Fit one line to ``points`` with median/MAD trimming.

    Returns the fit over the trimmed inliers; ``points`` counts the inliers.
    """

    pts = [(float(x), float(y)) for x, y in points]
    if len(pts) < 2:
        return LineFit(0, 0.0, 0.0, 0.0, 0.0, 0.0)

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    angle = _pca_angle_deg(xs, ys)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    # Signed distance from the line through the centroid, along its normal.
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    normals = [-(x - mean_x) * sin_a + (y - mean_y) * cos_a for x, y in pts]
    median, keep = _robust_inliers(normals, trim_tolerance_cm)
    if len(keep) < 2:
        keep = list(range(len(pts)))
        median = 0.0

    kept = [pts[i] for i in keep]
    xs = [p[0] for p in kept]
    ys = [p[1] for p in kept]
    angle = _pca_angle_deg(xs, ys)
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    normals = [-(x - mean_x) * sin_a + (y - mean_y) * cos_a for x, y in kept]
    rms = math.sqrt(sum(n * n for n in normals) / len(normals))
    max_abs = max(abs(n) for n in normals)
    along = [(x - mean_x) * cos_a + (y - mean_y) * sin_a for x, y in kept]
    span = max(along) - min(along)
    # Offset of the line from the origin along its normal.
    offset = -(mean_x * sin_a) + (mean_y * cos_a)
    return LineFit(len(kept), math.degrees(angle), span, rms, max_abs, offset)


def extract_lines(
    points,
    *,
    ransac_trials: int = 400,
    inlier_gate_cm: float = 4.0,
    min_points: int = 8,
    min_span_cm: float = 30.0,
    max_lines: int = 8,
    seed: int = 12345,
):
    """Extract several straight lines (four for a rectangle) from a scan.

    A simple RANSAC loop: pick the best-supported line, remove its inliers and
    repeat.  Used only for diagnostics, so determinism matters more than speed,
    hence the fixed seed.
    """

    import random

    rng = random.Random(seed)
    remaining = [(float(x), float(y)) for x, y in points]
    fits: list[tuple[LineFit, list[tuple[float, float]]]] = []

    while len(remaining) >= max(2, min_points) and len(fits) < max_lines:
        best = None
        best_inliers: list[tuple[float, float]] = []
        for _ in range(ransac_trials):
            (x1, y1), (x2, y2) = rng.sample(remaining, 2)
            dx, dy = x2 - x1, y2 - y1
            norm = math.hypot(dx, dy)
            if norm < 1e-6:
                continue
            nx, ny = -dy / norm, dx / norm
            inliers = [
                (x, y) for x, y in remaining
                if abs((x - x1) * nx + (y - y1) * ny) <= inlier_gate_cm
            ]
            if best is None or len(inliers) > len(best_inliers):
                best = (nx, ny)
                best_inliers = inliers
        if best is None or len(best_inliers) < min_points:
            break
        fit = fit_line_pca(best_inliers, trim_tolerance_cm=inlier_gate_cm)
        if fit.span_cm < min_span_cm:
            break
        fits.append((fit, best_inliers))
        keep = set(best_inliers)
        remaining = [p for p in remaining if p not in keep]

    return fits


@dataclass(frozen=True, slots=True)
class WallModeFit:
    """A dense mode found along one wall-normal axis."""

    points: int
    coordinate_cm: float
    rms_cm: float
    max_abs_residual_cm: float
    span_cm: float
    candidates: int


def fit_axis_mode(
    axis_values,
    *,
    expected_cm: float,
    span_values=None,
    tolerance_cm: float = 7.5,
    min_points: int = 12,
    min_span_cm: float = 50.0,
    max_rms_cm: float = 6.0,
    max_spread_cm: float | None = None,
) -> WallModeFit:
    """Find the dense normal-coordinate mode nearest ``expected_cm``.

    Safety-net returns are a mix of real boundary echoes and rays that passed
    through the mesh.  Averaging the whole association band would be dragged
    away by the far group, so this selects a locally dense cluster of points
    and prefers the one closest to the known wall coordinate.

    A cluster must also be *tight*: a handful of real boundary hits sitting
    next to a long smear of through-mesh returns can form a large but diffuse
    group, and accepting it would report a wall coordinate tens of centimetres
    away.  ``max_rms_cm`` and ``max_spread_cm`` reject that, so the caller sees
    NO_OBSERVATION instead of a wrong wall.

    ``span_values`` (the along-wall coordinate of each point) is used to reject
    a cluster that comes from a pole or other small object.
    """

    values = [float(v) for v in axis_values]
    spans = None if span_values is None else [float(v) for v in span_values]
    if len(values) != len(axis_values):
        raise ValueError("axis_values must be numeric")
    if spans is not None and len(spans) != len(values):
        raise ValueError("span_values must match axis_values length")
    if not values:
        return WallModeFit(0, expected_cm, 0.0, 0.0, 0.0, 0)
    if tolerance_cm <= 0.0 or min_points < 2:
        raise ValueError("tolerance_cm must be positive and min_points at least 2")

    window = max(float(max_spread_cm) if max_spread_cm else tolerance_cm, 1e-6)
    order = sorted(range(len(values)), key=lambda i: values[i])
    sorted_values = [values[i] for i in order]

    candidates = []
    start = 0
    while start < len(sorted_values):
        end = start
        while (
            end + 1 < len(sorted_values)
            and sorted_values[end + 1] - sorted_values[start] <= window
        ):
            end += 1
        cluster = sorted_values[start : end + 1]
        if len(cluster) >= max(2, int(min_points)):
            centre = cluster[len(cluster) // 2]
            candidates.append((centre, start, end))
        start += 1

    if not candidates:
        return WallModeFit(0, expected_cm, 0.0, 0.0, 0.0, 0)

    # Prefer the cluster nearest the expected wall, then the denser one.
    candidates.sort(key=lambda c: (abs(c[0] - expected_cm), -(c[2] - c[1])))
    centre, start, end = candidates[0]
    indices = order[start : end + 1]
    cluster = [values[i] for i in indices]

    residuals = [v - centre for v in cluster]
    rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
    max_abs = max(abs(r) for r in residuals)
    spread = max(cluster) - min(cluster)
    if spans is None:
        span = 0.0
    else:
        along = [spans[i] for i in indices]
        span = max(along) - min(along)

    # Too diffuse to be a boundary: report no observation rather than a wrong
    # wall coordinate.
    if rms > max_rms_cm or spread > window + 1e-9:
        return WallModeFit(0, expected_cm, 0.0, 0.0, span, len(candidates))

    if span and span < min_span_cm:
        # Too short along the wall: likely a pole rather than a boundary.
        return WallModeFit(0, expected_cm, 0.0, 0.0, span, len(candidates))

    return WallModeFit(len(cluster), centre, rms, max_abs, span, len(candidates))
