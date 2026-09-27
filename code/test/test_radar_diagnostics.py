"""Hardware-free tests for the D500 rectangle-boundary diagnostics."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_diagnostics import (  # noqa: E402
    aggregate_polar_bins,
    extract_lines,
    fit_axis_mode,
    fit_line_pca,
)
from components.radar_driver import RadarPoint, RadarScan  # noqa: E402


def make_scan(points_xy):
    """Build a RadarScan from body-frame (x_cm, y_cm) points."""

    radar_points = []
    for x_cm, y_cm in points_xy:
        distance_cm = math.hypot(x_cm, y_cm)
        # sensor_xy_cm() uses (d*cos(a), -d*sin(a)), so invert that.
        angle = math.degrees(math.atan2(-y_cm, x_cm))
        radar_points.append(
            RadarPoint(angle_cw_deg=angle, distance_mm=int(distance_cm * 10),
                       confidence=200)
        )
    return RadarScan(points=tuple(radar_points), timestamp_ms=0,
                     rotation_speed_deg_s=0)


def rectangle_points(half_cm=500.0, step_cm=4.0):
    """Points on the four edges of a square field centred on the origin."""

    points = []
    span = half_cm
    n = int(2 * span / step_cm)
    for i in range(n + 1):
        t = -span + i * step_cm
        points.append((-span, t))   # back   (x = -half)
        points.append((span, t))    # front  (x = +half)
        points.append((t, -span))   # right  (y = -half)
        points.append((t, span))    # left   (y = +half)
    return points


class RectangleLineExtractionTests(unittest.TestCase):
    """A normal rectangle must yield its four edges, not one principal axis."""

    def test_extracts_two_orthogonal_families(self) -> None:
        points = rectangle_points()
        fits = extract_lines(points, min_points=10, min_span_cm=100.0,
                             max_lines=6)
        self.assertGreaterEqual(len(fits), 2)

        # Every long edge of the square is axis-aligned: x = +-half or y = +-half.
        angles = [abs(((fit.angle_deg % 180.0) + 90.0) % 180.0 - 90.0)
                  for fit, _ in fits]
        axis_aligned = [a for a in angles if a < 3.0 or abs(a - 90.0) < 3.0]
        self.assertGreaterEqual(
            len(axis_aligned), 2,
            "expected at least two axis-aligned edges, got angles %r" % angles,
        )

    def test_single_principal_axis_is_not_used_as_a_wall_measure(self) -> None:
        """Regression: one global PCA over 360 deg must not look like a wall.

        A rectangle scanned all round has a huge residual about its single
        principal axis, which is exactly why the old diagnostic wrongly
        concluded "no clean wall".  This test pins that the per-line fit is
        small while the whole-cloud fit is not.
        """

        points = rectangle_points()
        whole = fit_line_pca(points, trim_tolerance_cm=1e9)
        self.assertGreater(whole.rms_cm, 100.0)

        fits = extract_lines(points, min_points=10, min_span_cm=100.0,
                             max_lines=4)
        self.assertTrue(fits)
        for fit, _ in fits:
            self.assertLess(fit.rms_cm, 1.0)


class SafetyNetRobustnessTests(unittest.TestCase):
    """Mesh returns mix real boundary echoes with long through-holes."""

    def test_dense_wall_with_through_outliers(self) -> None:
        # Wall at x = -200 cm: 80% near it, 20% pass through to 50..250 cm beyond.
        values = []
        spans = []
        for i in range(100):
            spans.append(float(i))
            if i < 80:
                values.append(-200.0 + (i % 7 - 3) * 1.0)
            else:
                values.append(-200.0 + 50.0 + (i % 200))
        fit = fit_axis_mode(values, expected_cm=-200.0, span_values=spans,
                            min_points=15, min_span_cm=0.0)
        self.assertGreater(fit.points, 0)
        self.assertLess(abs(fit.coordinate_cm - (-200.0)), 5.0)
        self.assertLess(fit.rms_cm, 5.0)

    def test_sparse_wall_with_many_outliers_reports_no_observation(self) -> None:
        """With too few real boundary hits the fit must refuse, not guess."""

        values = [-200.0 + (i % 5 - 2) * 1.0 for i in range(6)]
        values += [-100.0 + i for i in range(60)]
        spans = [float(i) for i in range(len(values))]
        fit = fit_axis_mode(values, expected_cm=-200.0, span_values=spans,
                            min_points=12, min_span_cm=0.0)
        self.assertEqual(fit.points, 0)

    def test_far_cluster_does_not_replace_the_real_boundary(self) -> None:
        """A bigger but far-away cluster must not win over the known wall."""

        near = [-200.0 + (i % 3 - 1) for i in range(12)]
        far = [-200.0 + 120.0 + i for i in range(80)]
        values = near + far
        spans = [float(i) for i in range(len(values))]
        fit = fit_axis_mode(values, expected_cm=-200.0, span_values=spans,
                            min_points=10, min_span_cm=0.0)
        self.assertGreater(fit.points, 0)
        self.assertLess(abs(fit.coordinate_cm - (-200.0)), 5.0)

    def test_short_along_wall_cluster_is_rejected(self) -> None:
        """A dense cluster from a pole must fail the along-wall span gate."""

        values = [-200.0 for _ in range(20)]
        spans = [float(i) * 0.5 for i in range(20)]  # 9.5 cm span only
        fit = fit_axis_mode(values, expected_cm=-200.0, span_values=spans,
                            min_points=5, min_span_cm=50.0)
        self.assertEqual(fit.points, 0)


class PolarAggregationTests(unittest.TestCase):
    def test_low_quantile_recovers_near_boundary(self) -> None:
        """Pooling revolutions lets a low quantile beat through-mesh hits."""

        scans = []
        for sweep in range(10):
            pts = []
            for i in range(36):
                angle = -180.0 + i * 10.0
                # Mostly the near boundary, sometimes punched through far away.
                distance = 200.0 if (i + sweep) % 5 else 600.0
                pts.append((distance * math.cos(math.radians(-angle)),
                            distance * math.sin(math.radians(-angle))))
            scans.append(make_scan(pts))
        bins = aggregate_polar_bins(scans, bin_deg=10.0)
        self.assertTrue(bins)
        for b in bins:
            self.assertGreater(b.hits, 1)
            low = b.quantile_cm(0.2)
            self.assertIsNotNone(low)
            self.assertLess(low, 300.0)

    def test_low_quantile_respects_min_hits(self) -> None:
        scans = [make_scan([(200.0, 0.0)])]
        bins = aggregate_polar_bins(scans, bin_deg=10.0)
        self.assertTrue(bins)
        self.assertIsNone(bins[0].near_quantile_cm(0.2, min_hits=5))


if __name__ == "__main__":
    unittest.main()
