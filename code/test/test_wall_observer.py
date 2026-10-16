"""W01..W12 plus C02..C05: raw scan -> local wall lines, offline and hardware-free.

The synthetic walls carry a few millimetres of range noise on purpose. A
mathematically perfect, exactly axis-aligned point train has zero perpendicular
variance, so its orthogonal fit is genuinely degenerate and is rejected on
purpose (see W07); a real D500 revolution is never that clean.
"""

from pathlib import Path
import math
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.wall_observer import (
    WallLine,
    WallObserver,
    WallSnapshot,
    WallTracker,
    extract_lines,
    fit_line_tls,
    scan_points_base,
    select_candidate,
    supported_corner,
    wrap_to_pi,
)

# Realistic D500 range jitter, metres.
SENSOR_NOISE_M = 0.005


class RadarPointStub:
    def __init__(self, angle_cw_deg, distance_mm, confidence=100):
        self.angle_cw_deg = angle_cw_deg
        self.distance_mm = distance_mm
        self.confidence = confidence

    def sensor_xy_cm(self):
        distance_cm = self.distance_mm / 10.0
        angle = math.radians(self.angle_cw_deg)
        return distance_cm * math.cos(angle), -distance_cm * math.sin(angle)


class RadarScanStub:
    def __init__(self, points, timestamp_ms=0, rotation_speed_deg_s=1800):
        self.points = tuple(points)
        self.timestamp_ms = timestamp_ms
        self.rotation_speed_deg_s = rotation_speed_deg_s


class MountStub:
    def __init__(self, x_forward_cm=0.0, y_left_cm=0.0, yaw_cw_deg=0.0):
        self.x_forward_cm = x_forward_cm
        self.y_left_cm = y_left_cm
        self.yaw_cw_deg = yaw_cw_deg


def right_wall_points(distance_m, *, span_m=3.0, count=61, noise_m=SENSOR_NOISE_M, rng=None):
    """Points on a wall to the robot's right, i.e. at y = -distance, n = (0, -1)."""
    rng = random.Random(1) if rng is None else rng
    step = span_m / (count - 1)
    return [(-span_m / 2 + index * step, -distance_m + rng.gauss(0.0, noise_m))
            for index in range(count)]


def front_wall_points(distance_m, *, span_m=3.0, count=61, noise_m=SENSOR_NOISE_M, rng=None):
    """Points on a wall ahead of the robot, i.e. at x = +distance, n = (1, 0)."""
    rng = random.Random(2) if rng is None else rng
    step = span_m / (count - 1)
    return [(distance_m + rng.gauss(0.0, noise_m), -span_m / 2 + index * step)
            for index in range(count)]


def rear_wall_points(distance_m, *, span_m=3.0, count=61, noise_m=SENSOR_NOISE_M, rng=None):
    """Points on a wall astern of the robot, i.e. at x = -distance, n = (-1, 0)."""
    rng = random.Random(3) if rng is None else rng
    step = span_m / (count - 1)
    return [(-distance_m + rng.gauss(0.0, noise_m), -span_m / 2 + index * step)
            for index in range(count)]


class LineFittingTests(unittest.TestCase):
    def test_w01_right_wall_distance_normal_span_and_rms(self):
        lines = extract_lines(right_wall_points(1.4), rng=random.Random(11))
        self.assertTrue(lines)
        line = min(lines, key=lambda item: item.rho_base_m)
        self.assertAlmostEqual(line.rho_base_m, 1.4, delta=0.02)
        self.assertLess(line.normal_xy[1], -0.9)
        self.assertGreaterEqual(line.span_m, 0.8)
        self.assertLessEqual(line.rms_m, 0.03)
        self.assertGreater(line.inlier_count, 10)

    def test_w02_a_2_13_m_wall_is_not_excluded_by_a_short_range_roi(self):
        lines = extract_lines(right_wall_points(2.13), rng=random.Random(12))
        self.assertTrue(lines)
        self.assertAlmostEqual(max(line.rho_base_m for line in lines), 2.13, delta=0.02)

    def test_a_front_and_a_rear_wall_keep_opposite_normals(self):
        front = extract_lines(front_wall_points(1.895), rng=random.Random(13))
        rear = extract_lines(rear_wall_points(1.69), rng=random.Random(14))
        self.assertGreater(front[0].normal_xy[0], 0.9)
        self.assertLess(rear[0].normal_xy[0], -0.9)

    def test_w03_nan_inf_empty_and_duplicate_points_never_yield_a_wall(self):
        rng = random.Random(15)
        self.assertEqual(extract_lines([], rng=rng), ())
        self.assertEqual(extract_lines([(1.0, float('nan')), (float('inf'), 2.0)], rng=rng), ())
        # A single duplicated location is degenerate, not a wall.
        self.assertEqual(extract_lines([(1.0, 1.0)] * 40, rng=rng), ())
        # Exactly collinear points are a perfectly good wall: zero
        # perpendicular spread is the least ambiguous fit there is.
        collinear = extract_lines([(-1.5 + index * 0.05, -1.6) for index in range(61)],
                                  rng=rng)
        self.assertEqual(len(collinear), 1)
        self.assertAlmostEqual(collinear[0].rho_base_m, 1.6, delta=0.01)
        # A set of one repeated location has no direction at all.
        self.assertEqual(extract_lines([(1.0, 1.0)] * 4, rng=rng), ())

    def test_w04_l_shape_yields_two_orthogonal_lines(self):
        rng = random.Random(16)
        right = right_wall_points(1.4, span_m=2.4, count=49, rng=random.Random(4))
        front = front_wall_points(1.8, span_m=2.4, count=49, rng=random.Random(5))
        lines = extract_lines(right + front, rng=rng)
        self.assertEqual(len(lines), 2)
        distances = sorted(line.rho_base_m for line in lines)
        self.assertAlmostEqual(distances[0], 1.4, delta=0.05)
        self.assertAlmostEqual(distances[1], 1.8, delta=0.05)
        bearings = sorted(line.bearing_rad for line in lines)
        # -90 deg (right wall) and -0.1 deg (front wall): orthogonal.
        self.assertAlmostEqual(bearings[0], -math.pi / 2, delta=math.radians(10))
        self.assertAlmostEqual(bearings[1], 0.0, delta=math.radians(10))
        self.assertAlmostEqual(bearings[1] - bearings[0], math.pi / 2, delta=math.radians(15))

    def test_a_short_pillar_edge_cannot_trigger_a_corner(self):
        rng = random.Random(17)
        right = right_wall_points(1.6, span_m=2.0, count=41, rng=random.Random(6))
        pillar = front_wall_points(1.9, span_m=0.25, count=6, rng=random.Random(7))
        lines = extract_lines(right + pillar, rng=rng)
        self.assertTrue(lines)
        self.assertTrue(all(line.span_m >= 0.8 for line in lines))

    def test_w05_a_gap_keeps_the_real_support_segment(self):
        rng = random.Random(18)
        near = right_wall_points(1.4, span_m=1.0, count=21, rng=random.Random(8))
        far = [(x + 2.2, y) for x, y in
               right_wall_points(1.4, span_m=1.0, count=21, rng=random.Random(9))]
        lines = extract_lines(near + far, rng=rng, max_support_gap_m=0.6)
        self.assertTrue(lines)
        for line in lines:
            # The 1.2 m hole must never be reported as continuous wall length.
            self.assertLess(line.span_m, 1.2)

    def test_w06_association_prefers_continuity_over_a_stronger_intruder(self):
        rng = random.Random(19)
        real = right_wall_points(1.6, span_m=1.2, count=25, rng=random.Random(10))
        intruder = right_wall_points(2.4, span_m=3.0, count=61, rng=random.Random(20))
        lines = extract_lines(real + intruder, rng=rng)
        seed = select_candidate(lines, -math.pi / 2, expected_distance_m=1.6)
        self.assertIsNotNone(seed)
        tracker = WallTracker()
        tracker.set_reference(seed)
        tracked = tracker.track(lines, expected_bearing_rad=seed.bearing_rad,
                               expected_distance_m=1.6)
        self.assertIsNotNone(tracked)
        self.assertAlmostEqual(tracked.rho_base_m, 1.6, delta=0.2)

    def test_w07_degenerate_fits_are_rejected(self):
        self.assertIsNone(fit_line_tls([(1.0, 1.0), (1.0, 1.0), (1.0, 1.0)]))
        self.assertIsNone(fit_line_tls([(0.0, 0.0), (0.5, 0.5)]))
        rng = random.Random(21)
        blob = [(rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05)) for _ in range(80)]
        self.assertEqual(extract_lines(blob, rng=rng), ())
        # Fewer than three points cannot define a line.
        self.assertIsNone(fit_line_tls([(1.0, 1.0), (1.0, 1.02)]))
        # A line through the base origin has rho == 0 and is not an external wall.
        self.assertIsNone(fit_line_tls([(i * 0.1, i * 0.1) for i in range(10)]))

    def test_w08_a_short_arc_cannot_masquerade_as_a_long_wall(self):
        rng = random.Random(22)
        arc = [(1.6 * math.cos(math.radians(-20 + index * 0.4)),
                1.6 * math.sin(math.radians(-20 + index * 0.4)))
               for index in range(61)]
        self.assertEqual(extract_lines(arc, rng=rng, min_span_m=1.5, max_rms_m=0.01), ())

    def test_sparse_front_wall_survives_removing_three_denser_walls(self):
        points = ([(-3 + 8 * i / 89, -1.6) for i in range(90)]
                  + [(-3 + 8 * i / 69, 1.2) for i in range(70)]
                  + [(-3, -1.5 + 2.6 * i / 49) for i in range(50)]
                  + [(5, -1.4 + 2.4 * i / 14) for i in range(15)])
        for seed in range(3):
            with self.subTest(seed=seed):
                lines = extract_lines(points, rng=random.Random(seed))
                front = [line for line in lines if line.normal_xy[0] > 0.9]
                self.assertEqual(len(front), 1)
                self.assertAlmostEqual(front[0].rho_base_m, 5.0, places=5)

    def test_w12_one_wall_only_is_still_a_usable_straight_section(self):
        lines = extract_lines(right_wall_points(1.9), rng=random.Random(23))
        self.assertEqual(len(lines), 1)
        self.assertAlmostEqual(lines[0].rho_base_m, 1.9, delta=0.02)

    def test_bearing_helper_wraps_to_the_principal_branch(self):
        for angle in (0.0, 1.0, -1.0, 3.0, -3.0):
            self.assertAlmostEqual(wrap_to_pi(angle), angle, places=9)
        self.assertAlmostEqual(abs(wrap_to_pi(3 * math.pi)), math.pi, places=9)
        self.assertAlmostEqual(wrap_to_pi(2 * math.pi + 0.25), 0.25, places=9)
        with self.assertRaises(ValueError):
            wrap_to_pi(float('nan'))


class CoordinateTests(unittest.TestCase):
    def test_c02_a_clockwise_90_degree_raw_point_is_on_the_sensor_right(self):
        points = scan_points_base(RadarScanStub([RadarPointStub(90.0, 1000)]), MountStub())
        self.assertAlmostEqual(points[0][0], 0.0, places=9)
        self.assertAlmostEqual(points[0][1], -1.0, places=9)

    def test_c03_a_mount_yaw_of_90_degrees_rotates_exactly_once(self):
        points = scan_points_base(RadarScanStub([RadarPointStub(0.0, 1000)]),
                                  MountStub(yaw_cw_deg=90.0))
        # Radar frame +90 deg clockwise puts the beam along the body's right.
        self.assertAlmostEqual(points[0][0], 0.0, places=9)
        self.assertAlmostEqual(points[0][1], -1.0, places=9)

    def test_c04_nonzero_mount_x_and_y_shift_every_direction(self):
        mount = MountStub(x_forward_cm=10.0, y_left_cm=5.0)
        forward = scan_points_base(RadarScanStub([RadarPointStub(0.0, 2000)]), mount)[0]
        self.assertAlmostEqual(forward[0], 2.1, places=9)
        self.assertAlmostEqual(forward[1], 0.05, places=9)
        left = scan_points_base(RadarScanStub([RadarPointStub(270.0, 2000)]), mount)[0]
        self.assertAlmostEqual(left[0], 0.1, places=9)
        self.assertAlmostEqual(left[1], 2.05, places=9)
        behind = scan_points_base(RadarScanStub([RadarPointStub(180.0, 2000)]), mount)[0]
        self.assertAlmostEqual(behind[0], -1.9, places=9)
        self.assertAlmostEqual(behind[1], 0.05, places=9)

    def test_c04_rho_base_equals_rho_lidar_plus_normal_dot_translation(self):
        scan = RadarScanStub([RadarPointStub(90.0, 1500)])
        point = scan_points_base(scan, MountStub(x_forward_cm=10.0))[0]
        normal = (0.0, -1.0)
        rho_base = point[0] * normal[0] + point[1] * normal[1]
        self.assertAlmostEqual(rho_base, 1.5, places=9)

    def test_the_mount_offset_is_applied_exactly_once(self):
        # 1.5 m to the radar's right plus a 10 cm forward mount offset must stay
        # a 1.5 m base-frame perpendicular distance to that wall.
        scan = RadarScanStub([RadarPointStub(90.0, 1500)])
        once = scan_points_base(scan, MountStub(x_forward_cm=10.0))[0]
        twice = scan_points_base(scan, MountStub(x_forward_cm=20.0))[0]
        self.assertAlmostEqual(once[1], twice[1], places=9)
        self.assertNotAlmostEqual(once[0], twice[0], places=9)

    def test_c05_flipping_normal_and_rho_together_leaves_the_geometry_unchanged(self):
        lines = extract_lines(right_wall_points(1.6), rng=random.Random(24))
        line = lines[0]
        flipped_normal = (-line.normal_xy[0], -line.normal_xy[1])
        flipped_rho = -line.rho_base_m
        # Both forms describe the same point set: n.p = rho.
        for point in ((1.5, -1.6), (0.0, -1.6), (-1.5, -1.6)):
            self.assertAlmostEqual(
                point[0] * line.normal_xy[0] + point[1] * line.normal_xy[1],
                line.rho_base_m, delta=0.05)
            self.assertAlmostEqual(point[0] * flipped_normal[0] + point[1] * flipped_normal[1],
                                   flipped_rho, delta=0.05)
        # Taking abs() of rho alone is what must never happen: the type refuses a
        # negative distance instead of silently inverting the normal.
        with self.assertRaises(ValueError):
            WallLine(normal_xy=line.normal_xy, rho_base_m=flipped_rho,
                     support_start_xy=line.support_start_xy,
                     support_end_xy=line.support_end_xy,
                     inlier_count=line.inlier_count, span_m=line.span_m, rms_m=line.rms_m)


class SnapshotFreshnessTests(unittest.TestCase):
    def make(self, *, seq=1, observed=10.0, lines=None):
        line = WallLine(normal_xy=(0.0, -1.0), rho_base_m=1.6,
                        support_start_xy=(0.0, -1.0), support_end_xy=(0.0, 1.0),
                        inlier_count=20, span_m=2.0, rms_m=0.01)
        return WallSnapshot(scan_seq=seq, scan_started_s=observed - 0.05,
                            scan_completed_s=observed + 0.05, observed_at_s=observed,
                            lines=tuple([line] if lines is None else lines),
                            point_count=40)

    def test_w09_reading_the_same_sequence_twenty_times_counts_once(self):
        from components.wall_navigation import FreshStopGate
        gate = FreshStopGate(required=2, max_age_s=1.0)
        gate.last_seq = 4
        snapshot = self.make(seq=5, observed=1.0)
        results = [gate.observe(snapshot=snapshot, now_s=1.1, zero_commanded_s=0.8,
                                error_rad=0.0) for _ in range(20)]
        self.assertEqual(results.count(True), 0)
        self.assertEqual(gate.stable, 1)

    def test_a_fresh_sequence_completes_but_a_stale_one_resets(self):
        from components.wall_navigation import FreshStopGate
        gate = FreshStopGate(required=2, max_age_s=0.4, tolerance_rad=math.radians(5))
        self.assertFalse(gate.observe(snapshot=self.make(seq=1, observed=1.0), now_s=1.06,
                                      zero_commanded_s=0.8, error_rad=0.0))
        self.assertTrue(gate.observe(snapshot=self.make(seq=2, observed=1.1), now_s=1.16,
                                     zero_commanded_s=0.8, error_rad=0.0))
        self.assertEqual(gate.stable, 2)
        # A late-published revolution that started before zero output cannot count.
        gate.reset()
        self.assertFalse(gate.observe(snapshot=self.make(seq=3, observed=0.5), now_s=1.2,
                                      zero_commanded_s=0.8, error_rad=0.0))
        self.assertEqual(gate.stable, 0)
        # A scan whose angle is outside tolerance resets the count as well.
        self.assertFalse(gate.observe(snapshot=self.make(seq=4, observed=1.2), now_s=1.26,
                                      zero_commanded_s=0.8, error_rad=math.radians(20)))
        self.assertEqual(gate.stable, 0)

    def test_w10_a_late_published_old_scan_never_drives(self):
        old = self.make(seq=3, observed=1.0)
        self.assertFalse(old.is_fresh(5.0, max_age_s=0.4))
        self.assertTrue(old.is_fresh(1.2, max_age_s=0.4))

    def test_w11_reconnect_invalidates_the_old_snapshot_and_reference(self):
        observer = WallObserver(clock=lambda: 100.0, serial_driver=None, mount=MountStub())
        published = observer.publish_points(right_wall_points(1.6))
        self.assertTrue(published.lines)
        observer._on_connected()
        self.assertIsNone(observer.latest())

    def test_an_errored_snapshot_is_never_fresh_but_an_empty_one_is(self):
        empty = WallSnapshot(scan_seq=1, scan_started_s=1.0, scan_completed_s=1.1,
                             observed_at_s=1.05, lines=())
        # A current scan with no usable line is a *selection* failure, not a
        # staleness failure: the caller reports "no wall visible" instead.
        self.assertTrue(empty.is_fresh(1.1))
        errored = WallSnapshot(scan_seq=1, scan_started_s=1.0, scan_completed_s=1.1,
                               observed_at_s=1.05, lines=(), error="fit failed")
        self.assertFalse(errored.is_fresh(1.1))

    def test_one_snapshot_holds_lines_measured_at_the_same_time(self):
        snapshot = self.make()
        self.assertLessEqual(snapshot.scan_started_s, snapshot.scan_completed_s)
        self.assertTrue(snapshot.is_fresh(snapshot.observed_at_s + 0.05))


class CornerTests(unittest.TestCase):
    def wall(self, normal, rho, start, end):
        return WallLine(normal_xy=normal, rho_base_m=rho, support_start_xy=start,
                        support_end_xy=end, inlier_count=30,
                        span_m=math.dist(start, end), rms_m=0.01)

    def test_supported_corner_accepts_a_front_right_corner(self):
        # Real supports meeting at the corner (1.9, -1.6).
        right = self.wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        front = self.wall((1.0, 0.0), 1.9, (1.9, 0.0), (-1.1, 0.0))
        corner = supported_corner(right, front)
        self.assertIsNotNone(corner)
        self.assertAlmostEqual(corner[0], 1.9, places=6)
        self.assertAlmostEqual(corner[1], -1.6, places=6)

    def test_a_corner_inside_both_observed_spans_is_accepted(self):
        # Both supports run past the corner, so the crossing is real evidence.
        right = self.wall((0.0, -1.0), 1.6, (0.0, -2.0), (0.0, 2.0))
        front = self.wall((1.0, 0.0), 1.9, (-2.0, 0.0), (2.0, 0.0))
        corner = supported_corner(right, front)
        self.assertIsNotNone(corner)
        self.assertAlmostEqual(corner[0], 1.9, places=6)
        self.assertAlmostEqual(corner[1], -1.6, places=6)

    def test_a_rear_corner_is_selected_with_the_rear_quadrant(self):
        # Supports meeting at the rear-right corner (-1.4, -1.6).
        right = self.wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        rear = self.wall((-1.0, 0.0), 1.4, (-1.4, 0.0), (1.6, 0.0))
        # The rear wall must be asked for explicitly, not matched as a front wall.
        self.assertIsNone(supported_corner(right, rear))
        corner = supported_corner(right, rear, rear=True)
        self.assertIsNotNone(corner)
        self.assertAlmostEqual(corner[0], -1.4, places=6)
        self.assertAlmostEqual(corner[1], -1.6, places=6)

    def test_a_parallel_wall_does_not_trigger_a_corner(self):
        right = self.wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        parallel = self.wall((0.0, 1.0), 2.2, (0.0, 1.8), (0.0, 2.6))
        self.assertIsNone(supported_corner(right, parallel))

    def test_a_short_wall_stopping_far_before_the_corner_is_rejected(self):
        right = self.wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        # Orthogonal, but its observed span ends 1.5 m short of the crossing
        # point: that is an unbounded extrapolation, not a corner that was seen.
        stub = self.wall((1.0, 0.0), 1.9, (0.4, 0.0), (0.5, 0.0))
        self.assertFalse(stub.covers_along((1.9, -1.6), max_extension_m=0.8))
        self.assertIsNone(supported_corner(right, stub))

    def test_a_wall_ending_just_short_of_the_corner_is_still_accepted(self):
        right = self.wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        # Ends 10 cm before the corner, inside the bounded-extension allowance.
        almost = self.wall((1.0, 0.0), 1.9, (0.4, 0.0), (1.8, 0.0))
        corner = supported_corner(right, almost)
        self.assertIsNotNone(corner)
        self.assertAlmostEqual(corner[0], 1.9, places=6)

    def test_an_orthogonal_wall_on_the_wrong_side_is_rejected(self):
        right = self.wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        # Normal points to +x, so this crossing is behind the chassis, not ahead.
        behind = self.wall((-1.0, 0.0), 1.9, (-1.9, 0.0), (1.1, 0.0))
        self.assertIsNone(supported_corner(right, behind))

    def test_the_fitted_rms_stays_inside_the_quality_gate(self):
        lines = extract_lines(right_wall_points(1.6), rng=random.Random(31))
        self.assertLess(lines[0].rms_m, 0.03)


if __name__ == '__main__':
    unittest.main()
