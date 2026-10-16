"""C06..C12 and T01..T08: wall control signs, braking and in-place turns.

Every convergence case actually integrates the emitted twist through a
differential model and regenerates the wall observation from the resulting
pose, instead of only asserting that a function was called or that a sign looks
plausible.
"""

from pathlib import Path
import math
import random
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.wall_navigation import (
    FollowRightWall,
    HoldStopped,
    LEFT_BEARING_RAD,
    MoveNormalToWall,
    REAR_BEARING_RAD,
    RIGHT_BEARING_RAD,
    TurnToWallBearing,
    WallActionLimits,
    WallTracker,
    braking_cap,
    follow_command,
    is_front_wall,
    is_rear_wall,
    is_right_wall,
    normal_move_command,
    turn_command,
    wall_pure_pursuit,
)
from components.wall_observer import WallLine, WallSnapshot

# The measured geometry from the field config, so the tests exercise the real
# track width rather than a convenient number.
TRACK_WIDTH_M = 0.198

LIMITS = WallActionLimits(
    follow_speed_m_s=0.15, vision_speed_m_s=0.08, minimum_speed_m_s=0.045,
    lookahead_m=0.20, track_width_m=TRACK_WIDTH_M, wheel_speed_m_s=0.30,
    max_linear_speed_m_s=0.20, max_angular_speed_rad_s=0.80,
    max_linear_accel_m_s2=0.30, max_angular_accel_rad_s2=1.00)


def make_wall(bearing_rad, rho, *, span=2.0, rms=0.01):
    normal = (math.cos(bearing_rad), math.sin(bearing_rad))
    tangent = (-normal[1], normal[0])
    centre = (normal[0] * rho, normal[1] * rho)
    return WallLine(normal_xy=normal, rho_base_m=rho,
                    support_start_xy=(centre[0] - span / 2 * tangent[0],
                                      centre[1] - span / 2 * tangent[1]),
                    support_end_xy=(centre[0] + span / 2 * tangent[0],
                                    centre[1] + span / 2 * tangent[1]),
                    inlier_count=30, span_m=span, rms_m=rms)


class SimulatedChassis:
    """A differential base plus a wall world, so control can be closed for real."""

    def __init__(self, *, x=0.0, y=0.0, yaw=0.0, right_distance=1.6):
        self.x, self.y, self.yaw = x, y, yaw
        self.right_distance = right_distance
        self.dt = 0.02

    def step(self, command):
        # Exact unicycle integration of the emitted body twist.
        self.x += command.linear_x_m_s * math.cos(self.yaw) * self.dt
        self.y += command.linear_x_m_s * math.sin(self.yaw) * self.dt
        self.yaw += command.angular_z_rad_s * self.dt

    def right_wall_rho(self):
        """Perpendicular body-frame distance to a wall parallel to the path.

        The world wall sits at ``y = -right_distance``; the chassis is rotated by
        ``yaw``, so the perpendicular distance to that wall as seen in the body
        frame is the world offset projected on the body's right normal.
        """
        normal_world = (0.0, -1.0)
        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
        # Body-frame expression of the world normal.
        normal_body = (cos_y * normal_world[0] + sin_y * normal_world[1],
                       -sin_y * normal_world[0] + cos_y * normal_world[1])
        # rho_base for a line n.p = rho expressed in body coordinates.
        point_world = (0.0, -self.right_distance)
        offset_body = (cos_y * (point_world[0] - self.x) + sin_y * (point_world[1] - self.y),
                       -sin_y * (point_world[0] - self.x) + cos_y * (point_world[1] - self.y))
        return normal_body[0] * offset_body[0] + normal_body[1] * offset_body[1]

    def right_wall_line(self, *, span=3.0):
        rho = self.right_wall_rho()
        bearing = math.atan2(-math.cos(self.yaw), -math.sin(self.yaw))
        return make_wall(bearing, rho, span=span)

    def front_wall_line(self, front_distance, *, span=3.0):
        rho = self.front_wall_rho(front_distance)
        bearing = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        return make_wall(bearing, rho, span=span)

    def front_wall_rho(self, front_distance):
        point_world = (0.0, front_distance)  # placeholder, replaced below
        del point_world
        normal_world = (0.0, 1.0)
        x_wall = front_distance
        cos_y, sin_y = math.cos(self.yaw), math.sin(self.yaw)
        offset_body = (cos_y * (x_wall - self.x) + sin_y * (0.0 - self.y),
                       -sin_y * (x_wall - self.x) + cos_y * (0.0 - self.y))
        normal_body = (cos_y * normal_world[0] + sin_y * normal_world[1],
                       -sin_y * normal_world[0] + cos_y * normal_world[1])
        return normal_body[0] * offset_body[0] + normal_body[1] * offset_body[1]


def snapshot_of(*lines, seq=1, observed=1.0):
    return WallSnapshot(scan_seq=seq, scan_started_s=observed - 0.05,
                        scan_completed_s=observed + 0.05, observed_at_s=observed,
                        lines=tuple(lines), point_count=60)


def wall(normal, rho, start, end):
    """A wall line with explicit support endpoints, for corner geometry."""
    return WallLine(normal_xy=normal, rho_base_m=rho, support_start_xy=start,
                    support_end_xy=end, inlier_count=30,
                    span_m=math.dist(start, end), rms_m=0.01)


class PurePursuitSignTests(unittest.TestCase):
    def test_c06_a_forward_error_away_from_the_wall_steers_toward_it(self):
        # Right wall normal (0,-1); rho bigger than target means too far out.
        wall = make_wall(-math.pi / 2, 1.9)
        v, omega = wall_pure_pursuit(wall, 1.6, 0.15, 0.20)
        self.assertGreater(v, 0.0)
        self.assertLess(omega, 0.0)  # steer right, back toward the wall

    def test_c06_a_forward_error_too_close_steers_away_from_the_wall(self):
        wall = make_wall(-math.pi / 2, 1.3)
        _, omega = wall_pure_pursuit(wall, 1.6, 0.15, 0.20)
        self.assertGreater(omega, 0.0)

    def test_c07_reversing_inverts_the_steering_response(self):
        wall = make_wall(-math.pi / 2, 1.9)
        v, omega = wall_pure_pursuit(wall, 1.6, -0.15, 0.20)
        self.assertLess(v, 0.0)
        self.assertGreater(omega, 0.0)

    def test_a_wall_not_alongside_the_chassis_is_refused(self):
        front = make_wall(0.0, 1.9)
        with self.assertRaises(ValueError):
            wall_pure_pursuit(front, 1.6, 0.15, 0.20)


class FollowCommandTests(unittest.TestCase):
    def test_c09_the_braking_cap_is_never_lifted_by_the_cruise_floor(self):
        wall = make_wall(-math.pi / 2, 1.6)
        # Far out, the cruise cap wins over the braking curve.
        self.assertAlmostEqual(braking_cap(1.9, 1.6, 0.15, deceleration_m_s2=0.3,
                                           tolerance_m=0.02), 0.15, places=9)
        # Close in, the curve is exactly sqrt(2*a*remaining).
        self.assertAlmostEqual(braking_cap(1.63, 1.6, 0.15, deceleration_m_s2=0.3,
                                           tolerance_m=0.02),
                               math.sqrt(2 * 0.3 * 0.03), places=9)
        # Deep inside the final approach the curve falls below the cruise floor.
        # The floor must not lift it: no terminal speed may ever exceed the cap.
        for remaining in (0.001, 0.0005, 0.0001):
            measured = 1.6 + remaining
            cap = braking_cap(measured, 1.6, 0.15, deceleration_m_s2=0.3,
                              tolerance_m=0.005)
            self.assertLess(cap, LIMITS.minimum_speed_m_s)
            command, fields = follow_command(
                wall, 1.6, +1, limits=LIMITS, terminal_measured_m=measured,
                terminal_target_m=1.6, terminal_tolerance_m=0.005)
            self.assertLessEqual(abs(command.linear_x_m_s), cap + 1e-12)
            self.assertLessEqual(command.linear_x_m_s, LIMITS.follow_speed_m_s)

    def test_c09_inside_tolerance_the_command_is_exactly_zero(self):
        wall = make_wall(-math.pi / 2, 1.6)
        command, fields = follow_command(wall, 1.6, +1, limits=LIMITS,
                                        terminal_measured_m=1.61, terminal_target_m=1.6)
        self.assertEqual(command.linear_x_m_s, 0.0)
        self.assertEqual(command.angular_z_rad_s, 0.0)
        self.assertTrue(fields["at_terminal"])

    def test_c10_a_crossed_threshold_stops_and_never_backs_up(self):
        # Already past the target: distance below target must still be zero, not
        # a reverse command chasing the threshold back.
        wall = make_wall(-math.pi / 2, 1.6)
        command, fields = follow_command(wall, 1.6, +1, limits=LIMITS,
                                        terminal_measured_m=1.55, terminal_target_m=1.6)
        self.assertEqual(command.linear_x_m_s, 0.0)
        self.assertEqual(command.angular_z_rad_s, 0.0)

    def test_the_terminal_creep_never_lets_the_chassis_stall_outside_tolerance(self):
        wall = make_wall(-math.pi / 2, 1.6)
        # 2 cm outside tolerance and nearly stationary: the braking curve rounds
        # to ~0, so a bounded creep keeps approach possible.
        command, fields = follow_command(wall, 1.6, +1, limits=LIMITS,
                                        terminal_measured_m=1.6201, terminal_target_m=1.6,
                                        terminal_tolerance_m=0.02)
        self.assertGreater(command.linear_x_m_s, 0.0)
        self.assertLessEqual(command.linear_x_m_s, LIMITS.follow_speed_m_s)

    def test_c11_a_large_curvature_keeps_the_no_wheel_reversal_limit(self):
        # A nearly perpendicular wall forces a large carrot curvature.
        wall = make_wall(-math.pi / 2 + math.radians(30), 1.6)
        command, fields = follow_command(wall, 1.6, +1, limits=LIMITS)
        peak = abs(command.linear_x_m_s) + abs(command.angular_z_rad_s) * LIMITS.track_width_m / 2
        self.assertLessEqual(peak, LIMITS.wheel_speed_m_s + 1e-9)

    def test_c12_this_component_never_negates_omega_for_reverse(self):
        # The C10B backend compensates once; a second negation here would show up
        # as an inverted bearing for the same geometry.
        wall = make_wall(-math.pi / 2, 1.9)
        forward = follow_command(wall, 1.6, +1, limits=LIMITS)[0]
        reverse = follow_command(wall, 1.6, -1, limits=LIMITS)[0]
        self.assertGreater(forward.linear_x_m_s, 0.0)
        self.assertLess(reverse.linear_x_m_s, 0.0)
        self.assertEqual(math.copysign(1, forward.angular_z_rad_s),
                         -math.copysign(1, reverse.angular_z_rad_s))

    def test_a_terminal_target_without_a_measurement_creeps_instead_of_failing(self):
        # The terminal wall can drop out of view for a scan or two. That must
        # bound the approach, not abort the leg: the caller keeps its last
        # measured remaining as a conservative cap.
        wall = make_wall(-math.pi / 2, 1.6)
        command, fields = follow_command(wall, 1.6, +1, limits=LIMITS,
                                        terminal_target_m=1.9)
        self.assertLessEqual(abs(command.linear_x_m_s), 0.008 + 1e-12)
        self.assertIsNone(fields["terminal_measured_m"])
        self.assertFalse(fields["at_terminal"])

    def test_a_terminal_measurement_without_a_target_is_rejected(self):
        wall = make_wall(-math.pi / 2, 1.6)
        with self.assertRaises(ValueError):
            follow_command(wall, 1.6, +1, limits=LIMITS, terminal_measured_m=1.7)

    def test_an_invalid_direction_is_rejected(self):
        wall = make_wall(-math.pi / 2, 1.6)
        with self.assertRaises(ValueError):
            follow_command(wall, 1.6, 0, limits=LIMITS)


class ClosedLoopFollowTests(unittest.TestCase):
    """C06/C07/C08 with a real differential integration and regenerated walls."""

    def _settle(self, chassis, target, direction, *, steps=600, yaw_gain=0.0):
        errors = []
        for _ in range(steps):
            wall = chassis.right_wall_line()
            outcome_command, fields = follow_command(wall, target, direction, limits=LIMITS)
            chassis.step(outcome_command)
            errors.append(fields["distance_error_m"])
        return errors

    def test_c06_forward_converges_from_too_far_and_too_close(self):
        for start in (2.2, 1.1):
            chassis = SimulatedChassis(right_distance=start)
            self._settle(chassis, 1.6, +1, steps=900)
            self.assertLess(abs(chassis.right_wall_rho() - 1.6), 0.05,
                            msg=f"start {start} did not converge")

    def test_c07_reverse_converges_and_does_not_diverge(self):
        for start in (2.2, 1.1):
            chassis = SimulatedChassis(right_distance=start)
            self._settle(chassis, 1.6, -1, steps=900)
            self.assertLess(abs(chassis.right_wall_rho() - 1.6), 0.05,
                            msg=f"reverse start {start} did not converge")

    def test_c08_a_ten_degree_yaw_offset_converges_in_both_directions(self):
        for yaw in (math.radians(10), math.radians(-10)):
            for direction in (+1, -1):
                chassis = SimulatedChassis(yaw=yaw, right_distance=1.9)
                self._settle(chassis, 1.6, direction, steps=1200)
                self.assertLess(abs(chassis.right_wall_rho() - 1.6), 0.06,
                                msg=f"yaw {yaw} direction {direction} did not converge")
                self.assertLess(abs(chassis.yaw - yaw), math.radians(20))


class TurnCommandTests(unittest.TestCase):
    def test_t01_a_bearing_crossing_minus_pi_keeps_turning_left(self):
        # Right wall at -pi/2 turning toward +pi (from right to behind).
        wall = make_wall(-math.pi / 2 + 0.05, 1.6)
        omega, fields = turn_command(wall, REAR_BEARING_RAD, limits=LIMITS)
        self.assertGreater(omega, 0.0)
        # Just past the -pi/+pi seam the sign must not flip.
        past = make_wall(math.pi - 0.05, 1.6)
        omega_past, fields_past = turn_command(past, REAR_BEARING_RAD, limits=LIMITS)
        self.assertLess(omega_past, 0.0)
        self.assertLess(abs(fields_past["error_rad"]), math.radians(5))

    def test_t02_the_front_wall_is_tracked_not_the_old_right_wall(self):
        front = make_wall(0.0, 1.9)
        omega, fields = turn_command(front, RIGHT_BEARING_RAD, limits=LIMITS)
        self.assertGreater(omega, 0.0)  # left turn
        self.assertAlmostEqual(fields["measured_bearing_rad"], 0.0, places=9)

    def test_t03_the_rear_wall_turns_right_to_become_the_right_wall(self):
        rear = make_wall(REAR_BEARING_RAD, 1.49)
        omega, fields = turn_command(rear, RIGHT_BEARING_RAD, limits=LIMITS)
        self.assertLess(omega, 0.0)  # right turn

    def test_an_in_tolerance_bearing_asks_for_zero(self):
        wall = make_wall(-math.pi / 2 + math.radians(1), 1.6)
        omega, _ = turn_command(wall, RIGHT_BEARING_RAD, limits=LIMITS)
        self.assertEqual(omega, 0.0)

    def test_a_turn_in_place_is_exempt_from_the_no_wheel_reversal_limit(self):
        wall = make_wall(0.0, 1.9)
        omega, _ = turn_command(wall, REAR_BEARING_RAD, limits=LIMITS)
        # 90 deg away: far more than 1.8*|v|/track_width for a stopped chassis.
        self.assertGreater(abs(omega), 1.8 * 0.0 / LIMITS.track_width_m)
        self.assertLessEqual(abs(omega), LIMITS.max_angular_speed_rad_s)


class ClosedLoopTurnTests(unittest.TestCase):
    def _turn(self, chassis, wall_bearing_at_start, target, direction, *, steps=900):
        tracker = WallTracker()
        for _ in range(steps):
            # The tracked wall keeps its world identity; its body bearing moves
            # with the chassis.
            bearing = wall_bearing_at_start - chassis.yaw
            wall = make_wall(bearing, 1.6)
            tracker.set_reference(wall)
            omega, fields = turn_command(wall, target, limits=LIMITS)
            from core.types import Twist2D
            chassis.step(Twist2D(0.0, omega))
            if abs(fields["error_rad"]) <= LIMITS.max_angular_speed_rad_s * 0.0:
                break
        return fields

    def test_a_left_turn_from_right_wall_to_rear_wall_completes(self):
        chassis = SimulatedChassis()
        # Right wall at -pi/2 must end at +pi (behind).
        error = self._turn_left(chassis, -math.pi / 2, REAR_BEARING_RAD)
        self.assertLess(abs(error), math.radians(3))

    def _turn_left(self, chassis, start_bearing, target):
        previous = start_bearing
        for _ in range(1500):
            bearing = previous - chassis.yaw
            wall = make_wall(bearing, 1.6)
            omega, fields = turn_command(wall, target, limits=LIMITS)
            from core.types import Twist2D
            chassis.step(Twist2D(0.0, omega))
            if abs(fields["error_rad"]) <= math.radians(0.5):
                return fields["error_rad"]
        return fields["error_rad"]

    def test_a_right_turn_from_rear_wall_to_right_wall_completes(self):
        chassis = SimulatedChassis()
        error = self._turn_left(chassis, REAR_BEARING_RAD, RIGHT_BEARING_RAD)
        self.assertLess(abs(error), math.radians(3))


class NormalMoveTests(unittest.TestCase):
    def test_c13_forward_toward_a_rear_wall_increases_the_distance(self):
        # Reference wall behind the chassis, so n_x = -1.
        wall = make_wall(REAR_BEARING_RAD, 1.0)
        speed, fields = normal_move_command(wall, 1.43, +1, max_speed_m_s=0.05)
        self.assertGreater(speed, 0.0)
        self.assertLess(fields["normal_x"], 0.0)

    def test_inside_tolerance_the_normal_move_is_zero(self):
        wall = make_wall(REAR_BEARING_RAD, 1.43)
        speed, _ = normal_move_command(wall, 1.43, +1, max_speed_m_s=0.05,
                                       tolerance_m=0.01)
        self.assertEqual(speed, 0.0)

    def test_a_passed_target_never_asks_for_a_reverse_correction(self):
        wall = make_wall(REAR_BEARING_RAD, 1.50)  # already past 1.43
        speed, fields = normal_move_command(wall, 1.43, +1, max_speed_m_s=0.05,
                                           tolerance_m=0.01)
        self.assertEqual(speed, 0.0)
        self.assertLess(fields["remaining_along_direction_m"], 0.0)

    def test_a_chassis_not_facing_the_wall_is_refused_instead_of_dividing_by_zero(self):
        wall = make_wall(RIGHT_BEARING_RAD, 1.6)  # n_x = 0
        with self.assertRaises(ValueError):
            normal_move_command(wall, 1.6, +1, max_speed_m_s=0.05)

    def test_reverse_toward_a_rear_wall_decreases_the_distance(self):
        wall = make_wall(REAR_BEARING_RAD, 1.5)
        speed, _ = normal_move_command(wall, 1.43, -1, max_speed_m_s=0.05)
        self.assertLess(speed, 0.0)


class DropHeadingBandTests(unittest.TestCase):
    """The 43/32 cm move holds the reference wall square, with a +/-5 deg band.

    With the chassis facing the wall its normal is parallel to the nose, so
    ``90 deg - heading_error`` is exactly the angle between that wall and the
    radar's forward axis.
    """

    def action(self, *, direction=+1, target=2.03, bearing=REAR_BEARING_RAD,
               band_deg=5.0, gain=1.5, desired=None, lost_timeout=None,
               tolerance_m=None):
        tracker = WallTracker()
        wall = make_wall(bearing, 1.6)
        tracker.set_reference(wall)
        extra = {} if lost_timeout is None else {"lost_timeout_s": lost_timeout}
        if tolerance_m is not None:
            extra["tolerance_m"] = tolerance_m
        return MoveNormalToWall(target_distance_m=target, direction=direction,
                                reference_bearing_rad=bearing, limits=LIMITS,
                                tracker=tracker,
                                desired_bearing_rad=(bearing if desired is None
                                                     else desired),
                                heading_gain=gain,
                                heading_band_rad=math.radians(band_deg),
                                clock=lambda: 0.0, **extra)

    def command_at_tilt(self, tilt_deg, **kwargs):
        action = self.action(**kwargs)
        # The wall sits ``tilt`` away from the heading the move is holding.
        bearing = REAR_BEARING_RAD + math.radians(tilt_deg)
        wall = make_wall(bearing, 1.6)
        snapshot = snapshot_of(wall, observed=1.0)
        out = action.step(snapshot, now_s=1.05)
        return out.command, out.fields

    def test_inside_five_degrees_the_heading_is_left_alone(self):
        for tilt in (0.0, 1.0, -2.0, 4.9, -4.9, 5.0):
            with self.subTest(tilt=tilt):
                command, fields = self.command_at_tilt(tilt)
                self.assertEqual(command.angular_z_rad_s, 0.0)
                self.assertTrue(fields["heading_within_band"])
                self.assertAlmostEqual(fields["heading_error_deg"], tilt, places=6)
                # The move still happens: a small tilt is not a reason to stall.
                self.assertGreater(abs(command.linear_x_m_s), 0.0)

    def test_outside_the_band_the_error_is_corrected_toward_the_band(self):
        for tilt in (6.0, 10.0, 20.0):
            with self.subTest(tilt=tilt):
                command, fields = self.command_at_tilt(tilt)
                self.assertFalse(fields["heading_within_band"])
                # The wall is to the left of where it should be, so the chassis
                # has to turn the other way to bring it back.
                self.assertGreater(command.angular_z_rad_s, 0.0)
                self.assertLessEqual(abs(command.angular_z_rad_s),
                                     LIMITS.max_angular_speed_rad_s)
        negative, _ = self.command_at_tilt(-10.0)
        self.assertLess(negative.angular_z_rad_s, 0.0)

    def test_the_correction_is_continuous_at_the_band_edge(self):
        # Infinitesimally outside the band the command leaves zero smoothly, so
        # crossing the edge cannot produce a jump.
        just_outside, _ = self.command_at_tilt(5.001)
        self.assertLess(abs(just_outside.angular_z_rad_s), 0.001)
        far_out, _ = self.command_at_tilt(30.0)
        self.assertGreater(abs(far_out.angular_z_rad_s), 0.0)

    def test_the_correction_saturates_instead_of_growing_without_bound(self):
        command, _ = self.command_at_tilt(80.0)
        self.assertLessEqual(abs(command.angular_z_rad_s),
                             LIMITS.max_angular_speed_rad_s)

    def test_the_band_is_configurable(self):
        command, fields = self.command_at_tilt(6.0, band_deg=10.0)
        self.assertEqual(command.angular_z_rad_s, 0.0)
        self.assertTrue(fields["heading_within_band"])
        # Nothing is corrected inside the wider band...
        tight, tight_fields = self.command_at_tilt(6.0, band_deg=1.0)
        # ...while the same tilt is corrected when the band is tight.
        self.assertGreater(abs(tight.angular_z_rad_s), 0.0)
        self.assertFalse(tight_fields["heading_within_band"])

    def test_a_negative_gain_or_band_is_rejected(self):
        with self.assertRaises(ValueError):
            self.action(gain=-1.0)
        with self.assertRaises(ValueError):
            self.action(band_deg=-1.0)

    def test_reaching_the_distance_while_off_axis_is_not_success(self):
        # The whole point of the band: arriving at the right perpendicular
        # distance while badly rotated is not the finished move. The generous
        # tolerance makes "the distance is already met" explicit rather than
        # depending on the exact float comparison at the tolerance edge.
        action = self.action(target=1.62, tolerance_m=0.05)
        wall = make_wall(REAR_BEARING_RAD + math.radians(30.0), 1.6)
        out = action.step(snapshot_of(wall, observed=1.0), now_s=1.05)
        self.assertFalse(out.done)
        self.assertTrue(out.waiting)
        self.assertEqual(out.reason, "squaring_heading")
        # It corrects in place rather than translating further.
        self.assertEqual(out.command.linear_x_m_s, 0.0)
        self.assertNotEqual(out.command.angular_z_rad_s, 0.0)

    def test_success_needs_the_distance_and_a_square_chassis(self):
        # Same distance, square chassis: now it is done.
        action = self.action(target=1.62, tolerance_m=0.05)
        wall = make_wall(REAR_BEARING_RAD, 1.6)
        out = action.step(snapshot_of(wall, observed=1.0), now_s=1.05)
        self.assertTrue(out.done)
        self.assertEqual(out.reason, "distance_reached")

    def test_the_tolerance_sits_outside_the_dead_band(self):
        # If they were equal, the controller would stop correcting exactly at
        # the edge and a residual just beyond it could never be resolved.
        action = self.action(target=1.62, tolerance_m=0.05)
        self.assertGreater(action.heading_tolerance_rad, action.heading_band_rad)
        # A residual of exactly the band is accepted at the target distance...
        within = action.step(snapshot_of(make_wall(REAR_BEARING_RAD + action.heading_band_rad,
                                                   1.6), observed=1.0), now_s=1.05)
        self.assertTrue(within.done)
        # ...while the same residual away from the target keeps being corrected.
        action2 = self.action()
        just_out = action2.step(snapshot_of(
            make_wall(REAR_BEARING_RAD + math.radians(5.2), 1.6), observed=1.0), now_s=1.05)
        self.assertFalse(just_out.done)
        self.assertGreater(abs(just_out.command.angular_z_rad_s), 0.0)

    def test_squaring_in_place_is_bounded_by_the_outage_timeout(self):
        # A chassis that cannot rotate must fail, not wait forever.
        action = self.action(target=1.62, tolerance_m=0.05, lost_timeout=1.0)
        wall = make_wall(REAR_BEARING_RAD + math.radians(30.0), 1.6)
        first = action.step(snapshot_of(wall, observed=1.0), now_s=1.0)
        self.assertEqual(first.reason, "squaring_heading")
        final = action.step(snapshot_of(wall, seq=9, observed=3.0), now_s=3.0)
        self.assertTrue(action.finished)
        self.assertFalse(final.done)
        self.assertEqual(final.reason, "squaring_heading")

    def test_the_axis_unwrap_survives_a_stored_bearing_flip(self):
        # rho and the normal are invariant under n -> -n, so the stored bearing
        # of a rear wall may be reported on the +pi side. That is the same wall,
        # not a 180 degree heading error.
        # Both sides of the pi wrap describe the same physical wall.
        for stored in (math.pi - math.radians(8.0), -math.pi + math.radians(8.0)):
            with self.subTest(stored=stored):
                action = self.action(bearing=stored, desired=stored, target=2.03)
                action.reference_bearing_rad = stored
                self.assertAlmostEqual(
                    action._axis_error(make_wall(stored, 1.6)), 0.0, places=9)

    def test_a_heading_error_of_pi_is_still_reported_as_pi(self):
        action = self.action(bearing=REAR_BEARING_RAD, desired=REAR_BEARING_RAD)
        action.reference_bearing_rad = REAR_BEARING_RAD
        self.assertAlmostEqual(abs(action._axis_error(make_wall(0.0, 1.6))), math.pi,
                               places=9)

    def test_both_move_directions_keep_the_same_heading_behaviour(self):
        # ``rho_dot = -n_x * v``: for a wall astern (n_x = -1) a forward move
        # increases the distance and a reverse move decreases it. Each direction
        # therefore needs a target the right side of the current 1.6 m for there
        # to be anything to do, and the heading band must behave identically.
        for direction, target in ((+1, 2.03), (-1, 1.2)):
            for tilt in (0.0, 10.0, -10.0):
                with self.subTest(direction=direction, tilt=tilt):
                    command, _ = self.command_at_tilt(tilt, direction=direction,
                                                      target=target)
                    expected = normal_move_command(
                        make_wall(REAR_BEARING_RAD + math.radians(tilt), 1.6),
                        target, direction, max_speed_m_s=0.05)[0]
                    self.assertEqual(math.copysign(1.0, command.linear_x_m_s),
                                     math.copysign(1.0, expected))
                    if abs(tilt) > 5.0:
                        self.assertNotEqual(command.angular_z_rad_s, 0.0)
                    else:
                        self.assertEqual(command.angular_z_rad_s, 0.0)


class ActionStateMachineTests(unittest.TestCase):
    def limits(self):
        return LIMITS

    def test_a_follow_action_reports_a_corner_once_and_zero_speed(self):
        # A real corner: the right wall and the front wall meet at (1.9, -1.6),
        # so both observed supports reach the crossing point. Two walls that
        # merely cross far from their supports are not a verified corner.
        action = FollowRightWall(target_distance_m=1.6, direction=+1, limits=LIMITS,
                                 terminal_role=is_front_wall, terminal_target_m=1.9,
                                 tracker=WallTracker())
        right = wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        front = wall((1.0, 0.0), 1.9, (1.9, 0.0), (-1.1, 0.0))
        outcome = action.step(snapshot_of(right, front, observed=0.98), now_s=1.0)
        self.assertTrue(outcome.done)
        self.assertEqual(outcome.reason, "terminal_reached")
        self.assertEqual(outcome.command.linear_x_m_s, 0.0)
        # The verified terminal is exposed for the mission layer's corner turn.
        self.assertIsNotNone(action.terminal_wall)
        self.assertAlmostEqual(action.terminal_wall.rho_base_m, 1.9, places=6)
        # Once per leg: a second call keeps returning the same finished outcome.
        self.assertTrue(action.step(snapshot_of(right, front, observed=1.08),
                                    now_s=1.1).done)

    def test_a_wide_l_corner_is_a_terminal(self):
        # Both lines cross near the origin, but neither support reaches the
        # crossing, so this must not be accepted as the end of the leg.
        action = FollowRightWall(target_distance_m=1.6, direction=+1, limits=LIMITS,
                                 terminal_role=is_front_wall, terminal_target_m=1.9,
                                 tracker=WallTracker())
        right = wall((0.0, -1.0), 1.6, (0.0, -2.0), (0.0, 2.0))
        front = wall((1.0, 0.0), 1.9, (-2.0, 0.0), (2.0, 0.0))
        outcome = action.step(snapshot_of(right, front, observed=0.98), now_s=1.0)
        self.assertTrue(outcome.done)

    def test_a_crossing_wall_far_outside_both_supports_is_not_a_terminal(self):
        # The two infinite lines cross, but the crossing is a metre outside both
        # observed supports. A small bounded extrapolation is tolerated because a
        # real wall may stop just short of the corner; a metre is not.
        action = FollowRightWall(target_distance_m=1.6, direction=+1, limits=LIMITS,
                                 terminal_role=is_front_wall, terminal_target_m=1.9,
                                 tracker=WallTracker())
        right = wall((0.0, -1.0), 1.6, (0.0, -0.3), (0.0, 0.3))
        front = wall((1.0, 0.0), 1.9, (-0.3, 0.0), (0.3, 0.0))
        outcome = action.step(snapshot_of(right, front, observed=0.98), now_s=1.0)
        self.assertFalse(outcome.done)
        self.assertFalse(outcome.fields.get("terminal_visible"))
        self.assertGreater(outcome.command.linear_x_m_s, 0.0)
        self.assertIsNone(action.terminal_wall)

    def test_a_short_obstacle_ahead_is_not_a_terminal_wall(self):
        # A short pillar ahead of the chassis looks like a front wall, but its
        # support ends 1.4 m short of where the real corner must be, so it must
        # not end the leg.
        action = FollowRightWall(target_distance_m=1.6, direction=+1, limits=LIMITS,
                                 terminal_role=is_front_wall, terminal_target_m=1.9,
                                 tracker=WallTracker())
        right = wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        pillar = wall((1.0, 0.0), 0.7, (0.7, 0.0), (0.9, 0.0))
        outcome = action.step(snapshot_of(right, pillar, observed=0.98), now_s=1.0)
        self.assertFalse(outcome.done)
        self.assertFalse(outcome.fields.get("terminal_visible"))
        # The leg keeps following the right wall rather than stopping on it.
        self.assertGreater(outcome.command.linear_x_m_s, 0.0)

    def test_the_right_wall_is_never_its_own_terminal(self):
        action = FollowRightWall(target_distance_m=1.6, direction=+1, limits=LIMITS,
                                 terminal_role=is_right_wall, terminal_target_m=1.0,
                                 tracker=WallTracker())
        right = wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        outcome = action.step(snapshot_of(right, observed=0.98), now_s=1.0)
        self.assertFalse(outcome.done)
        self.assertFalse(outcome.fields.get("terminal_visible"))

    def test_only_the_right_wall_is_a_normal_cruise(self):
        # The far terminal wall is simply not in view yet: that is ordinary
        # following, never a parameter error or a stop.
        action = FollowRightWall(target_distance_m=1.6, direction=+1, limits=LIMITS,
                                 terminal_role=is_front_wall, terminal_target_m=1.9,
                                 tracker=WallTracker())
        right = wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        outcome = action.step(snapshot_of(right, observed=0.98), now_s=1.0)
        self.assertFalse(outcome.done)
        self.assertEqual(outcome.reason, "")
        self.assertFalse(outcome.fields["terminal_seen"])
        self.assertGreater(outcome.command.linear_x_m_s, 0.0)

    def test_a_terminal_seen_then_lost_holds_zero_and_then_fails_bounded(self):
        # Once the terminal has been verified, losing it must not keep driving on
        # the last distance -- that distance cannot shrink as the chassis moves.
        action = FollowRightWall(target_distance_m=1.6, direction=+1, limits=LIMITS,
                                 terminal_role=is_front_wall, terminal_target_m=0.5,
                                 tracker=WallTracker(), lost_timeout_s=5.0)
        right = wall((0.0, -1.0), 1.6, (0.0, -1.6), (0.0, 1.4))
        front = wall((1.0, 0.0), 1.9, (1.9, 0.0), (-1.1, 0.0))
        first = action.step(snapshot_of(right, front, observed=0.98), now_s=1.0)
        self.assertFalse(first.done)
        self.assertTrue(first.fields["terminal_seen"])
        self.assertAlmostEqual(first.fields["terminal_measured_m"], 1.9, places=6)
        # The terminal is gone but the right wall is still perfectly visible.
        lost = action.step(snapshot_of(right, observed=1.05), now_s=1.1)
        self.assertEqual(lost.command.linear_x_m_s, 0.0)
        self.assertEqual(lost.command.angular_z_rad_s, 0.0)
        self.assertEqual(lost.reason, "terminal_wall_lost")
        # Bounded: it eventually gives up instead of waiting forever.
        final = action.step(snapshot_of(right, seq=99, observed=7.0), now_s=7.0)
        self.assertTrue(action.finished)
        self.assertFalse(final.done)
        self.assertEqual(final.reason, "terminal_wall_lost")

    def test_a_follow_action_waits_without_inventing_a_path_when_the_wall_is_gone(self):
        action = FollowRightWall(target_distance_m=1.6, direction=+1, limits=LIMITS,
                                 tracker=WallTracker(), clock=lambda: 0.0)
        # A fresh scan that simply contains no right wall.
        empty = WallSnapshot(scan_seq=1, scan_started_s=0.93, scan_completed_s=1.03,
                             observed_at_s=0.98, lines=(), point_count=12)
        outcome = action.step(empty, now_s=1.0)
        self.assertFalse(outcome.done)
        self.assertIn("not_visible", outcome.reason)
        self.assertEqual(outcome.command.linear_x_m_s, 0.0)
        # And an empty or absent snapshot is reported as a stale observation.
        self.assertEqual(action.step(None, now_s=1.0).reason, "no_wall_observation")

    def test_a_stale_snapshot_yields_no_motion(self):
        action = FollowRightWall(target_distance_m=1.6, direction=+1, limits=LIMITS,
                                 tracker=WallTracker())
        stale = WallSnapshot(scan_seq=1, scan_started_s=0.0, scan_completed_s=0.1,
                             observed_at_s=0.05,
                             lines=(make_wall(-math.pi / 2, 1.6),), point_count=40)
        outcome = action.step(stale, now_s=10.0)
        self.assertFalse(outcome.done)
        self.assertEqual(outcome.reason, "stale_wall_observation")
        self.assertEqual(outcome.command.linear_x_m_s, 0.0)

    def test_t05_a_turn_completes_from_wall_angles_with_no_t265(self):
        import inspect
        source = inspect.getsource(sys.modules[TurnToWallBearing.__module__])
        self.assertNotIn("T265", source)
        self.assertNotIn("TurnController", source)
        action = TurnToWallBearing(reference_bearing_rad=-math.pi / 2,
                                   desired_bearing_rad=RIGHT_BEARING_RAD,
                                   turn_direction=+1, limits=LIMITS,
                                   tracker=WallTracker(), tolerance_rad=math.radians(2),
                                   required_frames=2)
        # Already on the desired bearing: it must hold zero and confirm with two
        # fresh scans, each of which started after zero output was commanded.
        tracking = make_wall(RIGHT_BEARING_RAD, 1.6)
        self.assertFalse(action.step(snapshot_of(tracking, seq=1, observed=1.16),
                                     now_s=1.25).done)
        self.assertEqual(action.gate.stable, 0)  # that scan began before the hold
        self.assertFalse(action.step(snapshot_of(tracking, seq=2, observed=1.36),
                                     now_s=1.45).done)
        self.assertEqual(action.gate.stable, 1)
        second = action.step(snapshot_of(tracking, seq=3, observed=1.46), now_s=1.55)
        self.assertTrue(second.done)
        self.assertEqual(second.reason, "bearing_reached")
        self.assertEqual(second.command.linear_x_m_s, 0.0)

    def test_t06_a_turn_holds_zero_and_does_not_finish_on_a_timer(self):
        action = TurnToWallBearing(reference_bearing_rad=-math.pi / 2,
                                   desired_bearing_rad=REAR_BEARING_RAD,
                                   turn_direction=+1, limits=LIMITS,
                                   tracker=WallTracker(), lost_timeout_s=2.0)
        first = action.step(None, now_s=1.0)
        self.assertFalse(first.done)
        self.assertEqual(first.command.linear_x_m_s, 0.0)
        # The reference comes back; the same action resumes and still turns.
        resumed = action.step(snapshot_of(make_wall(-math.pi / 2, 1.6), observed=1.1),
                              now_s=1.16)
        self.assertFalse(resumed.done)
        self.assertGreater(resumed.command.angular_z_rad_s, 0.0)

    def test_t06_a_persistently_lost_reference_fails_instead_of_timing_a_turn(self):
        action = TurnToWallBearing(reference_bearing_rad=-math.pi / 2,
                                   desired_bearing_rad=REAR_BEARING_RAD,
                                   turn_direction=+1, limits=LIMITS,
                                   tracker=WallTracker(), lost_timeout_s=1.0)
        # A fresh scan that does not contain the tracked wall at all.
        elsewhere = make_wall(LEFT_BEARING_RAD, 2.4)
        first = action.step(snapshot_of(elsewhere, observed=1.0), now_s=1.05)
        self.assertEqual(first.reason, "reference_wall_lost")
        self.assertEqual(first.command.linear_x_m_s, 0.0)
        outcome = action.step(snapshot_of(elsewhere, seq=2, observed=2.5), now_s=2.55)
        self.assertTrue(action.finished)
        self.assertFalse(outcome.done)
        self.assertEqual(outcome.reason, "reference_wall_lost")

    def test_t07_a_scan_delivered_after_zero_that_started_before_it_cannot_confirm(self):
        action = TurnToWallBearing(reference_bearing_rad=-math.pi / 2,
                                   desired_bearing_rad=RIGHT_BEARING_RAD,
                                   turn_direction=+1, limits=LIMITS,
                                   tracker=WallTracker(), required_frames=2,
                                   tolerance_rad=math.radians(2))
        action.begin(10.0)
        tracking = make_wall(RIGHT_BEARING_RAD, 1.6)
        # This scan is already on the bearing and enters the hold at t=10.1, but
        # its revolution completed at 9.99, before zero output was commanded.
        before = WallSnapshot(scan_seq=1, scan_started_s=9.90, scan_completed_s=9.99,
                              observed_at_s=9.95, lines=(tracking,), point_count=40)
        self.assertFalse(action.step(before, now_s=10.1).done)
        self.assertEqual(action.gate.stable, 0)
        # A revolution that straddles the zero instant still contains samples from
        # while the chassis was turning, so it cannot confirm the stop either.
        straddling = WallSnapshot(scan_seq=2, scan_started_s=10.05,
                                  scan_completed_s=10.15, observed_at_s=10.10,
                                  lines=(tracking,), point_count=40)
        self.assertFalse(action.step(straddling, now_s=10.16).done)
        self.assertEqual(action.gate.stable, 0)
        # The first revolution acquired entirely after the command counts.
        after = WallSnapshot(scan_seq=3, scan_started_s=10.17, scan_completed_s=10.27,
                             observed_at_s=10.22, lines=(tracking,), point_count=40)
        self.assertFalse(action.step(after, now_s=10.28).done)
        self.assertEqual(action.gate.stable, 1)
        final = WallSnapshot(scan_seq=4, scan_started_s=10.29, scan_completed_s=10.39,
                             observed_at_s=10.34, lines=(tracking,), point_count=40)
        self.assertTrue(action.step(final, now_s=10.40).done)

    def test_t08_one_scan_inside_tolerance_then_one_outside_resets_the_count(self):
        action = TurnToWallBearing(reference_bearing_rad=-math.pi / 2,
                                   desired_bearing_rad=RIGHT_BEARING_RAD,
                                   turn_direction=+1, limits=LIMITS,
                                   tracker=WallTracker(), required_frames=2,
                                   tolerance_rad=math.radians(2))
        action.begin(1.0)
        inside = make_wall(RIGHT_BEARING_RAD, 1.6)
        self.assertFalse(action.step(snapshot_of(inside, seq=1, observed=1.05),
                                     now_s=1.11).done)
        outside = make_wall(RIGHT_BEARING_RAD + math.radians(20), 1.6)
        self.assertFalse(action.step(snapshot_of(outside, seq=2, observed=1.15),
                                     now_s=1.21).done)
        self.assertEqual(action.gate.stable, 0)


class HoldTests(unittest.TestCase):
    def test_hold_emits_zero_speed_and_is_cancellable_by_the_caller(self):
        clock = [0.0]
        action = HoldStopped(duration_s=0.5, clock=lambda: clock[0])
        action.begin(0.0)
        outcome = action.step(None, now_s=0.0)
        self.assertEqual(outcome.command.linear_x_m_s, 0.0)
        self.assertFalse(outcome.done)
        clock[0] = 0.6
        self.assertTrue(action.step(None, now_s=0.6).done)

    def test_a_negative_hold_duration_is_rejected(self):
        with self.assertRaises(ValueError):
            HoldStopped(duration_s=-1.0)


if __name__ == '__main__':
    unittest.main()
