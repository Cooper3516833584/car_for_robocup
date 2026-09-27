"""Hardware-free tests for the rectangle bootstrap and its explicit ambiguity.

The central requirement is that a square field must NOT silently collapse to one
of its four equally valid global orientations.  Numerical noise always makes one
candidate score marginally better; picking it would build a map rotated by a
multiple of 90 degrees, which is worse than having no map because navigation
would confidently drive the wrong way.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_driver import Pose2D  # noqa: E402
from components.rectangle_bootstrap import (  # noqa: E402
    OrientationAmbiguity,
    bootstrap_rectangle,
    resolve_bootstrap_candidates,
)


def rectangle_points(width_cm, height_cm, *, robot=(0.0, 0.0, 0.0),
                     step_cm=5.0, sides="all"):
    """Points on the rectangle edges, expressed in the robot's own frame.

    ``robot`` is the robot pose inside the field frame (cm, CW-positive deg).
    ``sides`` selects which edges are visible: 'all', 'back_right', ...
    """

    x0, y0, yaw_cw = robot
    edges = []
    if sides in ("all", "back") or "back" in sides:
        edges.append(("back", lambda t: (0.0, t), height_cm))
    if sides in ("all", "right") or "right" in sides:
        edges.append(("right", lambda t: (t, 0.0), width_cm))
    if sides in ("all", "front") or "front" in sides:
        edges.append(("front", lambda t: (width_cm, t), height_cm))
    if sides in ("all", "left") or "left" in sides:
        edges.append(("left", lambda t: (t, height_cm), width_cm))

    pts = []
    for _name, fn, extent in edges:
        t = 0.0
        while t <= extent + 1e-9:
            wx, wy = fn(t)
            dx, dy = wx - x0, wy - y0
            cos_a = math.cos(math.radians(yaw_cw))
            sin_a = math.sin(math.radians(yaw_cw))
            # Body frame is the field frame rotated by -yaw (CW-positive).
            bx = cos_a * dx - sin_a * dy
            by = sin_a * dx + cos_a * dy
            pts.append((bx, by))
            t += step_cm
    return pts


class SquareAmbiguityTests(unittest.TestCase):
    """10 x 10 m competition field."""

    def test_square_yields_four_candidates_and_stays_ambiguous(self) -> None:
        points = rectangle_points(1000.0, 1000.0, robot=(500.0, 500.0, 0.0))
        result = bootstrap_rectangle(
            points, field_width_cm=1000.0, field_height_cm=1000.0
        )
        self.assertTrue(result.geometry_valid, result.reason)
        self.assertFalse(
            result.orientation_resolved,
            "a square field must not be reported as orientation-resolved",
        )
        self.assertEqual(result.ambiguity_order, OrientationAmbiguity.SQUARE)
        self.assertEqual(result.candidate_count, 4)

    def test_square_candidates_differ_by_multiples_of_90_degrees(self) -> None:
        points = rectangle_points(1000.0, 1000.0, robot=(500.0, 500.0, 20.0))
        result = bootstrap_rectangle(
            points, field_width_cm=1000.0, field_height_cm=1000.0
        )
        self.assertEqual(result.candidate_count, 4)
        yaws = [c.pose_in_world.yaw_cw_deg for c in result.candidates]
        for index in range(1, 4):
            delta = (yaws[index] - yaws[0]) % 360.0
            self.assertAlmostEqual(delta, 90.0 * index, places=6)

    def test_observed_dimensions_match_the_configured_field(self) -> None:
        points = rectangle_points(1000.0, 1000.0, robot=(500.0, 500.0, 0.0))
        result = bootstrap_rectangle(
            points, field_width_cm=1000.0, field_height_cm=1000.0
        )
        self.assertAlmostEqual(result.observed_width_cm, 1000.0, delta=60.0)
        self.assertAlmostEqual(result.observed_height_cm, 1000.0, delta=60.0)


class NonSquareRectangleTests(unittest.TestCase):
    """A measured non-square field removes the 90 deg axis swap only."""

    def test_rectangle_keeps_the_180_degree_ambiguity(self) -> None:
        points = rectangle_points(500.0, 400.0, robot=(250.0, 200.0, 0.0))
        result = bootstrap_rectangle(
            points, field_width_cm=500.0, field_height_cm=400.0
        )
        self.assertTrue(result.geometry_valid, result.reason)
        self.assertFalse(result.orientation_resolved)
        self.assertEqual(result.ambiguity_order, OrientationAmbiguity.MIRRORED)
        self.assertEqual(result.candidate_count, 4)
        # The 90/270 rotations are reported but are not consistent with a
        # non-square extent; the caller must apply the 180 deg ambiguity order.
        yaws = sorted(c.pose_in_world.yaw_cw_deg % 360.0 for c in result.candidates)
        self.assertEqual(len(yaws), 4)

    def test_wrong_configured_extent_is_rejected(self) -> None:
        """Interior clutter must not be accepted as a field-sized rectangle."""

        points = rectangle_points(500.0, 400.0, robot=(250.0, 200.0, 0.0))
        result = bootstrap_rectangle(
            points, field_width_cm=1000.0, field_height_cm=1000.0
        )
        self.assertFalse(result.geometry_valid)
        self.assertEqual(result.candidate_count, 0)


class PartialVisibilityTests(unittest.TestCase):
    def test_two_perpendicular_edges_are_enough_geometry(self) -> None:
        points = rectangle_points(1000.0, 1000.0, robot=(200.0, 200.0, 0.0),
                                  sides=("back", "right"))
        result = bootstrap_rectangle(
            points, field_width_cm=1000.0, field_height_cm=1000.0
        )
        # Two edges cannot fix the position along either wall, so no full
        # bootstrap: the result must say so instead of inventing a pose.
        self.assertFalse(result.geometry_valid)
        self.assertIn(
            result.reason,
            {"single_axis_position_unobservable", "incomplete_rectangle"},
        )
        self.assertEqual(result.candidate_count, 0)

    def test_single_edge_cannot_bootstrap(self) -> None:
        points = rectangle_points(1000.0, 1000.0, robot=(200.0, 200.0, 0.0),
                                  sides=("back",))
        result = bootstrap_rectangle(
            points, field_width_cm=1000.0, field_height_cm=1000.0
        )
        self.assertFalse(result.geometry_valid)
        self.assertEqual(result.candidate_count, 0)


class ClutterRobustnessTests(unittest.TestCase):
    """Interior boxes and short walls must not be mistaken for the boundary."""

    def test_interior_clutter_does_not_beat_the_real_boundary(self) -> None:
        points = rectangle_points(1000.0, 1000.0, robot=(500.0, 500.0, 0.0))
        # A dense interior box edge that is closer to the robot than the walls.
        clutter = []
        for i in range(200):
            clutter.append((i * 0.5 - 50.0, -260.0))
            clutter.append((-260.0, i * 0.5 - 50.0))
        result = bootstrap_rectangle(
            points + clutter, field_width_cm=1000.0, field_height_cm=1000.0
        )
        self.assertTrue(result.geometry_valid, result.reason)
        self.assertAlmostEqual(result.observed_width_cm, 1000.0, delta=80.0)
        self.assertAlmostEqual(result.observed_height_cm, 1000.0, delta=80.0)

    def test_random_noise_only_does_not_bootstrap(self) -> None:
        import random

        rng = random.Random(7)
        noise = [(rng.uniform(-500, 500), rng.uniform(-500, 500))
                 for _ in range(400)]
        result = bootstrap_rectangle(
            noise, field_width_cm=1000.0, field_height_cm=1000.0
        )
        self.assertFalse(result.geometry_valid)


class ResolverTests(unittest.TestCase):
    def _ambiguous(self):
        points = rectangle_points(1000.0, 1000.0, robot=(500.0, 500.0, 0.0))
        return bootstrap_rectangle(
            points, field_width_cm=1000.0, field_height_cm=1000.0
        )

    def test_without_a_prior_the_ambiguity_is_preserved(self) -> None:
        result = self._ambiguous()
        resolved, reason = resolve_bootstrap_candidates(result, None)
        self.assertIs(resolved, result)
        self.assertEqual(reason, "no_global_prior")
        self.assertFalse(resolved.orientation_resolved)
        self.assertEqual(resolved.candidate_count, 4)

    def test_a_real_prior_selects_exactly_one_candidate(self) -> None:
        result = self._ambiguous()

        class FixedHeadingPrior:
            """Stands in for a rule-mandated fixed start heading."""

            def __init__(self, target_cw_deg):
                self.target = target_cw_deg

            def select_candidate(self, candidates):
                return min(
                    candidates,
                    key=lambda c: abs(
                        ((c.pose_in_world.yaw_cw_deg - self.target) + 180.0)
                        % 360.0 - 180.0
                    ),
                )

        resolved, reason = resolve_bootstrap_candidates(
            result, FixedHeadingPrior(0.0)
        )
        self.assertEqual(reason, "resolved_by_prior")
        self.assertTrue(resolved.orientation_resolved)
        self.assertEqual(resolved.candidate_count, 1)
        self.assertAlmostEqual(
            resolved.candidates[0].pose_in_world.yaw_cw_deg % 360.0, 0.0,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
