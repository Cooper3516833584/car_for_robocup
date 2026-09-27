"""Hardware-free tests for the wall-predictor coordinate conversion.

Regression guard for a real bug: ``tools/yaw_wall_crosscheck.py`` used to define
its wall frame as ``[-half, +half]`` and then add ``+half`` to the predicted
position as well.  That double-shifted the origin, so the tool reported "no
boundary observed" for reasons that had nothing to do with the field.

Two ``Pose2D`` types exist in this repository and must not be mixed:
``core.types.Pose2D`` is metres / CCW radians, ``radar_driver.Pose2D`` is
centimetres / CW degrees.
"""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_driver import Pose2D as RadarPose2D  # noqa: E402
from components.radar_pose_adapter import (  # noqa: E402
    RadarPoseAdapter,
    compose_pose_with_delta_prior,
)
from core.types import Pose2D as CorePose2D  # noqa: E402


class WallPredictorConversionTests(unittest.TestCase):
    def test_zero_delta_returns_the_start_prior_exactly(self) -> None:
        """With no motion the predictor must be the prior, with no extra shift."""

        prior = RadarPose2D(250.0, 250.0, 0.0)
        got = compose_pose_with_delta_prior(prior, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(got.x_cm, 250.0, places=9)
        self.assertAlmostEqual(got.y_cm, 250.0, places=9)
        self.assertAlmostEqual(got.yaw_cw_deg, 0.0, places=9)

    def test_no_half_field_offset_is_added(self) -> None:
        """Regression: a 500 cm field with the car at its centre predicts centre.

        Adding half the field size would predict the corner instead.
        """

        prior = RadarPose2D(250.0, 250.0, 0.0)
        got = compose_pose_with_delta_prior(prior, 0.0, 0.0, 0.0)
        self.assertNotAlmostEqual(got.x_cm, 500.0, places=6)
        self.assertNotAlmostEqual(got.y_cm, 500.0, places=6)

    def test_forward_delta_moves_along_the_prior_heading(self) -> None:
        prior = RadarPose2D(100.0, 100.0, 0.0)
        got = compose_pose_with_delta_prior(prior, 1.0, 0.0, 0.0)
        self.assertAlmostEqual(got.x_cm, 200.0, places=6)
        self.assertAlmostEqual(got.y_cm, 100.0, places=6)

    def test_prior_yaw_rotates_the_delta(self) -> None:
        """A prior heading of +90 deg CCW must send forward motion along +Y."""

        prior = RadarPose2D(0.0, 0.0, -90.0)   # CW-positive field: -90 == +90 CCW
        got = compose_pose_with_delta_prior(prior, 1.0, 0.0, 0.0)
        self.assertAlmostEqual(got.x_cm, 0.0, places=6)
        self.assertAlmostEqual(got.y_cm, 100.0, places=6)

    def test_yaw_delta_accumulates_in_ccw_convention(self) -> None:
        prior = RadarPose2D(0.0, 0.0, 0.0)
        got = compose_pose_with_delta_prior(prior, 0.0, 0.0, math.radians(30.0))
        # +30 deg CCW becomes -30 in the CW-positive radar convention.
        self.assertAlmostEqual(got.yaw_cw_deg, -30.0, places=6)

    def test_round_trip_between_conventions_is_consistent(self) -> None:
        """core (m, CCW rad) -> radar (cm, CW deg) must round-trip."""

        adapter = RadarPoseAdapter()
        core = CorePose2D(2.5, -1.2, math.radians(35.0), 0.0)
        radar = adapter.from_map_base_pose(core)
        self.assertAlmostEqual(radar.x_cm, 250.0, places=9)
        self.assertAlmostEqual(radar.y_cm, -120.0, places=9)
        self.assertAlmostEqual(radar.yaw_cw_deg, -35.0, places=9)
        # And back again through the existing forward conversion.
        back = adapter.to_map_base_pose(radar, timestamp_s=0.0)
        self.assertAlmostEqual(back.x_m, core.x_m, places=9)
        self.assertAlmostEqual(back.y_m, core.y_m, places=9)
        self.assertAlmostEqual(back.yaw_rad, core.yaw_rad, places=9)

    def test_non_finite_inputs_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            compose_pose_with_delta_prior(
                RadarPose2D(0.0, 0.0, 0.0), float("nan"), 0.0, 0.0
            )
        with self.assertRaises(ValueError):
            RadarPoseAdapter().from_map_base_pose(
                CorePose2D(float("inf"), 0.0, 0.0, 0.0)
            )


if __name__ == "__main__":
    unittest.main()
