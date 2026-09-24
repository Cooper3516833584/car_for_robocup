from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.differential_kinematics import (
    DifferentialGeometry,
    DifferentialKinematics,
    scale_wheels_to_limit,
)
from core import Twist2D, WheelSpeeds


class DifferentialKinematicsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.kinematics = DifferentialKinematics(DifferentialGeometry(0.20))

    def assertWheelsAlmostEqual(self, actual: WheelSpeeds, expected: WheelSpeeds) -> None:
        self.assertAlmostEqual(actual.left_m_s, expected.left_m_s, places=12)
        self.assertAlmostEqual(actual.right_m_s, expected.right_m_s, places=12)

    def assertTwistAlmostEqual(self, actual: Twist2D, expected: Twist2D) -> None:
        self.assertAlmostEqual(actual.linear_x_m_s, expected.linear_x_m_s, delta=1e-9)
        self.assertAlmostEqual(actual.angular_z_rad_s, expected.angular_z_rad_s, delta=1e-9)

    def test_straight_line(self) -> None:
        wheels = self.kinematics.twist_to_wheels(Twist2D(0.5, 0.0))
        self.assertWheelsAlmostEqual(wheels, WheelSpeeds(0.5, 0.5))

    def test_in_place_left_rotation(self) -> None:
        wheels = self.kinematics.twist_to_wheels(Twist2D(0.0, 1.0))
        self.assertWheelsAlmostEqual(wheels, WheelSpeeds(-0.1, 0.1))

    def test_left_turn_arc(self) -> None:
        wheels = self.kinematics.twist_to_wheels(Twist2D(0.4, 1.0))
        self.assertWheelsAlmostEqual(wheels, WheelSpeeds(0.3, 0.5))

    def test_positive_and_negative_yaw_directions(self) -> None:
        left_turn = self.kinematics.twist_to_wheels(Twist2D(0.3, 0.5))
        right_turn = self.kinematics.twist_to_wheels(Twist2D(0.3, -0.5))

        self.assertGreater(left_turn.right_m_s, left_turn.left_m_s)
        self.assertGreater(right_turn.left_m_s, right_turn.right_m_s)
        self.assertGreater(self.kinematics.wheels_to_twist(left_turn).angular_z_rad_s, 0.0)
        self.assertLess(self.kinematics.wheels_to_twist(right_turn).angular_z_rad_s, 0.0)

    def test_multiple_twists_round_trip(self) -> None:
        for linear, angular in (
            (0.0, 0.0), (0.5, 0.0), (0.0, 1.0), (0.4, 1.0),
            (-0.25, 0.8), (0.73, -1.25), (-0.1, -0.5),
        ):
            with self.subTest(linear=linear, angular=angular):
                original = Twist2D(linear, angular)
                self.assertTwistAlmostEqual(
                    self.kinematics.wheels_to_twist(
                        self.kinematics.twist_to_wheels(original)
                    ),
                    original,
                )

    def test_wheel_saturation_preserves_proportion(self) -> None:
        result = scale_wheels_to_limit(WheelSpeeds(1.0, 0.5), 0.5)
        self.assertWheelsAlmostEqual(result, WheelSpeeds(0.5, 0.25))

    def test_saturation_handles_reverse_and_counter_rotation(self) -> None:
        result = scale_wheels_to_limit(WheelSpeeds(-2.0, 1.0), 0.5)
        self.assertWheelsAlmostEqual(result, WheelSpeeds(-0.5, 0.25))

    def test_invalid_geometry_and_non_finite_inputs_are_rejected(self) -> None:
        for width in (0.0, -0.1, float("nan"), float("inf")):
            with self.subTest(width=width), self.assertRaises(ValueError):
                DifferentialGeometry(width)
        with self.assertRaisesRegex(ValueError, "linear_x_m_s"):
            self.kinematics.twist_to_wheels(Twist2D(float("nan"), 0.0))
        with self.assertRaisesRegex(ValueError, "right_m_s"):
            self.kinematics.wheels_to_twist(WheelSpeeds(0.0, float("inf")))

    def test_invalid_saturation_limit_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            scale_wheels_to_limit(WheelSpeeds(0.1, 0.2), 0.0)


if __name__ == "__main__":
    unittest.main()
