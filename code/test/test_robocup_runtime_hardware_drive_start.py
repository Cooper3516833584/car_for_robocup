from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import FakeDriveBackend
from components.navigation_common import NavigationGoal, NavigationGrid
from config.v2_factory import build_differential_drive
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from core.types import Pose2D, PoseQuality
import robocup_runtime


class FakeClock:
    def __init__(self, value: float = 10.0) -> None:
        self.value = value

    def __call__(self) -> float:
        self.value += 0.001
        return self.value


class HardwareLazyDriveStartTests(unittest.TestCase):
    def test_hardware_first_nonzero_command_and_next_step_are_monotonic(self) -> None:
        source_config = load_v2_config()
        config = replace(
            source_config,
            calibration=replace(
                source_config.calibration,
                geometry_measured=True,
                sensor_extrinsics_measured=True,
            ),
            competition_map=replace(source_config.competition_map, measured=True),
            footprint=replace(source_config.footprint, measured=True),
            d500=replace(source_config.d500, enabled=False),
            t265=replace(source_config.t265, enabled=False),
        )
        clock = FakeClock()

        def fake_drive_builder(cfg, *, fake=False, clock=None, **_kwargs):
            return build_differential_drive(cfg, fake=True, clock=clock)

        with patch.object(robocup_runtime, "build_differential_drive", side_effect=fake_drive_builder):
            runtime = robocup_runtime.build_runtime(
                config,
                RuntimeMode.HARDWARE_MISSION,
                clock=clock,
                world=NavigationGrid(120, 120, 0.1, origin_x_m=-6.0, origin_y_m=-6.0),
            )

        self.assertFalse(runtime.drive.is_running)
        runtime.fusion.update_d500(Pose2D(0.0, 0.0, 0.0, clock()), PoseQuality("d500", True, False))
        runtime.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, clock()), PoseQuality("t265", True, False))
        runtime.mission.set_navigation_goal(NavigationGoal(2.0, 0.0))
        runtime.start()

        first = runtime.step()
        self.assertIsNone(first.error)
        self.assertTrue(runtime.drive.is_running)
        self.assertTrue(runtime.drive.backend.commands)
        first_wheels = runtime.drive.backend.commands[-1]
        self.assertIsNotNone(first_wheels.left_m_s)
        self.assertIsNotNone(first_wheels.right_m_s)
        self.assertGreater(abs(first_wheels.left_m_s) + abs(first_wheels.right_m_s), 0.0)

        second = runtime.step()
        self.assertIsNone(second.error)
        self.assertEqual(second.mission_state.value, "navigating")
        self.assertEqual(len(runtime.drive.backend.commands), 2)
        runtime.close()


if __name__ == "__main__":
    unittest.main()
