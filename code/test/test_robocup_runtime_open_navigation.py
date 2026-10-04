from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config.v2_factory import build_differential_drive
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from core.types import Pose2D, PoseQuality

import robocup_runtime


class HardwareOpenNavigationTests(unittest.TestCase):
    def test_hardware_runtime_navigates_from_arbitrary_relative_pose(self) -> None:
        source = load_v2_config()
        config = replace(
            source,
            d500=replace(source.d500, enabled=False),
            t265=replace(source.t265, enabled=False),
        )

        def fake_drive_builder(cfg, **_kwargs):
            return build_differential_drive(cfg, fake=True, clock=lambda: 1.0)

        with patch.object(robocup_runtime, "build_differential_drive", side_effect=fake_drive_builder):
            runtime = robocup_runtime.build_runtime(config, RuntimeMode.HARDWARE_MISSION, clock=lambda: 1.0)

        try:
            runtime.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("t265", True, False))
            runtime.fusion.update_d500(Pose2D(-12.0, 8.0, 0.0, 1.0), PoseQuality("d500", True, False))
            runtime.start()
            runtime.mission.set_navigation_goal(robocup_runtime.NavigationGoal(-11.0, 8.0))
            result = runtime.step()
            self.assertIsNone(result.error)
            self.assertIsNotNone(result.navigation)
            self.assertEqual(result.navigation.path, ((-12.0, 8.0), (-11.0, 8.0)))
        finally:
            runtime.close()


if __name__ == "__main__":
    unittest.main()
