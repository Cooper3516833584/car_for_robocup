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


class HardwareWorldTests(unittest.TestCase):
    def test_hardware_runtime_builds_configured_world_and_navigates(self) -> None:
        source_config = load_v2_config()
        config = replace(
            source_config,
            calibration=replace(source_config.calibration, geometry_measured=True, sensor_extrinsics_measured=True),
            competition_map=replace(source_config.competition_map, measured=True),
            footprint=replace(source_config.footprint, measured=True),
            d500=replace(source_config.d500, enabled=False),
            t265=replace(source_config.t265, enabled=False),
        )

        def fake_drive_builder(cfg, **_kwargs):
            return build_differential_drive(cfg, fake=True, clock=lambda: 1.0)

        with patch.object(robocup_runtime, "build_differential_drive", side_effect=fake_drive_builder):
            runtime = robocup_runtime.build_runtime(config, RuntimeMode.HARDWARE_MISSION, clock=lambda: 1.0)

        self.assertIsNotNone(runtime.world)
        self.assertEqual(runtime.world.width, 200)
        self.assertEqual(runtime.world.height, 200)
        runtime.fusion.update_d500(Pose2D(1.0, 1.0, 0.0, 1.0), PoseQuality("d500", True, False))
        runtime.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 1.0), PoseQuality("t265", True, False))
        runtime.start()
        runtime.mission.set_navigation_goal(robocup_runtime.NavigationGoal(2.0, 1.0))
        result = runtime.step()
        self.assertIsNone(result.error)
        self.assertIsNotNone(result.navigation)
        self.assertNotEqual(result.navigation.state.value, "blocked")
        self.assertNotEqual(result.navigation.diagnostics.get("reason"), "map_unavailable")
        runtime.close()


if __name__ == "__main__":
    unittest.main()
