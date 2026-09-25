from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.navigation_common import NavigationGoal, NavigationGrid
from components.pose_fusion import PoseFusionState
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from core.types import Pose2D, PoseQuality
from robocup_runtime import RobocupMissionState, build_runtime


class PoseLossTimeoutTests(unittest.TestCase):
    def make_runtime(self):
        source = load_v2_config()
        config = replace(
            source,
            fusion=replace(source.fusion, t265_max_age_s=0.1, d500_max_age_s=0.1),
            safety=replace(source.safety, stop_if_pose_lost_s=0.5),
            t265=replace(source.t265, enabled=False),
            d500=replace(source.d500, enabled=False),
        )
        runtime = build_runtime(
            config,
            RuntimeMode.DRY_RUN,
            clock=lambda: 10.0,
            world=NavigationGrid(100, 100, 0.1, origin_x_m=-5.0, origin_y_m=-5.0),
        )
        runtime.fusion.update_t265(Pose2D(0.0, 0.0, 0.0, 10.0), PoseQuality("t265", True, False))
        runtime.fusion.update_d500(Pose2D(0.0, 0.0, 0.0, 10.0), PoseQuality("d500", True, False))
        runtime.mission.set_navigation_goal(NavigationGoal(2.0, 0.0))
        runtime.start()
        return runtime

    def test_short_pose_loss_stops_motion_without_final_safe_stop(self) -> None:
        runtime = self.make_runtime()
        runtime.step(now_s=10.0)
        result = runtime.step(now_s=10.3)
        self.assertIs(result.estimate.state, PoseFusionState.LOST)
        self.assertIs(result.mission_state, RobocupMissionState.NAVIGATING)
        self.assertEqual(result.command.linear_x_m_s, 0.0)
        self.assertTrue(runtime.drive.backend.stopped)
        runtime.close()

    def test_timeout_safe_stops_and_recovery_resets_loss_timer(self) -> None:
        runtime = self.make_runtime()
        runtime.step(now_s=10.0)
        runtime.step(now_s=10.3)  # 0.2 s beyond the last sensor freshness window.
        runtime.fusion.update_t265(Pose2D(0.05, 0.0, 0.0, 10.35), PoseQuality("t265", True, False))
        runtime.fusion.update_d500(Pose2D(0.05, 0.0, 0.0, 10.35), PoseQuality("d500", True, False))
        recovered = runtime.step(now_s=10.35)
        self.assertIsNot(recovered.estimate.state, PoseFusionState.LOST)

        short = runtime.step(now_s=10.6)
        self.assertIs(short.mission_state, RobocupMissionState.NAVIGATING)
        still_waiting = runtime.step(now_s=10.9)
        self.assertIs(still_waiting.mission_state, RobocupMissionState.NAVIGATING)
        final = runtime.step(now_s=11.0)
        self.assertIs(final.mission_state, RobocupMissionState.SAFE_STOP)
        self.assertTrue(runtime.drive.backend.stopped)
        runtime.close()


if __name__ == "__main__":
    unittest.main()
