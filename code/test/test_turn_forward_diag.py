"""Exercise the diagnostic's turn/straight handoff through the real runtime."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.pose_fusion import FusedPoseEstimate, PoseFusionState
from config.v2_runtime import RuntimeMode
from core.types import Pose2D
from robocup_runtime import build_runtime, load_runtime_config
from tools.turn_forward_diag import run


class TurnForwardDiagnosticTests(unittest.TestCase):
    def test_completed_turn_resumes_mission_and_actually_drives_forward(self):
        now = [10.0]
        pose = [0.0, 0.0, 0.0]
        events = []
        forward_commands = []
        config = load_runtime_config()
        runtime = build_runtime(config, RuntimeMode.DRY_RUN, clock=lambda: now[0])
        self.addCleanup(runtime.close)
        runtime._consume_t265 = lambda *_a, **_k: None
        runtime._consume_d500 = lambda *_a, **_k: None
        runtime.fusion.estimate = lambda stamp: FusedPoseEstimate(
            Pose2D(*pose, stamp), PoseFusionState.OK, 0.0,
            ("t265", "slam", "fused"), 0.0, 0.0, None, None, True,
            anchor_initialized=True, t265_confidence=1.0,
        )
        runtime.record_event = lambda event, **data: events.append((event, data))
        start_runtime = runtime.start

        def start_without_default_dry_run_goal():
            start_runtime()
            runtime.motion.stop()

        runtime.start = start_without_default_dry_run_goal

        def advance(dt):
            twist = runtime.drive.last_limited_twist
            if twist.linear_x_m_s > 0:
                forward_commands.append(twist.linear_x_m_s)
            yaw = pose[2] + twist.angular_z_rad_s * dt / 2
            pose[0] += twist.linear_x_m_s * math.cos(yaw) * dt
            pose[1] += twist.linear_x_m_s * math.sin(yaw) * dt
            pose[2] += twist.angular_z_rad_s * dt
            now[0] += dt

        result = run(runtime, config, abort=lambda: False,
                     clock=lambda: now[0], sleep=advance)
        self.assertTrue(forward_commands)
        self.assertGreater(result["straight_along_m"], 0.17)
        self.assertLessEqual(result["straight_along_m"], 0.20)
        self.assertLess(abs(result["straight_lateral_m"]), 0.005)
        self.assertEqual([data["stage"] for event, data in events
                          if event == "turn_forward_diag_stage_done"],
                         ["left_90deg", "forward_20cm"])
        self.assertEqual(runtime.drive.last_limited_twist.linear_x_m_s, 0.0)
        self.assertEqual(runtime.drive.last_limited_twist.angular_z_rad_s, 0.0)


if __name__ == "__main__":
    unittest.main()
