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
        self.exercise_handoff(0.20)

    def test_configurable_hold_keeps_drive_stopped_and_sensors_live(self):
        self.exercise_handoff(1.0)

    def test_straight_only_baseline_never_commands_a_turn(self):
        self.exercise_handoff(1.0, skip_turn=True)

    def test_right_turn_comparison_resumes_forward_and_stops(self):
        self.exercise_handoff(1.0, turn_direction="right")

    def test_47cm_comparison_keeps_same_bounded_sequence(self):
        self.exercise_handoff(1.0, distance_m=0.47)

    def exercise_handoff(self, settle_s, skip_turn=False, turn_direction="left", distance_m=0.20):
        turn_stage = "%s_90deg" % turn_direction
        straight_stage = "forward_%dcm" % round(distance_m * 100)
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
        runtime.record_event = lambda event, **data: events.append((event, {**data, "time_s": now[0]}))
        start_runtime = runtime.start

        def start_without_default_dry_run_goal():
            start_runtime()
            runtime.motion.stop()

        runtime.start = start_without_default_dry_run_goal

        def advance(dt):
            twist = runtime.drive.last_limited_twist
            if skip_turn:
                self.assertAlmostEqual(twist.angular_z_rad_s, 0.0)
            if events and events[-1][1].get("stage") == turn_stage and events[-1][0].endswith("done"):
                self.assertEqual(twist.linear_x_m_s, 0.0)
                self.assertEqual(twist.angular_z_rad_s, 0.0)
            if twist.linear_x_m_s > 0:
                forward_commands.append(twist.linear_x_m_s)
            yaw = pose[2] + twist.angular_z_rad_s * dt / 2
            pose[0] += twist.linear_x_m_s * math.cos(yaw) * dt
            pose[1] += twist.linear_x_m_s * math.sin(yaw) * dt
            pose[2] += twist.angular_z_rad_s * dt
            now[0] += dt

        result = run(runtime, config, abort=lambda: False,
                     clock=lambda: now[0], sleep=advance,
                     settle_after_turn_s=settle_s, skip_turn=skip_turn,
                     turn_direction=turn_direction, distance_m=distance_m)
        self.assertTrue(forward_commands)
        self.assertGreater(result["straight_along_m"], distance_m-0.03)
        self.assertLessEqual(result["straight_along_m"], distance_m)
        self.assertLess(abs(result["straight_lateral_m"]), 0.005)
        self.assertEqual([data["stage"] for event, data in events
                          if event == "turn_forward_diag_stage_done"],
                         ([straight_stage] if skip_turn else [turn_stage, straight_stage]))
        self.assertEqual(runtime.drive.last_limited_twist.linear_x_m_s, 0.0)
        self.assertEqual(runtime.drive.last_limited_twist.angular_z_rad_s, 0.0)
        turn_done = next(data["time_s"] for event, data in events
                         if (event == "turn_forward_diag_turn_skipped" if skip_turn else
                             event.endswith("done") and data.get("stage") == turn_stage))
        forward_start = next(data["time_s"] for event, data in events
                             if event.endswith("start") and data.get("stage") == straight_stage)
        self.assertAlmostEqual(forward_start-turn_done, settle_s)
        if skip_turn:
            self.assertEqual(result["turn_yaw_rad"], 0.0)
        else:
            turn_sign = 1.0 if turn_direction == "left" else -1.0
            self.assertGreater(turn_sign * result["turn_yaw_rad"], math.radians(85))


if __name__ == "__main__":
    unittest.main()
