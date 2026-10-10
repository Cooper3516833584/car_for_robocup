"""Offline acceptance of the CH3 0 deg visual drop alignment.

Two layers are covered separately: the packaged reference photograph really
yields the outer-ring arc (not the frame centre, not the gold baffle), and the
closed alignment loop keeps correcting abnormal pixel errors instead of
declaring a failed drop, falling back to the fused 47 cm point only when the
0 deg view is genuinely unusable.
"""

from contextlib import ExitStack
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, PropertyMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import competition_task as task
import payload_detour
from components import drop_target_vision as dvision
from components.pose_fusion import FusedPoseEstimate, PoseFusion, PoseFusionState
from components.relay_lcus import FakeLCUSRelay
from components.payload_task import prepare_payload
from config.v2_runtime import RuntimeMode
from core.types import Pose2D
from robocup_runtime import build_runtime, load_runtime_config

REFERENCE = (300.0, 300.0)
ROAD_GAIN = -800.0   # px per metre of lateral displacement along the road axis
ALONG_GAIN = 600.0   # px per metre of displacement towards the target


class ChassisModel:
    """Fused chassis plus 0 deg image model used only by these tests.

    ``error_px`` is the pixel error the 0 deg frame reports at the end of the
    coarse approach, so each test states its deviation directly: a positive X is
    "target right of the reference" and a positive Y is "arc below it".
    """

    def __init__(self, pose=(0.0, 0.0, math.pi / 2), error_px=(40.0, -36.0), *,
                 road_gain=ROAD_GAIN, along_gain=ALONG_GAIN, visible=True):
        self.pose = [float(pose[0]), float(pose[1]), float(pose[2])]
        self.start = (self.pose[0], self.pose[1])
        self.road_yaw = self.pose[2] - math.pi / 2
        self.side_yaw = self.pose[2]
        self.road_gain, self.along_gain, self.visible = road_gain, along_gain, visible
        self.stages, self.lines, self.arc_calls = [], [], 0
        # Pose whose 0 deg frame equals the reference feature: built so that the
        # coarse-approach endpoint carries exactly ``error_px``.
        lateral = -error_px[0] / road_gain
        reach = task.DROP_COARSE_M - error_px[1] / along_gain
        self.ideal = (self.start[0] + lateral * math.cos(self.road_yaw)
                                   + reach * math.cos(self.side_yaw),
                      self.start[1] + lateral * math.sin(self.road_yaw)
                                   + reach * math.sin(self.side_yaw))

    def arc(self):
        self.arc_calls += 1
        if not self.visible:
            return None
        dx, dy = self.pose[0] - self.ideal[0], self.pose[1] - self.ideal[1]
        lateral = dx * math.cos(self.road_yaw) + dy * math.sin(self.road_yaw)
        along = dx * math.cos(self.side_yaw) + dy * math.sin(self.side_yaw)
        return (REFERENCE[0] + self.road_gain * lateral,
                REFERENCE[1] + self.along_gain * along, 450.0)

    def move(self, start, end, reverse=False):
        current = (round(self.pose[0], 9), round(self.pose[1], 9))
        if current != (round(start[0], 9), round(start[1], 9)):
            raise AssertionError(f"move did not start at the current fused pose: "
                                 f"{start} != {self.pose}")
        self.lines.append((start, end, bool(reverse)))
        self.pose[0], self.pose[1] = end


class DropAlignLoopTests(unittest.TestCase):
    """The 0 deg loop corrects, degrades and never invents a hard failure."""

    def setUp(self):
        self.config = load_runtime_config()
        self.make()

    def make(self, *, visible=True, road_gain=ROAD_GAIN, along_gain=ALONG_GAIN,
             error_px=(40.0, -36.0)):
        self.events = []
        self.caps = []
        self.chassis = ChassisModel(error_px=error_px, road_gain=road_gain,
                                    along_gain=along_gain, visible=visible)
        self.camera = Mock()
        self.camera.read.return_value = (True, object())
        self.runtime = SimpleNamespace(motion=SimpleNamespace(drive=self.config.drive),
                                       record_event=self.record)
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.stack.enter_context(patch.object(dvision, "read_reference", return_value=REFERENCE))
        self.extract = self.stack.enter_context(patch.object(dvision, "extract_target_arc"))
        self.extract.side_effect = lambda frame, color="yellow": self.chassis.arc()
        self.opener = self.stack.enter_context(
            patch.object(task, "_open_yellow_camera", return_value=(self.camera, False)))
        return self

    def record(self, event, **values):
        self.events.append((event, values))

    def events_of(self, name):
        return [values for event, values in self.events if event == name]

    def motion(self, label, method, *args, **kwargs):
        self.chassis.stages.append((label, method))
        self.caps.append(self.runtime.motion.drive.max_linear_speed_m_s)
        if method == "track_global_line":
            self.chassis.move(args[0], args[1], kwargs.get("reverse", False))
        elif method == "rotate_to":
            self.chassis.pose[2] = args[0]
        else:
            raise AssertionError(f"unexpected motion method: {method}")
        return SimpleNamespace(x_m=self.chassis.pose[0], y_m=self.chassis.pose[1],
                               yaw_rad=self.chassis.pose[2])

    def align(self, **kwargs):
        side_pose = SimpleNamespace(x_m=self.chassis.pose[0], y_m=self.chassis.pose[1],
                                    yaw_rad=self.chassis.pose[2])
        return task.align_drop_position(self.runtime, self.camera, side_pose,
                                        self.chassis.road_yaw, self.chassis.side_yaw,
                                        self.motion, **kwargs)

    def test_visual_alignment_converges_to_the_reference_pose(self):
        pose = self.align()
        self.assertIsNotNone(pose)
        done = self.events_of("drop_align_done")[-1]
        self.assertTrue(done["success"])
        self.assertEqual(done["source"], "visual")
        stages = [label for label, _ in self.chassis.stages]
        self.assertEqual(stages[0], "coarse_approach")
        self.assertIn("align_turn_road", stages)
        self.assertIn("align_shift_road", stages)
        self.assertIn("align_turn_side", stages)
        self.assertIn("align_along", stages)
        self.assertAlmostEqual(math.dist(self.chassis.lines[0][0], self.chassis.lines[0][1]),
                               task.DROP_COARSE_M, delta=1e-6)
        steps = self.events_of("drop_align_step")
        self.assertTrue(all(step["success"] is False for step in steps[:-1]))
        self.assertTrue(steps[-1]["success"])
        self.assertLess(abs(self.chassis.pose[0] - self.chassis.ideal[0]), 0.02)
        self.assertLess(abs(self.chassis.pose[1] - self.chassis.ideal[1]), 0.02)

    def test_abnormal_pixel_errors_keep_correcting_without_a_safe_stop(self):
        for error in ((30.0, 0.0), (80.0, 0.0), (0.0, 80.0), (80.0, 80.0)):
            with self.subTest(error=error):
                self.make(error_px=error)
                pose = self.align()
                self.assertIsNotNone(pose)
                steps = self.events_of("drop_align_step")
                self.assertGreaterEqual(len(steps), 3)
                self.assertTrue(any(abs(step["error_x_px"]) > task.DROP_X_TOL_PX
                                    or abs(step["error_y_px"]) > task.DROP_Y_TOL_PX
                                    for step in steps))
                self.assertTrue(self.events_of("drop_align_done")[-1]["success"])

    def test_exhausted_correction_loop_still_uses_the_best_observed_pose(self):
        self.stack.enter_context(patch.object(task, "DROP_FINE_MAX_ACTIONS", 2))
        self.make(error_px=(400.0, 0.0))
        pose = self.align()
        self.assertIsNotNone(pose)
        done = self.events_of("drop_align_done")[-1]
        self.assertFalse(done["success"])
        self.assertEqual(done["fallback"], "best_pose")
        # Two 2 cm corrections were made, then the best observed pose was kept.
        self.assertLess(done["score_px"], 400.0)
        self.assertAlmostEqual(done["pose"].x_m, 0.02, delta=0.011)

    def test_inverted_mount_flips_each_axis_once_and_then_converges(self):
        self.make(road_gain=-ROAD_GAIN, along_gain=-ALONG_GAIN, error_px=(24.0, -24.0))
        self.align()
        flips = {values["axis"] for values in self.events_of("drop_align_sign_flip")}
        self.assertEqual(flips, {"x", "y"})
        self.assertTrue(self.events_of("drop_align_done")[-1]["success"])

    def test_missing_arc_creeps_to_the_original_fused_lateral_distance(self):
        self.make(visible=False)
        start = (self.chassis.pose[0], self.chassis.pose[1])
        pose = self.align()
        stages = [label for label, _ in self.chassis.stages]
        self.assertEqual(stages[0], "coarse_approach")
        self.assertTrue(all(label == "creep_to_fallback" for label in stages[1:]),
                        f"unexpected stages: {stages}")
        self.assertAlmostEqual(math.dist(start, (pose.x_m, pose.y_m)),
                               task.DROP_FALLBACK_M, delta=1e-6)
        self.assertAlmostEqual(pose.x_m - start[0], 0.0, delta=1e-6)
        self.assertAlmostEqual(pose.y_m - start[1], task.DROP_FALLBACK_M, delta=1e-6)
        done = self.events_of("drop_align_done")[-1]
        self.assertFalse(done["success"])
        self.assertEqual(done["fallback"], "fused_47cm")
        self.assertGreater(self.camera.read.call_count, 0)

    def test_unavailable_camera_uses_one_fused_47cm_move(self):
        self.make(visible=False)
        self.opener.side_effect = OSError("no /dev/video0")
        pose = self.align()
        self.assertEqual([label for label, _ in self.chassis.stages],
                         ["coarse_approach", "fallback_47cm"])
        self.assertAlmostEqual(math.dist(self.chassis.lines[1][0], self.chassis.lines[1][1]),
                               task.DROP_FALLBACK_M - task.DROP_COARSE_M, delta=1e-6)
        self.assertTrue(self.events_of("drop_align_camera_failed"))
        self.assertFalse(self.events_of("drop_align_step"))
        self.assertIsNotNone(pose)

    def test_unreadable_reference_photo_falls_back_without_a_camera(self):
        self.stack.enter_context(patch.object(dvision, "read_reference", return_value=None))
        self.align()
        self.assertEqual([label for label, _ in self.chassis.stages],
                         ["coarse_approach", "fallback_47cm"])
        self.opener.assert_not_called()
        self.assertEqual(self.events_of("drop_align_done")[-1]["fallback"], "fused_47cm")

    def test_other_colour_never_uses_the_yellow_ch3_photograph(self):
        self.stack.enter_context(patch.object(dvision, "read_reference",
                                              side_effect=AssertionError("yellow reference used")))
        pose = self.align(color="red")
        self.assertIsNotNone(pose)
        self.opener.assert_not_called()
        self.assertEqual(self.events_of("drop_align_done")[-1]["fallback"], "fused_47cm")

    def test_fine_moves_are_speed_capped_for_the_loop_and_the_drive_is_restored(self):
        self.align()
        self.assertIs(self.runtime.motion.drive, self.config.drive)
        # Only the coarse approach runs on the route drive; every refinement
        # turn and micro-move runs on the dedicated fine speed.
        self.assertEqual(self.caps[0], self.config.drive.max_linear_speed_m_s)
        self.assertTrue(all(cap == task.DROP_FINE_SPEED_M_S for cap in self.caps[1:]))
        self.assertTrue(len(self.chassis.lines[1:]) > 3)

    def test_driver_failure_still_propagates(self):
        calls = {"count": 0}

        def failing(label, method, *args, **kwargs):
            calls["count"] += 1
            if calls["count"] > 1:
                raise RuntimeError("drive backend failed")
            return self.motion(label, method, *args, **kwargs)

        side_pose = SimpleNamespace(x_m=self.chassis.pose[0], y_m=self.chassis.pose[1],
                                    yaw_rad=self.chassis.side_yaw)
        with self.assertRaisesRegex(RuntimeError, "drive backend failed"):
            task.align_drop_position(self.runtime, self.camera, side_pose,
                                     self.chassis.road_yaw, self.chassis.side_yaw, failing)
        self.assertIs(self.runtime.motion.drive, self.config.drive)


class ReferencePhotoTests(unittest.TestCase):
    """The packaged 0 deg photograph is the only alignment reference."""

    @classmethod
    def setUpClass(cls):
        import cv2
        cls.image = cv2.imread(str(task.DROP_REFERENCE))
        if cls.image is None:
            raise AssertionError(f"missing packaged reference {task.DROP_REFERENCE}")

    def test_arc_is_the_outer_ring_not_the_centre_or_the_gold_baffle(self):
        arc = dvision.read_reference(task.DROP_REFERENCE)
        self.assertIsNotNone(arc)
        apex_x, apex_y, width = arc
        self.assertAlmostEqual(apex_x, 297, delta=6)
        self.assertAlmostEqual(apex_y, 309, delta=6)
        self.assertAlmostEqual(width, 450, delta=40)
        # The correct drop position is not the image centre, and the arc apex
        # sits far above the gold CH3 baffle at the bottom of the frame.
        self.assertGreater(abs(apex_x - 320), 8)
        self.assertLess(apex_y, 360)

    def test_pixel_translation_moves_the_feature_by_the_same_pixels(self):
        import cv2
        import numpy as np
        moved = cv2.warpAffine(self.image, np.float32([[1, 0, 15], [0, 1, 10]]), (640, 480))
        base = dvision.extract_target_arc(self.image)
        shifted = dvision.extract_target_arc(moved)
        self.assertAlmostEqual(shifted[0] - base[0], 15, delta=3)
        self.assertAlmostEqual(shifted[1] - base[1], 10, delta=3)
        self.assertEqual(dvision.reference_error(shifted, base),
                         (shifted[0] - base[0], shifted[1] - base[1]))
        self.assertEqual(dvision.reference_error(base, base), (0, 0))

    def test_other_colours_and_unusable_frames_have_no_arc(self):
        for color in ("red", "blue", "green", "magenta"):
            self.assertIsNone(dvision.extract_target_arc(self.image, color))
        self.assertIsNone(dvision.extract_target_arc(None, "yellow"))
        self.assertIsNone(dvision.read_reference(task.DROP_REFERENCE.parent / "missing.jpg"))


class DetourFineAlignTests(unittest.TestCase):
    """The detour returns from the real drop pose, never from a fixed 47 cm."""

    def fixture(self, pose=(5., 7., math.pi / 2)):
        self.now, self.steps = 10., 0
        self.pose = list(pose)
        self.releasing = False
        self.stage_poses, self.events, self.releases = {}, [], []
        self.runtime = build_runtime(load_runtime_config(), RuntimeMode.DRY_RUN,
                                     clock=lambda: self.now)
        self.addCleanup(self.runtime.close)
        self.relay = FakeLCUSRelay(4)
        self.runtime.relay = self.relay
        self.runtime._consume_t265 = lambda *_a, **_k: None
        self.runtime._consume_d500 = lambda *_a, **_k: None
        self.runtime.fusion.estimate = lambda now: FusedPoseEstimate(
            Pose2D(*self.pose, now), PoseFusionState.OK, 0., ("test",), 0., 0.,
            None, None, True, anchor_initialized=True, t265_confidence=1.)
        local = patch.object(PoseFusion, "continuous_t265_pose", new_callable=PropertyMock)
        prop = local.start()
        self.addCleanup(local.stop)
        prop.side_effect = lambda: Pose2D(*self.pose, self.now)
        self.runtime.record_event = self.record
        self.runtime.start()
        self.runtime.motion.stop()

    def record(self, event, **values):
        self.events.append((event, values))
        if event == "payload_release_start":
            self.releases.append((values["channel"], tuple(self.pose)))
        if event == "payload_detour_stage_done":
            self.stage_poses[values["stage"]] = values["pose"]

    def advance(self, dt):
        self.steps += 1
        self.assertLess(self.steps, 15000)
        twist = self.runtime.drive.last_limited_twist
        yaw = self.pose[2] + twist.angular_z_rad_s * dt / 2
        self.pose[0] += twist.linear_x_m_s * math.cos(yaw) * dt
        self.pose[1] += twist.linear_x_m_s * math.sin(yaw) * dt
        self.pose[2] += twist.angular_z_rad_s * dt
        self.now += dt

    def fine_align(self, side_pose, road_yaw, side_yaw, motion):
        """Stand-in for the visual callback: 55 cm approach plus a 3 cm shift."""
        start = (side_pose.x_m, side_pose.y_m)
        far = (start[0] + .55 * math.cos(side_yaw), start[1] + .55 * math.sin(side_yaw))
        motion("coarse_approach", "track_global_line", start, far)
        turned = motion("align_turn_road", "rotate_to", road_yaw)
        here = (turned.x_m, turned.y_m)
        shifted = (here[0] + .03 * math.cos(road_yaw), here[1] + .03 * math.sin(road_yaw))
        motion("align_shift_road", "track_global_line", here, shifted)
        return motion("align_turn_side", "rotate_to", side_yaw)

    def run_detour(self, fine_align):
        return payload_detour.run_payload_detour(
            self.runtime, payload_detour.DetourSettings(payload_slot=2),
            clock=lambda: self.now, sleep=self.advance, fine_align=fine_align)

    def test_return_leg_reuses_the_real_drop_pose_and_restores_road_yaw(self):
        self.fixture()
        self.assertTrue(prepare_payload(self.relay, 2))
        returned = self.run_detour(self.fine_align)
        starts = {values["stage"]: values for event, values in self.events
                  if event == "payload_detour_stage_start"}
        self.assertIn("advance_7cm", starts)
        self.assertIn("coarse_approach", starts)
        self.assertNotIn("forward_47cm", starts)
        turn_xy = starts["coarse_approach"]["args"][0]
        drop_xy = starts["return_from_drop"]["args"][0]
        self.assertEqual(starts["return_from_drop"]["args"][1], turn_xy)
        self.assertTrue(starts["return_from_drop"]["kwargs"]["reverse"])
        self.assertNotAlmostEqual(math.dist(drop_xy, turn_xy), .47, delta=.05)
        self.assertGreater(math.dist(drop_xy, turn_xy), .5)
        self.assertAlmostEqual(returned.yaw_rad, math.pi / 2, delta=.06)
        self.assertEqual([channel for channel, _ in self.releases], [3])
        self.assertFalse(any(self.relay.query_status().values()))

    def test_zero_length_return_is_skipped_without_a_line_error(self):
        self.fixture()
        self.assertTrue(prepare_payload(self.relay, 2))

        def back_to_turn(side_pose, road_yaw, side_yaw, motion):
            start = (side_pose.x_m, side_pose.y_m)
            motion("coarse_approach", "track_global_line", start,
                   (start[0] + task.DROP_COARSE_M * math.cos(side_yaw),
                    start[1] + task.DROP_COARSE_M * math.sin(side_yaw)))
            return side_pose

        self.run_detour(back_to_turn)
        self.assertTrue(any(event == "payload_detour_stage_skipped" for event, _ in self.events))
        self.assertEqual([channel for channel, _ in self.releases], [3])
        self.assertEqual(self.runtime.drive.last_limited_twist.linear_x_m_s, 0)


if __name__ == "__main__":
    unittest.main()
