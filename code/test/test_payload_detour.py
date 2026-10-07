"""Closed-loop software acceptance of detour geometry, resume and safe exits."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import center_target_route as route
from components.pose_fusion import FusedPoseEstimate, PoseFusionState
from components.relay_lcus import FakeLCUSRelay
from components.payload_task import payload_channel, prepare_payload
from config.v2_runtime import RuntimeMode
from core.types import Pose2D
from payload_detour import DetourSettings, run_payload_detour
from robocup_runtime import RobocupMissionState, build_runtime, load_runtime_config
from tools.run_center_target_route import payload_config

BOX = (0, 0, 100, 100, .9, 0)  # Valid target outside the old central 50%.
CENTER_BOX = (290, 0, 350, 40, .9, 0)  # Horizontally centred, near the image top.


class PayloadDetourTests(unittest.TestCase):
    def fixture(self, pose=(0., 0., 0.), *, started=True):
        self.now, self.steps = 10., 0
        self.pose = list(pose)
        self.stage = None
        self.releasing = False
        self.stage_poses, self.events, self.releases = {}, [], []
        self.runtime = build_runtime(load_runtime_config(), RuntimeMode.DRY_RUN,
                                     clock=lambda: self.now)
        self.addCleanup(self.runtime.close)
        self.original_navigation = self.runtime.motion.navigation
        self.relay = FakeLCUSRelay(4)
        self.runtime.relay = self.relay
        self.runtime._consume_t265 = lambda *_a, **_k: None
        self.runtime._consume_d500 = lambda *_a, **_k: None
        self.runtime.fusion.estimate = lambda now: FusedPoseEstimate(
            Pose2D(*self.pose, now), PoseFusionState.OK, 0., ("test",), 0., 0.,
            None, None, True, anchor_initialized=True)
        self.runtime.record_event = self.record
        if started:
            self.runtime.start()
            self.runtime.motion.stop()

    def record(self, event, **values):
        self.events.append((event, values))
        if event == "payload_release_start":
            self.assert_stopped()
            self.releasing = True
            self.releases.append((values["channel"], tuple(self.pose)))
        if event == "payload_release_done":
            self.releasing = False
        if event == "payload_detour_stage_start":
            self.stage = values["stage"]
        if event == "payload_detour_stage_done":
            self.stage_poses[values["stage"]] = values["pose"]

    def advance(self, dt):
        self.steps += 1
        self.assertLess(self.steps, 15000)
        twist = self.runtime.drive.last_limited_twist
        if self.releasing:
            self.assert_stopped()
            self.assertFalse(self.relay.get_channel_state(self.releases[-1][0]))
        yaw = self.pose[2] + twist.angular_z_rad_s * dt / 2
        self.pose[0] += twist.linear_x_m_s * math.cos(yaw) * dt
        self.pose[1] += twist.linear_x_m_s * math.sin(yaw) * dt
        self.pose[2] += twist.angular_z_rad_s * dt
        self.now += dt

    def assert_stopped(self):
        twist = self.runtime.drive.last_limited_twist
        self.assertEqual((twist.linear_x_m_s, twist.angular_z_rad_s), (0, 0))

    def run_detour(self, settings=None, **kwargs):
        settings = settings or DetourSettings()
        self.assertTrue(prepare_payload(self.relay, settings.payload_slot))
        kwargs.setdefault("sleep", self.advance)
        return run_payload_detour(self.runtime, settings,
                                  clock=lambda: self.now, **kwargs)

    def assert_safe_exit(self):
        self.assert_stopped()
        self.assertIs(self.runtime.motion.navigation, self.original_navigation)
        self.assertFalse(any(self.relay.query_status().values()))

    def test_all_three_slots_geometry_reverse_and_heading_restoration(self):
        for slot in (1, 2, 3):
            with self.subTest(slot=slot):
                self.fixture((5., 7., math.pi / 2))
                returned = self.run_detour(DetourSettings(payload_slot=slot))
                road = self.stage_poses["advance_7cm"]
                side = self.stage_poses["forward_47cm"]
                reverse = self.stage_poses["reverse_47cm"]
                self.assertAlmostEqual(road.x_m, 5., delta=.006)
                self.assertAlmostEqual(road.y_m, 7.07, delta=.006)
                self.assertAlmostEqual(math.dist((road.x_m, road.y_m), (side.x_m, side.y_m)), .47, delta=.006)
                self.assertLess(side.x_m, road.x_m - .45)
                self.assertLess(math.dist((road.x_m, road.y_m), (reverse.x_m, reverse.y_m)), .03)
                self.assertAlmostEqual(returned.yaw_rad, math.pi / 2, delta=.055)
                self.assertEqual([r[0] for r in self.releases], [slot + 1])
                self.assert_safe_exit()

    def test_abort_at_each_moving_stage_stops_and_switches_all_relays_off(self):
        for stage in ("advance_7cm", "left_90deg", "forward_47cm", "reverse_47cm", "right_90deg"):
            with self.subTest(stage=stage):
                self.fixture()

                def guard():
                    if self.stage == stage:
                        raise RuntimeError("operator STOP")

                with self.assertRaisesRegex(RuntimeError, "operator STOP"):
                    self.run_detour(guard=guard)
                self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
                self.assert_safe_exit()

    def test_abort_during_release_keeps_magnet_off_before_exit(self):
        self.fixture()

        def guard():
            if self.releasing:
                raise RuntimeError("operator STOP during release")

        with self.assertRaisesRegex(RuntimeError, "operator STOP during release"):
            self.run_detour(guard=guard)
        self.assertNotIn("reverse_47cm", self.stage_poses)
        self.assert_safe_exit()

    def test_visual_failure_during_approach_prevents_release(self):
        self.fixture()

        def check_vision():
            if self.stage == "forward_47cm":
                raise RuntimeError("YOLO inference stale")

        with self.assertRaisesRegex(RuntimeError, "stale"):
            self.run_detour(check_vision=check_vision)
        self.assertFalse(self.releases)
        self.assert_safe_exit()

    def test_relay_failure_stops_instead_of_reversing_or_resuming(self):
        self.fixture()
        self.relay.turn_off = Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError, "release"):
            self.run_detour()
        self.assertNotIn("reverse_47cm", self.stage_poses)
        self.assert_safe_exit()

    def vision(self, target):
        vision = Mock(ready=True)
        vision.observe.side_effect = lambda _: (math.floor(self.now / .08), target())
        vision.observe_drop.side_effect = lambda _: (
            math.floor(self.now / .08), target(), CENTER_BOX if target() is not None else None)
        return vision

    def test_patrol_resumes_original_endpoints_and_persistent_target_drops_once(self):
        self.fixture(started=False)
        alarm = Mock()
        vision = self.vision(lambda: BOX if self.pose[0] > .5 or self.releases else None)
        count = route.run_route(self.runtime, vision, alarm, detour=DetourSettings(payload_slot=2),
                                clock=lambda: self.now, sleep=self.advance)
        self.assertEqual(count, 1)
        self.assertEqual([r[0] for r in self.releases], [3])
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.FINISHED)
        self.assertAlmostEqual(self.pose[0], .3, delta=.04)
        self.assertAlmostEqual(self.pose[1], 4.2, delta=.04)
        self.assertAlmostEqual(self.pose[2], math.pi, delta=.06)
        alarm.on.assert_not_called()
        self.assert_safe_exit()

    def test_new_target_after_three_clear_camera_frames_can_drop_again(self):
        self.fixture(started=False)
        vision = self.vision(lambda: BOX if (.5 < self.pose[0] < .9 or 1.2 < self.pose[0] < 1.6)
                             and self.pose[1] < .1 else None)
        count = route.run_route(self.runtime, vision, Mock(), detour=DetourSettings(),
                                clock=lambda: self.now, sleep=self.advance)
        self.assertEqual(count, 2)
        self.assertEqual(len(self.releases), 2)

    def test_targets_seen_only_during_patrol_turns_do_not_drop(self):
        self.fixture(started=False)
        vision = self.vision(lambda: BOX if .2 < self.pose[2] < 1.3 else None)
        count = route.run_route(self.runtime, vision, Mock(), detour=DetourSettings(),
                                clock=lambda: self.now, sleep=self.advance)
        self.assertEqual(count, 0)
        self.assertFalse(self.releases)

    def test_missing_relay_refuses_before_any_motion(self):
        self.fixture(started=False)
        self.runtime.relay = None
        with self.assertRaisesRegex(RuntimeError, "configured"):
            route.run_route(self.runtime, Mock(), Mock(), detour=DetourSettings())
        self.assertFalse(self.runtime.drive.is_running)
        self.assert_stopped()

    def test_explicit_relay_configuration_and_cleanup_policy(self):
        config = load_runtime_config()
        with self.assertRaisesRegex(ValueError, "explicit"):
            payload_config(config, action="drop", release_mode="relay")
        configured = payload_config(config, action="drop", release_mode="relay",
                                    relay_port="/dev/confirmed-relay", relay_channels=4)
        self.assertTrue(configured.relay.enabled)
        self.assertEqual(configured.relay.port, "/dev/confirmed-relay")
        self.assertEqual(configured.relay.channel_count, 4)
        configured = replace(configured, relay=replace(configured.relay, disconnect_on_shutdown=False))
        self.assertTrue(payload_config(configured, action="drop", release_mode="relay").relay.disconnect_on_shutdown)
        with self.assertRaisesRegex(ValueError, "disabled"):
            payload_config(configured, action="beep")

        simulated = payload_config(configured, action="drop")
        self.assertFalse(simulated.relay.enabled)
        self.assertEqual(simulated.relay.channel_count, 4)

    def test_patrol_holding_failure_prevents_motion_and_disconnects_all(self):
        self.fixture(started=False)
        self.relay.turn_on = Mock(return_value=False)
        with self.assertRaisesRegex(RuntimeError, "holding"):
            route.run_route(self.runtime, self.vision(lambda: None), None,
                            detour=DetourSettings(), clock=lambda: self.now, sleep=self.advance)
        self.assertFalse(self.stage_poses)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assert_safe_exit()
        self.assertGreater(self.relay.off_requests, 0)

    def test_selected_left_front_channel_missing_refuses_before_patrol(self):
        self.fixture(started=False)
        self.runtime.relay = FakeLCUSRelay(3)
        with self.assertRaisesRegex(RuntimeError, "configured"):
            route.run_route(self.runtime, Mock(), None, detour=DetourSettings(payload_slot=3))
        self.assertFalse(self.runtime.drive.is_running)

    def test_successful_detour_releases_selected_magnet_and_preserves_other_loaded_magnets(self):
        self.fixture()
        self.relay.turn_on(2)
        self.relay.turn_on(3)
        self.relay.turn_on(4)
        self.run_detour(DetourSettings(payload_slot=2))
        self.assertEqual(self.relay.query_status(), {1: False, 2: True, 3: False, 4: True})
        self.assert_stopped()

    def test_lateral_residual_at_drop_does_not_pivot_and_return_reuses_approach_heading(self):
        self.fixture()
        advance = self.advance
        injected = False
        samples = []

        def with_residual(dt):
            nonlocal injected
            advance(dt)
            if self.stage == "forward_47cm" and self.pose[1] > .4 and not injected:
                self.pose[0] += .02
                injected = True
            if self.stage in ("forward_47cm", "reverse_47cm"):
                samples.append((self.stage, self.pose[2], self.runtime.drive.last_limited_twist))

        self.run_detour(sleep=with_residual)
        self.assertTrue(injected)
        side_yaw = self.stage_poses["left_90deg"].yaw_rad
        for stage, yaw, twist in samples:
            self.assertLess(abs(yaw - side_yaw), math.radians(10))
            if stage == "reverse_47cm":
                self.assertLessEqual(twist.linear_x_m_s, 0)
        starts = [data for event, data in self.events if event == "payload_detour_stage_start"
                  and data["stage"] in ("forward_47cm", "reverse_47cm")]
        self.assertEqual([data["kwargs"]["heading_yaw_rad"] for data in starts], [side_yaw, side_yaw])
        self.assert_safe_exit()

    def test_real_payload_profile_rejects_missing_fourth_channel(self):
        config = load_runtime_config()
        with self.assertRaisesRegex(ValueError, "2, 3 and 4"):
            payload_config(config, action="drop", release_mode="relay",
                           relay_port="/dev/confirmed-relay", relay_channels=3)

    def test_visibility_slows_before_horizontal_center_trigger(self):
        self.fixture(started=False)
        vision = Mock(ready=True)

        def observation(_):
            visible = BOX if .3 < self.pose[0] < 1.4 and self.pose[1] < .1 else None
            centered = CENTER_BOX if visible is not None and self.pose[0] > .7 else None
            return math.floor(self.now / .08), visible, centered

        vision.observe_drop.side_effect = observation
        vision.observe.side_effect = lambda _: observation(None)[:2]
        before_center_speeds = []
        advance = self.advance

        def tracked_advance(dt):
            if .4 < self.pose[0] < .65 and self.pose[1] < .1 and not self.releases:
                before_center_speeds.append(self.runtime.drive.last_limited_twist.linear_x_m_s)
                self.assertFalse(self.releases)
                self.assertLessEqual(self.runtime.motion.drive.max_linear_speed_m_s, .08)
            advance(dt)

        original = self.runtime.motion.drive
        count = route.run_route(self.runtime, vision, None, detour=DetourSettings(),
                                clock=lambda: self.now, sleep=tracked_advance)
        self.assertEqual(count, 1)
        self.assertTrue(before_center_speeds)
        self.assertLessEqual(max(before_center_speeds), .081)
        road = self.stage_poses["advance_7cm"]
        self.assertAlmostEqual(road.x_m, .77, delta=.012)
        self.assertIs(self.runtime.motion.drive, original)

    def test_horizontal_trigger_ignores_y_and_considers_all_colours(self):
        for cy in (5, 240, 475):
            for cls in range(4):
                centered = (300, cy - 4, 340, cy + 4, .9, cls)
                self.assertEqual(route.horizontal_target([BOX, centered], 640), centered)
        self.assertIsNone(route.horizontal_target([BOX], 640))
        latch = route.EntryLatch()
        self.assertIsNone(latch.update(BOX, trigger=False))
        self.assertTrue(latch.armed)
        self.assertEqual(latch.update(CENTER_BOX), CENTER_BOX)
        for _ in range(10):
            self.assertIsNone(latch.update(BOX, trigger=False))
        self.assertFalse(latch.armed)  # Visible off-centre target cannot re-arm.

    def test_full_frame_trigger_includes_edges_and_rejects_bad_boxes(self):
        self.assertEqual(route.visible_target([BOX], 640, 480), BOX)
        self.assertIsNone(route.central_target([BOX], 640, 480))
        for box in ((0, 0, 100, 100, .1, 0), (20, 20, 10, 30, .9, 1),
                    (0, 0, float("nan"), 100, .9, 2)):
            self.assertIsNone(route.visible_target([box], 640, 480))


if __name__ == "__main__":
    unittest.main()
