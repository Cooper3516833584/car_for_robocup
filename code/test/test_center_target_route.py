"""Hardware-free tests of the actual control loop, including interrupted turns."""

import math
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import center_target_route as task
from components.pose_fusion import FusedPoseEstimate, PoseFusionState
from config.v2_runtime import RuntimeMode
from core.types import Pose2D
from robocup_runtime import RobocupMissionState, build_runtime, load_runtime_config
from hal.pwm import PWMBackendError
from tools.run_center_target_route import park_servo


class CenterTargetTests(unittest.TestCase):
    def test_all_classes_and_actual_frame_dimensions(self):
        for cls in range(4):
            box = (240, 160, 400, 320, 0.9, cls)
            self.assertEqual(task.central_target([box], 640, 480), box)
        self.assertIsNone(task.central_target([(0, 160, 100, 320, .9, 0)], 640, 480))
        self.assertIsNone(task.central_target([(240, 0, 400, 80, .9, 0)], 640, 480))
        self.assertIsNone(task.central_target([(240, 160, 400, 320, .1, 0)], 640, 480))
        self.assertIsNone(task.central_target([(400, 160, 240, 320, .9, 0)], 640, 480))

    def test_vision_cpu_selection_respects_allowed_cores(self):
        policies = [(1800000, {0, 1, 2, 3}), (2352000, {4, 5}), (2304000, {6, 7})]
        self.assertEqual(task.select_fastest_cpus(policies, range(8)), {4, 5})
        self.assertEqual(task.select_fastest_cpus(policies, {0, 6, 7}), {6, 7})
        self.assertEqual(task.select_fastest_cpus([], range(8)), set())

    def test_persistent_target_and_short_dropout_do_not_repeat(self):
        latch = task.EntryLatch()
        box = (1, 1, 2, 2, .9, 2)
        self.assertEqual(latch.update(box), box)
        for _ in range(10):
            self.assertIsNone(latch.update(box))
        latch.update(None)
        latch.update(None)
        self.assertIsNone(latch.update(box))
        for _ in range(3):
            latch.update(None)
        self.assertEqual(latch.update(box), box)

    def test_rotated_start_route_lengths_and_left_turns(self):
        start = Pose2D(5, 7, math.pi / 2, 0)
        route = task.route_from_pose(start)
        self.assertEqual([a.method for a in route],
                         ["follow_segment", "rotate_to", "follow_segment", "rotate_to", "follow_segment"])
        for index, distance in ((0, 2.8), (2, 4.2), (4, 2.5)):
            self.assertAlmostEqual(math.dist(*route[index].args), distance)
        self.assertAlmostEqual(route[1].args[0] - start.yaw_rad, math.pi / 2)
        self.assertAlmostEqual(route[3].args[0] - route[1].args[0], math.pi / 2)

    def test_pwm_export_permission_delay_is_bounded_and_retried(self):
        servo = Mock()
        failure = PWMBackendError("udev permissions pending")
        failure.__cause__ = PermissionError("period")
        servo.start.side_effect = [failure, None]
        now = [0.]
        park_servo(servo, 90, clock=lambda: now[0],
                   sleep=lambda dt: now.__setitem__(0, now[0] + dt))
        self.assertEqual(servo.start.call_count, 2)
        servo.set_angle.assert_called_once_with(90, settle=True)
        servo.start.side_effect = failure
        with self.assertRaises(PWMBackendError):
            park_servo(servo, 90, clock=lambda: now[0],
                       sleep=lambda dt: now.__setitem__(0, now[0] + dt))

    def fixture(self):
        self.now = 10.0
        self.pose = [0., 0., 0.]
        self.steps = 0
        self.runtime = build_runtime(load_runtime_config(), RuntimeMode.DRY_RUN,
                                     clock=lambda: self.now)
        self.addCleanup(self.runtime.close)
        self.runtime.record_event = Mock()
        self.runtime._consume_t265 = lambda *_a, **_k: None
        self.runtime._consume_d500 = lambda *_a, **_k: None
        self.runtime.fusion.estimate = lambda now: FusedPoseEstimate(
            Pose2D(*self.pose, now), PoseFusionState.OK, 0., ("test",), 0., 0.,
            None, None, True, anchor_initialized=True)
        self.beeps = []
        self.alarm = Mock()
        self.alarm.on.side_effect = self.beep_on
        self.alarm.off.side_effect = self.beep_off
        self.beep_start = None

    def beep_on(self):
        twist = self.runtime.drive.last_limited_twist
        self.assertEqual((twist.linear_x_m_s, twist.angular_z_rad_s), (0, 0))
        self.beep_start = self.now

    def beep_off(self):
        if self.beep_start is not None:
            self.beeps.append(self.now - self.beep_start)
            self.beep_start = None

    def advance(self, dt):
        self.steps += 1
        self.assertLess(self.steps, 10000)
        twist = self.runtime.drive.last_limited_twist
        if self.beep_start is not None:
            self.assertEqual((twist.linear_x_m_s, twist.angular_z_rad_s), (0, 0))
        yaw = self.pose[2] + twist.angular_z_rad_s * dt / 2
        self.pose[0] += twist.linear_x_m_s * math.cos(yaw) * dt
        self.pose[1] += twist.linear_x_m_s * math.sin(yaw) * dt
        self.pose[2] += twist.angular_z_rad_s * dt
        self.now += dt

    def test_stop_beep_resume_remaining_distance_and_turn(self):
        self.fixture()
        events = [False, False]

        def poll(now):
            if not events[0] and self.pose[0] > 1:
                events[0] = True
                return (240, 160, 400, 320, .9, 0)
            if not events[1] and self.pose[2] > .5:
                events[1] = True
                return (240, 160, 400, 320, .9, 3)
            return None

        vision = Mock(ready=True)
        vision.poll.side_effect = poll
        count = task.run_route(self.runtime, vision, self.alarm,
                               clock=lambda: self.now, sleep=self.advance)
        self.assertEqual(count, 2)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.FINISHED)
        self.assertAlmostEqual(self.pose[0], .3, delta=.04)
        self.assertAlmostEqual(self.pose[1], 4.2, delta=.04)
        self.assertAlmostEqual(self.pose[2], math.pi, delta=.06)
        for duration in self.beeps:
            self.assertAlmostEqual(duration, 1., delta=.001)

    def test_vision_failure_during_movement_stops_and_does_not_resume(self):
        self.fixture()
        vision = Mock(ready=True)

        def poll(now):
            if self.pose[0] > .2:
                raise RuntimeError("camera disconnected")
            return None

        vision.poll.side_effect = poll
        with self.assertRaisesRegex(RuntimeError, "camera disconnected"):
            task.run_route(self.runtime, vision, self.alarm,
                           clock=lambda: self.now, sleep=self.advance)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist.linear_x_m_s, 0)
        self.alarm.on.assert_not_called()

    def test_worker_stale_and_error_observations_fail_closed(self):
        vision = task.YoloVision()
        vision._frame_at = 5
        with self.assertRaisesRegex(RuntimeError, "stale"):
            vision.poll(8)
        vision._error = "camera failure"
        with self.assertRaisesRegex(RuntimeError, "camera failure"):
            vision.poll(5)

    def test_interrupt_during_beep_silences_and_stops(self):
        self.fixture()
        vision = Mock(ready=True)
        vision.poll.return_value = (240, 160, 400, 320, .9, 1)
        with self.assertRaisesRegex(RuntimeError, "interrupted"):
            task.run_route(self.runtime, vision, self.alarm,
                           abort=lambda: self.now >= 10.25,
                           clock=lambda: self.now, sleep=self.advance)
        self.assertLess(self.beeps[0], 1.)
        self.assertEqual(self.runtime.mission.state, RobocupMissionState.SAFE_STOP)
        self.assertEqual(self.runtime.drive.last_limited_twist.linear_x_m_s, 0)
        self.assertIsNone(self.beep_start)


if __name__ == "__main__":
    unittest.main()
