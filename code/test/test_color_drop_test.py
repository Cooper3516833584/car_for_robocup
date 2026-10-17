"""Run the actual test loop against simulated motion; never open hardware."""

import contextlib
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import color_drop_test as program
from core.types import Pose2D


class ColorDropTest(unittest.TestCase):
    def run_program(self, *, failure=None):
        clock = NS(now=100., x=0., y=0., yaw=0., v=0., w=0., reads=0)
        drive = Mock()
        source = Mock()
        servo = Mock()
        releases = []
        cleanup = []

        def advance(seconds):
            clock.x += clock.v * math.cos(clock.yaw) * seconds
            clock.y += clock.v * math.sin(clock.yaw) * seconds
            clock.yaw += clock.w * seconds
            clock.now += seconds

        def command(twist):
            clock.v, clock.w = twist.linear_x_m_s, twist.angular_z_rad_s

        def stop():
            clock.v, clock.w = 0., 0.

        drive.command.side_effect = command
        drive.forward.side_effect = lambda v: command(program.Twist2D(v, 0))
        drive.rotate.side_effect = lambda w: command(program.Twist2D(0, w))
        drive.stop.side_effect = stop
        drive.close.side_effect = stop
        source.stop.side_effect = lambda: cleanup.append(("source", clock.v, clock.w))

        def read():
            advance(.005)
            clock.reads += 1
            if failure == "pose" and clock.reads > 80:
                return None
            return Pose2D(clock.x, clock.y, program.wrap(clock.yaw), clock.now)

        source.read.side_effect = read
        adapter = Mock(max_age_s=.15)
        adapter.adapt.side_effect = lambda raw, **_: NS(pose=raw, reason=None)
        vision = Mock(ready=True, frame_size=(640, 480))

        def observe(_):
            if failure == "vision" and clock.reads > 25:
                raise RuntimeError("camera lost")
            stamp = 100. if failure == "same_frame" else clock.now
            colours = () if failure == "no_target" else tuple(
                (color, (300, 200, 340, 240, .9, 0)) for color in program.SLOTS)
            if failure == "stop" and clock.reads > 80:
                program.signal.getsignal(program.signal.SIGTERM)(None, None)
            return stamp, colours

        vision.observe_colours.side_effect = observe
        vision.close.side_effect = lambda: cleanup.append(("vision", clock.v, clock.w))

        def release(slot=None):
            self.assertEqual((clock.v, clock.w), (0, 0))
            if slot is not None:
                releases.append((slot, clock.x, clock.y, program.wrap(clock.yaw)))
                if failure == "release":
                    raise RuntimeError("PWM failed")
            else:
                cleanup.append(("payload", clock.v, clock.w))
            return True

        config = NS(drive=NS(protocol_mode="differential_vx_vz", allow_in_place_rotation=True),
                    calibration=NS(c10b_diff_firmware_verified=True), t265_mount=None,
                    fusion=NS(t265_min_tracker_confidence=2, t265_max_age_s=.15))
        with contextlib.ExitStack() as stack:
            values = dict(load_v2_config=Mock(return_value=config),
                          build_differential_drive=Mock(return_value=drive),
                          build_t265_source=Mock(return_value=source),
                          build_servo=Mock(return_value=servo),
                          T265PoseAdapter=Mock(return_value=adapter),
                          YoloVision=Mock(return_value=vision), park_servo=Mock(),
                          payload_configured=Mock(return_value=True),
                          close_payload=Mock(return_value=True), open_payload=release)
            for name, value in values.items():
                stack.enter_context(patch.object(program, name, value))
            stack.enter_context(patch.object(program.time, "monotonic", lambda: clock.now))
            stack.enter_context(patch.object(program.time, "sleep", advance))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            status = program.main()
        self.assertEqual((clock.v, clock.w), (0, 0))
        self.assertEqual([row[0] for row in cleanup], ["payload", "vision", "source"])
        self.assertTrue(all(row[1:] == (0, 0) for row in cleanup))
        drive.close.assert_called_once()
        return status, releases, clock

    def test_all_colours_once_correct_release_heading_and_return(self):
        status, releases, clock = self.run_program()
        self.assertEqual(status, 0)
        self.assertEqual([row[0] for row in releases], [1, 2, 0, 4])
        for slot, _, y, yaw in releases:
            self.assertAlmostEqual(y, .32 if slot == 4 else .43, delta=.035)
            wanted = math.pi if slot == 4 else math.pi / 2
            self.assertLess(abs(program.wrap(yaw - wanted)), program.ANGLE_TOLERANCE_RAD)
        self.assertAlmostEqual(clock.y, 0, delta=.04)
        self.assertLess(abs(program.wrap(clock.yaw)), program.ANGLE_TOLERANCE_RAD)

    def test_sensor_and_release_failures_stop_before_cleanup(self):
        for failure in ("pose", "vision", "release", "stop"):
            with self.subTest(failure=failure):
                status, releases, _ = self.run_program(failure=failure)
                self.assertEqual(status, 1)
                self.assertLessEqual(len(releases), 1)

    def test_same_frame_cannot_confirm_and_empty_search_has_distance_limit(self):
        for failure in ("same_frame", "no_target"):
            with self.subTest(failure=failure):
                status, releases, clock = self.run_program(failure=failure)
                self.assertEqual(status, 1)
                self.assertEqual(releases, [])
                self.assertLess(clock.x, program.MAX_SEARCH_M + .01)


if __name__ == "__main__":
    unittest.main()
