"""Hardware-free checks for the supervised single-action motion tool."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.types import Twist2D  # noqa: E402
from tools.basic_motion import (  # noqa: E402
    BasicMotionRunner, YawAccumulator, build_parser, forward_progress_cm,
    motion_timeout_s, validate_angle_deg, validate_distance_cm,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeDrive:
    command_timeout_s = 0.25

    def __init__(self, fail: bool = False) -> None:
        self.fail = fail
        self.last_limited_twist = Twist2D(0.0, 0.0)
        self.commands = []
        self.stop_count = 0

    def command(self, twist: Twist2D) -> None:
        if self.fail:
            raise RuntimeError("backend failed")
        self.commands.append(twist)
        self.last_limited_twist = twist

    def stop(self) -> None:
        self.stop_count += 1
        self.last_limited_twist = Twist2D(0.0, 0.0)


class ScriptedReader:
    def __init__(self, clock: FakeClock, poses) -> None:
        self.clock = clock
        self.poses = list(poses)
        self.last = None

    def latest_sample(self):
        if self.poses:
            self.last = self.poses.pop(0)
        return None if self.last is None else (self.clock(), self.last)

    def latest(self):
        return self.last


class BasicMotionTests(unittest.TestCase):
    def runner(self, poses=(), *, fail_drive=False):
        clock = FakeClock()
        drive = FakeDrive(fail=fail_drive)
        reader = ScriptedReader(clock, poses)
        return BasicMotionRunner(drive, reader, clock=clock, sleep=clock.sleep), drive, reader

    def test_input_range_and_one_thousand_cm_budget(self) -> None:
        for distance in (10, -10, 50, -50, 1000, -1000):
            self.assertEqual(validate_distance_cm(distance), distance)
        for distance in (0, 9.9, 1000.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                validate_distance_cm(distance)
        self.assertEqual(motion_timeout_s(1000.0, 10.0), 205.0)
        self.assertEqual(motion_timeout_s(10.0, 10.0), 7.0)
        self.assertEqual(build_parser().parse_args([
            "--config", "local.toml", "distance", "--cm", "-1000",
        ]).cm, -1000.0)

    def test_relative_signed_yaw_unwraps_both_full_turns(self) -> None:
        self.assertEqual(validate_angle_deg(360), 360)
        self.assertEqual(validate_angle_deg(-360), -360)
        self.assertEqual(validate_angle_deg(0), 0)
        with self.assertRaises(ValueError):
            validate_angle_deg(360.1)
        with self.assertRaises(ValueError):
            validate_angle_deg(float("nan"))
        for direction in (1, -1):
            accumulator = YawAccumulator(0.0)
            for step in range(1, 37):
                yaw = math.radians((direction * 10 * step + 180) % 360 - 180)
                accumulator.add(yaw)
            self.assertAlmostEqual(math.degrees(accumulator.total_rad), direction * 360)

    def test_forward_projection_uses_start_heading_and_reverse_sign(self) -> None:
        self.assertAlmostEqual(forward_progress_cm(
            (0.0, 0.0, math.pi / 2), (0.0, 0.5, math.pi / 2),
        ), 50.0)
        self.assertAlmostEqual(forward_progress_cm(
            (0.0, 0.0, 0.0), (-0.5, 0.0, 0.0),
        ), -50.0)

    def test_jog_commands_all_wheel_directions_and_always_stops(self) -> None:
        for direction, expected_axis, expected_sign in (
            ("forward", "linear", 1), ("backward", "linear", -1),
            ("left", "angular", 1), ("right", "angular", -1),
        ):
            runner, drive, _ = self.runner()
            runner.jog(direction, 0.1, 0.05, 0.35)
            self.assertTrue(drive.commands)
            if expected_axis == "linear":
                self.assertTrue(all(cmd.linear_x_m_s * expected_sign > 0.0
                                    and cmd.angular_z_rad_s == 0.0 for cmd in drive.commands))
            else:
                self.assertTrue(all(cmd.angular_z_rad_s * expected_sign > 0.0
                                    and cmd.linear_x_m_s == 0.0 for cmd in drive.commands))
            self.assertGreaterEqual(drive.stop_count, 1)
        runner, drive, _ = self.runner(fail_drive=True)
        with self.assertRaisesRegex(RuntimeError, "backend failed"):
            runner.jog("forward", 0.1, 0.05, 0.35)
        self.assertGreaterEqual(drive.stop_count, 1)

    def test_signed_distance_reaches_target_then_settles_stopped(self) -> None:
        for target, sign in ((10.0, 1.0), (-10.0, -1.0)):
            poses = [(sign * step * 0.01, 0.0, 0.0) for step in range(11)]
            runner, drive, _ = self.runner(poses)
            result = runner.distance(target, 0.10)
            self.assertAlmostEqual(result.achieved, target)
            self.assertTrue(drive.commands)
            self.assertTrue(all(cmd.linear_x_m_s * sign > 0.0
                                and cmd.angular_z_rad_s == 0.0 for cmd in drive.commands))
            self.assertGreaterEqual(drive.stop_count, 2)
            self.assertIn("stop", [row[-1] for row in runner.trace])
            self.assertIn("settle", [row[-1] for row in runner.trace])
            self.assertTrue(all(row[4] == 0.0 and row[5] == 0.0
                                for row in runner.trace if row[-1] in {"stop", "settle"}))

    def test_trace_keeps_raw_camera_displacement_separate_from_base_pose(self) -> None:
        runner, _, reader = self.runner()
        raw = [(-0.25, 0.0)]
        reader.latest_raw_xy = lambda: raw[0]
        runner._record((0.0, 0.0, 0.0), "first")
        raw[0] = (0.0, -0.25)
        runner._record((0.0, 0.0, math.pi / 2.0), "turned")
        self.assertEqual(runner.trace[0][-3:-1], (0.0, 0.0))
        self.assertEqual(runner.trace[1][-3:-1], (25.0, -25.0))
        self.assertEqual(runner.trace[1][1:3], (0.0, 0.0))

    def test_one_thousand_cm_completes_in_virtual_time(self) -> None:
        poses = [(step * 0.05, 0.0, 0.0) for step in range(201)]
        runner, drive, _ = self.runner(poses)
        result = runner.distance(1000.0, 0.10)
        self.assertAlmostEqual(result.achieved, 1000.0)
        self.assertTrue(drive.commands)
        self.assertLess(result.elapsed_s, 205.0)

    def test_relative_angle_reaches_both_directions_across_wrap(self) -> None:
        for target, sign in ((360.0, 1.0), (-360.0, -1.0)):
            poses = [
                (0.0, 0.0, math.radians((sign * 10 * step + 180) % 360 - 180))
                for step in range(37)
            ]
            runner, drive, _ = self.runner(poses)
            result = runner.rotate(target, 0.35)
            self.assertAlmostEqual(result.achieved, target)
            self.assertAlmostEqual(result.center_shift_cm, 0.0)
            self.assertTrue(all(cmd.linear_x_m_s == 0.0 and cmd.angular_z_rad_s * sign > 0.0
                                for cmd in drive.commands))
            self.assertGreaterEqual(drive.stop_count, 2)

    def test_arbitrary_relative_angle_and_zero_are_safe(self) -> None:
        for target, sign in ((37.0, 1.0), (-37.0, -1.0)):
            poses = [
                (0.0, 0.0, math.radians(sign * value))
                for value in (0.0, 10.0, 20.0, 30.0, 37.0)
            ]
            runner, drive, _ = self.runner(poses)
            result = runner.rotate(target, 0.35)
            self.assertAlmostEqual(result.achieved, target)
            self.assertTrue(drive.commands)
        runner, drive, _ = self.runner()
        result = runner.rotate(0.0, 0.35)
        self.assertEqual(result.achieved, 0.0)
        self.assertEqual(drive.commands, [])
        self.assertGreaterEqual(drive.stop_count, 1)

    def test_pose_loss_jump_wrong_direction_abort_and_timeout_stop(self) -> None:
        runner, drive, reader = self.runner([(0.0, 0.0, 0.0)])
        original = reader.latest_sample
        calls = 0

        def disappear():
            nonlocal calls
            calls += 1
            return original() if calls == 1 else None

        reader.latest_sample = disappear
        with self.assertRaisesRegex(RuntimeError, "T265 pose stale"):
            runner.distance(10.0, 0.1)
        self.assertEqual(drive.commands, [])
        self.assertGreaterEqual(drive.stop_count, 1)

        for poses, error in (
            ([(0, 0, 0), (0.11, 0, 0)], "T265 pose jumped"),
            ([(0, 0, 0), (-0.06, 0, 0)], "opposes"),
        ):
            runner, drive, _ = self.runner(poses)
            with self.assertRaisesRegex(RuntimeError, error):
                runner.distance(10.0, 0.1)
            self.assertGreaterEqual(drive.stop_count, 1)

        runner, drive, _ = self.runner([(0.0, 0.0, 0.0)])
        with self.assertRaises(TimeoutError):
            runner.distance(10.0, 0.1)
        self.assertGreaterEqual(drive.stop_count, 1)

    def test_operator_abort_and_rotation_drift_stop(self) -> None:
        runner, drive, _ = self.runner([(0.0, 0.0, 0.0)])
        runner.aborted = True
        with self.assertRaisesRegex(RuntimeError, "operator aborted"):
            runner.rotate(90.0, 0.35)
        self.assertGreaterEqual(drive.stop_count, 1)

        runner, drive, _ = self.runner([
            (0.0, 0.0, 0.0), (0.06, 0.0, 0.1), (0.11, 0.0, 0.2),
        ])
        with self.assertRaisesRegex(RuntimeError, "base_link moved"):
            runner.rotate(90.0, 0.35)
        self.assertGreaterEqual(drive.stop_count, 1)

    def test_closed_loop_backend_failure_stops(self) -> None:
        runner, drive, _ = self.runner(
            [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0)], fail_drive=True,
        )
        with self.assertRaisesRegex(RuntimeError, "backend failed"):
            runner.distance(10.0, 0.10)
        self.assertGreaterEqual(drive.stop_count, 1)


if __name__ == "__main__":
    unittest.main()
