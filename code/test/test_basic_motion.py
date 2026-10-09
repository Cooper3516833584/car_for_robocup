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

    def test_retired_executor_stops_without_commanding_motion(self):
        runner, drive, _ = self.runner()
        for method,args in ((runner.distance,(10,.1)),(runner.rotate,(90,.3)),(runner.jog,('forward',.1,.05,.3))):
            with self.assertRaisesRegex(RuntimeError,'Retired'):
                method(*args)
        self.assertFalse(drive.commands)
        self.assertGreaterEqual(drive.stop_count,3)


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








    def test_retired_cli_entries_fail_before_building_hardware(self):
        from unittest.mock import patch
        from tools import basic_motion, square_run, drive_calibration, drive_actuation_probe, auto_motion_diag, yaw_wall_crosscheck
        from config import v2_factory
        with patch.object(v2_factory,'build_differential_drive',side_effect=AssertionError('hardware opened')):
            for module in (basic_motion,square_run,drive_calibration,drive_actuation_probe,auto_motion_diag):
                with self.subTest(entry=module.__name__):
                    self.assertEqual(module.main([]),1)
            self.assertEqual(yaw_wall_crosscheck.main(),1)

if __name__ == "__main__":
    unittest.main()
