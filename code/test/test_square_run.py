"""Hardware-free checks for the supervised square calibration tool."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from core.types import Twist2D  # noqa: E402
from tools.square_run import (  # noqa: E402
    Runner,
    T265PoseReader,
    segment_progress,
    segment_timeout_s,
)


class FakeDrive:
    def __init__(self) -> None:
        self.last_limited_twist = Twist2D(0.0, 0.0)
        self.commands = []
        self.stop_count = 0

    def command(self, twist: Twist2D) -> None:
        self.last_limited_twist = twist
        self.commands.append(twist)

    def stop(self) -> None:
        self.stop_count += 1
        self.last_limited_twist = Twist2D(0.0, 0.0)


class ScriptedReader:
    def __init__(self, poses) -> None:
        self.poses = list(poses)

    def latest(self):
        if len(self.poses) > 1:
            return self.poses.pop(0)
        return self.poses[0] if self.poses else None


class SquareRunSafetyTests(unittest.TestCase):
    def test_segment_budget_uses_matching_units_and_is_bounded(self) -> None:
        self.assertAlmostEqual(segment_timeout_s(100.0, 22.0), 11.0909, places=3)
        self.assertAlmostEqual(
            segment_timeout_s(90.0, math.degrees(0.4)), 9.854, places=2
        )
        self.assertEqual(segment_timeout_s(100.0, 1.0), 25.0)
        with self.assertRaises(ValueError):
            segment_timeout_s(100.0, 0.0)

    def test_progress_is_signed_in_the_requested_direction(self) -> None:
        self.assertAlmostEqual(
            segment_progress((0.0, 0.0, 0.0), (-0.10, 0.0, 0.0), "distance"),
            -10.0,
        )
        self.assertAlmostEqual(
            segment_progress((0.0, 0.0, math.pi / 2),
                             (0.0, 1.0, math.pi / 2), "distance"),
            100.0,
        )
        self.assertAlmostEqual(
            segment_progress((0.0, 0.0, 0.0),
                             (0.0, 0.0, math.pi / 2), "angle"),
            90.0,
        )

    def test_stale_t265_pose_is_not_returned(self) -> None:
        reader = T265PoseReader(None, serial="", max_age_s=0.15,
                                min_tracker_confidence=2)
        reader._sample = (time.monotonic() - 1.0, (0.0, 0.0, 0.0))
        self.assertIsNone(reader.latest())

    def test_missing_pose_stops_before_a_motor_command(self) -> None:
        drive = FakeDrive()
        runner = Runner(drive, ScriptedReader([]), 50.0)
        with self.assertRaisesRegex(RuntimeError, "T265 pose stale"):
            runner.hold(100, 0, 1.0, start_pose=(0.0, 0.0, 0.0),
                        mode="distance", target=10.0)
        self.assertEqual(drive.commands, [])
        self.assertGreaterEqual(drive.stop_count, 1)

    def test_trace_includes_zero_command_and_settle_samples(self) -> None:
        drive = FakeDrive()
        reader = ScriptedReader([
            (0.0, 0.0, 0.0), (0.05, 0.0, 0.0), (0.10, 0.0, 0.0),
        ])
        runner = Runner(drive, reader, 50.0)
        runner.hold(100, 0, 1.0, start_pose=(0.0, 0.0, 0.0),
                    mode="distance", target=10.0)
        self.assertTrue(drive.commands)
        self.assertGreaterEqual(drive.stop_count, 2)
        self.assertIn("stop", [row[-1] for row in runner.trace])
        self.assertIn("settle", [row[-1] for row in runner.trace])
        self.assertTrue(all(row[4] == 0.0 and row[5] == 0.0
                            for row in runner.trace if row[-1] in {"stop", "settle"}))


if __name__ == "__main__":
    unittest.main()
