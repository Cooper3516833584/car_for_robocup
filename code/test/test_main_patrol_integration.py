"""Main-entry guard behaviour: STOP forms, deadline, SIGTERM and CLI validation.

The competition mission itself is covered by ``test_wall_competition.py`` and
``test_wall_runtime.py``. What lives here is the supervision around it: the
STOP-marker signature, the shared task deadline, SIGTERM cleanup and the CLI
validation that must reject a bad invocation before any hardware is built.
"""

from pathlib import Path
import signal
import sys
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workspace_temp import WorkspaceTempDir, cleanup_temp_dir, temp_dir

import main_robocup
from main_robocup import build_parser, main, stop_marker_signature


class FakeDrive:
    def __init__(self):
        self.stops = 0

    def stop(self):
        self.stops += 1

    def start(self):
        pass

    def close(self):
        pass

    def command(self, twist, now_s=None):
        pass


class FakeMission:
    def __init__(self):
        self.state = Mock(value="init")
        self.last_error = None
        self.task_counts = None
        self.phase = "init"
        self.safe_stops = []

    def set_task_counts(self, counts):
        pass

    def finish(self):
        self.state = Mock(value="finished")

    def request_safe_stop(self, reason=""):
        self.safe_stops.append(reason)
        self.state = Mock(value="safe_stop")

    def on_payload_action_done(self):
        pass


class FakeWallRuntime:
    """Just enough wall runtime for the supervision code under test."""

    def __init__(self, clock):
        self.observer = object()
        self.limits = None
        self.drive = FakeDrive()
        self.mission = FakeMission()
        self.stop_latched = False
        self.is_running = True
        self.mode = Mock(value="hardware-mission")
        self.config = None
        self.event_logger = None
        self.events = []
        self.clock = clock
        self.closed = False

    def record_event(self, name, **fields):
        self.events.append((name, fields))

    def start(self):
        pass

    def close(self):
        self.closed = True

    def step(self):
        return Mock(command=Mock(linear_x_m_s=0.0, angular_z_rad_s=0.0), error=None)


class StopMarkerSignatureTests(unittest.TestCase):
    def setUp(self):
        self.directory = temp_dir("stop")
        self.addCleanup(cleanup_temp_dir, self.directory)

    def test_signature_covers_file_identity_size_and_nanosecond_timestamps(self):
        marker = Path(self.directory) / "STOP"
        self.assertIsNone(stop_marker_signature(marker))
        marker.write_text("first")
        first = stop_marker_signature(marker)
        self.assertIsNotNone(first)
        marker.write_text("second-round")
        self.assertNotEqual(stop_marker_signature(marker), first)

    def test_an_absent_marker_is_not_a_stop(self):
        self.assertIsNone(stop_marker_signature(Path(self.directory) / "missing"))

    def test_stat_errors_are_not_treated_as_absence(self):
        marker = Path(self.directory) / "STOP"
        marker.write_text("present")
        with patch.object(Path, "stat", side_effect=OSError("denied")):
            with self.assertRaises(OSError):
                stop_marker_signature(marker)


class CliValidationTests(unittest.TestCase):
    def run_main(self, argv):
        return main(argv)

    def test_competition_requires_hardware_mission(self):
        code = self.run_main(["--competition", "--mode", "dry-run"])
        self.assertEqual(code, 2)

    def test_competition_rejects_goal_arguments(self):
        code = self.run_main(["--competition", "--mode", "hardware-mission",
                              "--goal-x", "1.0", "--goal-y", "2.0"])
        self.assertEqual(code, 2)

    def test_goal_x_and_goal_y_must_be_paired(self):
        self.assertEqual(self.run_main(["--goal-x", "1.0"]), 2)

    def test_goal_yaw_requires_a_goal(self):
        self.assertEqual(self.run_main(["--goal-yaw", "1.0"]), 2)

    def test_max_seconds_accepts_the_competition_profile(self):
        # The wall chain budget runs to 1200 s; the old <=600 cap must be gone.
        self.assertEqual(build_parser().parse_args(["--max-seconds", "1200"]).max_seconds, 1200)
        self.assertEqual(self.run_main(["--max-seconds", "1201"]), 2)
        self.assertEqual(self.run_main(["--max-seconds", "0"]), 2)
        self.assertEqual(self.run_main(["--max-seconds", "nan"]), 2)

    def test_speed_scale_must_be_finite_and_positive(self):
        for value in ("0", "-1", "nan"):
            with self.subTest(value=value):
                self.assertEqual(self.run_main(["--speed-scale", value]), 2)

    def test_leg_applies_only_to_the_wall_leg_stages(self):
        self.assertEqual(self.run_main(["--competition", "--mode", "hardware-mission",
                                        "--competition-stage", "full", "--leg", "1"]), 2)

    def test_wall_leg_requires_a_leg_number(self):
        self.assertEqual(self.run_main(["--competition", "--mode", "hardware-mission",
                                        "--competition-stage", "wall-leg"]), 2)

    def test_task_board_startup_requires_hardware_mission_and_an_angle(self):
        self.assertEqual(self.run_main(["--task-board-camera", "0"]), 2)
        self.assertEqual(self.run_main(["--task-board-camera", "0",
                                        "--task-board-turn-deg", "90"]), 2)


class GuardAndShutdownTests(unittest.TestCase):
    """The guards must fire without depending on the retired fused route."""

    def make_runtime(self, clock):
        return FakeWallRuntime(clock)

    def test_a_stop_marker_written_before_start_is_rejected(self):
        directory = temp_dir("guard")
        self.addCleanup(cleanup_temp_dir, directory)
        marker = Path(directory) / "STOP"
        marker.write_text("operator stop")
        now = [0.0]
        runtime = self.make_runtime(lambda: now[0])
        with patch.object(main_robocup, "build_wall_runtime", return_value=runtime), \
             patch("competition_task.run_full_mission", return_value=True):
            code = main(["--competition", "--mode", "hardware-mission",
                         "--config", str(main_robocup.Path("configs/robocup_diffdrive.toml")),
                         "--log-dir", directory, "--max-seconds", "30"])
        # A pre-existing STOP marker must stop the run without claiming success.
        self.assertNotEqual(code, 0)
        self.assertTrue(runtime.closed)

    def test_a_zero_deadline_aborts_before_any_motion(self):
        directory = temp_dir("deadline")
        self.addCleanup(cleanup_temp_dir, directory)
        now = [0.0]
        runtime = self.make_runtime(lambda: now[0])

        def mission(*_args, **_kwargs):
            raise RuntimeError("competition task time limit expired")

        with patch.object(main_robocup, "build_wall_runtime", return_value=runtime), \
             patch("competition_task.run_full_mission", side_effect=mission):
            code = main(["--competition", "--mode", "hardware-mission",
                         "--log-dir", directory, "--max-seconds", "1"])
        self.assertEqual(code, 1)
        self.assertTrue(runtime.closed)

    def test_sigterm_is_restored_after_the_run(self):
        if threading.current_thread() is not threading.main_thread():
            self.skipTest("signal handlers are main-thread only")
        directory = temp_dir("sigterm")
        self.addCleanup(cleanup_temp_dir, directory)
        before = signal.getsignal(signal.SIGTERM)
        now = [0.0]
        runtime = self.make_runtime(lambda: now[0])
        with patch.object(main_robocup, "build_wall_runtime", return_value=runtime), \
             patch("competition_task.run_full_mission", side_effect=KeyboardInterrupt):
            code = main(["--competition", "--mode", "hardware-mission",
                         "--log-dir", directory, "--max-seconds", "30"])
        self.assertEqual(code, 130)
        self.assertEqual(signal.getsignal(signal.SIGTERM), before)

    def test_a_runtime_build_failure_is_reported_not_raised(self):
        with patch.object(main_robocup, "build_wall_runtime",
                          side_effect=main_robocup.WallRuntimeReadinessError("no lidar")):
            code = main(["--competition", "--mode", "hardware-mission"])
        self.assertEqual(code, 2)


if __name__ == "__main__":
    unittest.main()
