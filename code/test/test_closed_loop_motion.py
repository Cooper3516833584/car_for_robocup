"""Hardware-free checks for the supervised closed-loop action tool."""

from __future__ import annotations

import contextlib
from dataclasses import dataclass
import io
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from components.basic_motion_controller import MotionActionState  # noqa: E402
from core.types import Pose2D  # noqa: E402
from robocup_runtime import RobocupMissionState  # noqa: E402
from tools import closed_loop_motion as tool  # noqa: E402


@dataclass
class Sample:
    x_m: float
    y_m: float = 0.0
    yaw_rad: float = 0.0
    t265_age_s: float = 0.01
    slam_age_s: float = 0.01
    t265_confidence: float = 1.0
    source_flags: tuple[str, ...] = ("t265", "slam", "fused")


class FakeMotion:
    def __init__(self):
        self.calls = []

    def drive_distance(self, distance):
        self.calls.append(("drive_distance", distance))


class FakeDrive:
    def __init__(self):
        self.is_running = True
        self.stop_count = 0

    def stop(self):
        self.is_running = False
        self.stop_count += 1


class FakeRuntime:
    def __init__(self, steps):
        self.steps = iter(steps)
        self.motion = FakeMotion()
        self.drive = FakeDrive()
        self.started = False

    def start(self):
        self.started = True

    def step(self):
        return next(self.steps)


def fake_step(sample, mission, motion=None):
    action = None if motion is None else type("Motion", (), {"state": motion})()
    return type("Step", (), {"now_s": 1.0, "estimate": sample,
                             "mission_state": mission, "motion": action, "error": None})()


class ClosedLoopMotionTests(unittest.TestCase):
    def test_all_eight_actions_use_start_pose_and_correct_target(self):
        start = Pose2D(1.0, 2.0, math.pi / 2, 1.0)
        cases = [
            (tool.ActionRequest("drive-distance", distance_m=0.5), "drive_distance", (0.5,), 1.0, 2.5),
            (tool.ActionRequest("rotate", angle_rad=-math.pi / 2), "rotate", (-math.pi / 2,), 1.0, 2.0),
            (tool.ActionRequest("rotate-to", angle_rad=-math.pi / 2), "rotate_to", (0.0,), 1.0, 2.0),
            (tool.ActionRequest("face-point", forward_m=0.3, left_m=0.1), "face_point", (0.9, 2.3), 1.0, 2.0),
            (tool.ActionRequest("drive-to", forward_m=0.3, left_m=0.1), "drive_to", (0.9, 2.3), 0.9, 2.3),
            (tool.ActionRequest("follow-segment", forward_m=0.3, left_m=0.1), "follow_segment", ((1.0, 2.0), (0.9, 2.3)), 0.9, 2.3),
            (tool.ActionRequest("navigate-to", forward_m=0.3, left_m=0.1), "navigate_to", (0.9, 2.3), 0.9, 2.3),
            (tool.ActionRequest("navigate-to-pose", forward_m=0.3, left_m=0.1, angle_rad=-math.pi / 2),
             "navigate_to_pose", (0.9, 2.3, 0.0), 0.9, 2.3),
        ]
        for request, method, arguments, goal_x, goal_y in cases:
            with self.subTest(request=request.name):
                target = tool.target_from_start(start, request)
                self.assertEqual(target.method, method)
                self.assertEqual(len(target.args), len(arguments))
                for actual, expected in zip(target.args, arguments):
                    if isinstance(expected, tuple):
                        self.assertEqual(len(actual), len(expected))
                        for a, e in zip(actual, expected):
                            self.assertAlmostEqual(a, e)
                    else:
                        self.assertAlmostEqual(actual, expected)
                self.assertAlmostEqual(target.goal_x_m, goal_x)
                self.assertAlmostEqual(target.goal_y_m, goal_y)

    def test_parser_rejects_nonfinite_and_out_of_bounds_requests(self):
        parser = tool.build_parser()
        base = ["--config", "unused.toml", "--output", "unused-output"]
        cases = [
            ["drive-distance", "--cm", "0"],
            ["drive-distance", "--cm", "51"],
            ["rotate", "--deg", "nan"],
            ["rotate-to", "--deg", "91"],
            ["navigate-to", "--forward-cm", "40", "--left-cm", "40"],
            ["face-point", "--forward-cm", "0", "--left-cm", "0"],
        ]
        for tail in cases:
            with self.subTest(tail=tail), self.assertRaises(ValueError):
                tool.request_from_args(parser.parse_args(base + tail))

    def test_settled_metrics_include_reverse_distance_and_wrapped_yaw(self):
        start = Sample(0.0, yaw_rad=math.radians(170))
        end = Sample(-0.49, yaw_rad=math.radians(-170))
        target = tool.target_from_start(Pose2D(0.0, 0.0, 0.0, 0.0),
                                        tool.ActionRequest("drive-distance", distance_m=-0.5))
        metrics = tool.measurements(Sample(0.0), Sample(-0.48), Sample(-0.49),
                                    [Sample(0.0), Sample(-0.48), Sample(-0.49)],
                                    tool.ActionRequest("drive-distance", distance_m=-0.5), target)
        self.assertAlmostEqual(metrics["distance_error_m"], 0.01)
        self.assertTrue(metrics["fused_precision_met"])
        self.assertEqual(metrics["precision_source"], "fused_pose")
        self.assertTrue(metrics["precision_met"])
        turn_target = tool.target_from_start(Pose2D(0.0, 0.0, start.yaw_rad, 0.0),
                                             tool.ActionRequest("rotate", angle_rad=math.radians(20)))
        turn = tool.measurements(start, end, end, [start, end],
                                 tool.ActionRequest("rotate", angle_rad=math.radians(20)), turn_target)
        self.assertAlmostEqual(turn["relative_yaw_rad"], math.radians(20))

    def test_one_action_stops_and_waits_for_settled_pose(self):
        sample0 = Sample(0.0)
        sample_end = Sample(0.05)
        steps = [fake_step(sample0, RobocupMissionState.READY) for _ in range(4)]
        steps += [fake_step(sample0, RobocupMissionState.NAVIGATING, MotionActionState.RUNNING)]
        steps += [fake_step(sample_end, RobocupMissionState.TARGET_OPERATION, MotionActionState.SUCCEEDED)]
        steps += [fake_step(sample_end, RobocupMissionState.TARGET_OPERATION) for _ in range(8)]
        runtime = FakeRuntime(steps)
        runtime.t265_source = type("Source", (), {"serial": "t265-test"})()
        started_serials = []
        now = [0.0]

        def clock():
            return now[0]

        def sleep(seconds):
            now[0] += seconds

        with patch.object(tool, "accepted_fused_sample", side_effect=lambda estimate, *_args, **_kwargs: estimate):
            result = tool.run_one(runtime, object(), tool.ActionRequest("drive-distance", distance_m=0.05),
                                  abort=lambda: False, max_s=2.0, preflight_s=1.0, settle_s=1.0,
                                  preflight_stable_s=0.1,
                                  on_started=lambda started_runtime: started_serials.append(
                                      tool._runtime_serial(started_runtime)),
                                  sleep=sleep, clock=clock)
        self.assertTrue(runtime.started)
        self.assertEqual(started_serials, ["t265-test"])
        self.assertEqual(runtime.motion.calls, [("drive_distance", 0.05)])
        self.assertEqual(runtime.drive.stop_count, 1)
        self.assertAlmostEqual(result["metrics"]["distance_error_m"], 0.0)
        self.assertGreaterEqual(result["sample_count"], 3)

    def test_action_waits_for_stable_fused_pose_before_submission(self):
        steps = [fake_step(Sample(0.0), RobocupMissionState.READY) for _ in range(4)]
        runtime = FakeRuntime(steps)
        now = [0.0]

        def clock():
            return now[0]

        def sleep(seconds):
            now[0] += seconds

        with patch.object(tool, "accepted_fused_sample", side_effect=lambda estimate, *_args, **_kwargs: estimate):
            with self.assertRaisesRegex(TimeoutError, "did not stay healthy"):
                tool.run_one(runtime, object(), tool.ActionRequest("drive-distance", distance_m=0.05),
                             abort=lambda: False, max_s=1.0, preflight_s=0.2, settle_s=1.0,
                             preflight_stable_s=0.5, sleep=sleep, clock=clock)
        self.assertEqual(runtime.motion.calls, [])


def _clock_and_sleep():
    now = [0.0]

    def clock():
        return now[0]

    def sleep(seconds):
        now[0] += seconds

    return clock, sleep


def _run_one(runtime, request, *, max_s=2.0, preflight_s=1.0, settle_s=1.0,
             preflight_stable_s=0.1, abort=lambda: False):
    clock, sleep = _clock_and_sleep()
    with patch.object(tool, "accepted_fused_sample", side_effect=lambda estimate, *_args, **_kwargs: estimate):
        return tool.run_one(runtime, object(), request, abort=abort, max_s=max_s,
                            preflight_s=preflight_s, settle_s=settle_s,
                            preflight_stable_s=preflight_stable_s, sleep=sleep, clock=clock)


class ClosedLoopSafetyTests(unittest.TestCase):
    """Stage-0 step 4: state termination, abnormal parking and bounded limits."""

    def test_motion_terminal_states_stop_the_action(self) -> None:
        terminal = {
            MotionActionState.BLOCKED: "blocked",
            MotionActionState.POSE_LOST: "pose_lost",
            MotionActionState.SAFE_STOPPED: "safe_stopped",
            MotionActionState.ERROR: "error",
        }
        for state, name in terminal.items():
            with self.subTest(state=name):
                steps = [fake_step(Sample(0.0), RobocupMissionState.READY) for _ in range(4)]
                steps.append(fake_step(Sample(0.0), RobocupMissionState.NAVIGATING, state))
                runtime = FakeRuntime(steps)
                with self.assertRaisesRegex(RuntimeError, "motion ended in %s" % name):
                    _run_one(runtime, tool.ActionRequest("drive-distance", distance_m=0.05))

    def test_lost_fused_pose_during_the_action_raises(self) -> None:
        steps = [fake_step(Sample(0.0), RobocupMissionState.READY) for _ in range(4)]
        steps.append(fake_step(None, RobocupMissionState.NAVIGATING, MotionActionState.RUNNING))
        runtime = FakeRuntime(steps)
        with self.assertRaisesRegex(RuntimeError, "healthy fused pose lost during action"):
            _run_one(runtime, tool.ActionRequest("drive-distance", distance_m=0.05))

    def test_mission_error_state_raises_before_any_action(self) -> None:
        steps = [fake_step(Sample(0.0), RobocupMissionState.ERROR)]
        runtime = FakeRuntime(steps)
        with self.assertRaisesRegex(RuntimeError, "mission entered"):
            _run_one(runtime, tool.ActionRequest("drive-distance", distance_m=0.05))
        self.assertEqual(runtime.motion.calls, [])

    def test_action_without_success_hits_its_deadline(self) -> None:
        steps = [fake_step(Sample(0.0), RobocupMissionState.READY) for _ in range(4)]
        steps += [fake_step(Sample(0.0), RobocupMissionState.NAVIGATING, MotionActionState.RUNNING)
                  for _ in range(200)]
        runtime = FakeRuntime(steps)
        with self.assertRaisesRegex(TimeoutError, "exceeded its deadline"):
            _run_one(runtime, tool.ActionRequest("drive-distance", distance_m=0.05), max_s=0.2)

    def test_operator_abort_stops_before_submission(self) -> None:
        steps = [fake_step(Sample(0.0), RobocupMissionState.READY) for _ in range(4)]
        runtime = FakeRuntime(steps)
        with self.assertRaisesRegex(RuntimeError, "operator aborted"):
            _run_one(runtime, tool.ActionRequest("drive-distance", distance_m=0.05),
                     abort=lambda: True)
        self.assertEqual(runtime.motion.calls, [])

    def test_vehicle_never_settling_hits_the_bounded_window(self) -> None:
        steps = [fake_step(Sample(0.0), RobocupMissionState.READY) for _ in range(4)]
        steps.append(fake_step(Sample(0.0), RobocupMissionState.NAVIGATING, MotionActionState.RUNNING))
        steps.append(fake_step(Sample(0.05), RobocupMissionState.TARGET_OPERATION,
                               MotionActionState.SUCCEEDED))
        drifting = [fake_step(Sample(0.05 + 0.02 * index), RobocupMissionState.TARGET_OPERATION)
                    for index in range(1, 60)]
        runtime = FakeRuntime(steps + drifting)
        with self.assertRaisesRegex(TimeoutError, "did not settle"):
            _run_one(runtime, tool.ActionRequest("drive-distance", distance_m=0.05), settle_s=0.5)

    def test_leaving_the_bounded_test_area_raises(self) -> None:
        steps = [fake_step(Sample(0.0), RobocupMissionState.READY) for _ in range(4)]
        steps += [fake_step(Sample(0.05 * index), RobocupMissionState.NAVIGATING,
                            MotionActionState.RUNNING) for index in range(1, 22)]
        runtime = FakeRuntime(steps)
        with self.assertRaisesRegex(RuntimeError, "left the bounded test area"):
            _run_one(runtime, tool.ActionRequest("drive-distance", distance_m=0.05))

    def test_fused_pose_jump_during_the_action_raises(self) -> None:
        steps = [fake_step(Sample(0.0), RobocupMissionState.READY) for _ in range(4)]
        steps.append(fake_step(Sample(0.0), RobocupMissionState.NAVIGATING, MotionActionState.RUNNING))
        steps.append(fake_step(Sample(0.20), RobocupMissionState.NAVIGATING, MotionActionState.RUNNING))
        runtime = FakeRuntime(steps)
        with self.assertRaisesRegex(RuntimeError, "fused pose jumped"):
            _run_one(runtime, tool.ActionRequest("drive-distance", distance_m=0.05))

    def test_single_action_is_submitted_once(self) -> None:
        steps = [fake_step(Sample(0.0), RobocupMissionState.READY) for _ in range(4)]
        steps.append(fake_step(Sample(0.05), RobocupMissionState.NAVIGATING, MotionActionState.RUNNING))
        steps.append(fake_step(Sample(0.05), RobocupMissionState.TARGET_OPERATION,
                               MotionActionState.SUCCEEDED))
        steps += [fake_step(Sample(0.05), RobocupMissionState.TARGET_OPERATION) for _ in range(8)]
        runtime = FakeRuntime(steps)
        _run_one(runtime, tool.ActionRequest("drive-distance", distance_m=0.05))
        self.assertEqual(len(runtime.motion.calls), 1)


class ClosedLoopMainSafetyTests(unittest.TestCase):
    """The tool must request a safe stop and close the runtime on every failure."""

    class _Mission:
        def __init__(self):
            self.safe_stop_reasons = []

        def request_safe_stop(self, reason):
            self.safe_stop_reasons.append(reason)

    class _Runtime:
        def __init__(self):
            self.mission = ClosedLoopMainSafetyTests._Mission()
            self.closed = False
            self.stop_count = 0
            self.is_running = True
            self.motion = FakeMotion()
            self.drive = FakeDrive()

        def start(self):
            pass

        def close(self):
            self.closed = True
            self.stop_count += 1

    class _Logger:
        instances = []

        def __init__(self, path):
            self.dropped_events = 0
            self.write_error = None
            self.closed = False
            ClosedLoopMainSafetyTests._Logger.instances.append(self)

        def close(self):
            self.closed = True

    def _run_main(self, run_one, build_error=None):
        ClosedLoopMainSafetyTests._Logger.instances = []
        runtime = self._Runtime()
        builder = (MagicMock(side_effect=build_error) if build_error is not None
                   else MagicMock(return_value=runtime))
        with tempfile.TemporaryDirectory() as tmp:
            config_path = Path(tmp) / "car.toml"
            shutil.copyfile(ROOT / "configs" / "robocup_diffdrive.example.toml", config_path)
            output = Path(tmp) / "run"
            argv = ["--config", str(config_path), "--output", str(output),
                    "--confirm-motor-test", "--confirm-area-clear", "--confirm-estop-ready",
                    "drive-distance", "--cm", "20"]
            with patch("sys.platform", "linux"), \
                    patch.object(tool, "build_runtime", builder), \
                    patch.object(tool, "run_one", run_one), \
                    patch.object(tool, "JsonlEventLogger", ClosedLoopMainSafetyTests._Logger), \
                    contextlib.redirect_stdout(io.StringIO()):
                code = tool.main(argv)
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
        return code, summary, runtime

    def test_failed_action_requests_safe_stop_and_closes_the_runtime(self) -> None:
        def boom(*_args, **_kwargs):
            raise RuntimeError("motion ended in blocked")

        code, summary, runtime = self._run_main(boom)
        self.assertEqual(code, 1)
        self.assertFalse(summary["valid"])
        self.assertIn("motion ended in blocked", summary["error"])
        self.assertTrue(runtime.mission.safe_stop_reasons)
        self.assertTrue(runtime.closed)

    def test_startup_failure_closes_the_logger_without_a_runtime(self) -> None:
        code, summary, _runtime = self._run_main(
            lambda *_a, **_k: {}, build_error=RuntimeError("cannot open /dev/ttyACM0"))
        self.assertEqual(code, 1)
        self.assertFalse(summary["valid"])
        self.assertIn("cannot open /dev/ttyACM0", summary["error"])
        logger = ClosedLoopMainSafetyTests._Logger.instances[0]
        self.assertTrue(logger.closed)

    def test_successful_action_writes_a_valid_summary_and_closes(self) -> None:
        def ok(*_args, **_kwargs):
            return {"target": {}, "metrics": {"precision_met": True}, "sample_count": 3}

        code, summary, runtime = self._run_main(ok)
        self.assertEqual(code, 0)
        self.assertTrue(summary["valid"])
        self.assertIsNone(summary["error"])
        self.assertTrue(summary["precision_met"])
        self.assertTrue(runtime.closed)
        self.assertEqual(runtime.mission.safe_stop_reasons, [])

    def test_dropped_events_invalidate_an_otherwise_good_run(self) -> None:
        def ok(*_args, **_kwargs):
            return {"target": {}, "metrics": {"precision_met": True}, "sample_count": 3}

        original_init = ClosedLoopMainSafetyTests._Logger.__init__

        def init_with_drops(self, path):
            original_init(self, path)
            self.dropped_events = 7

        with patch.object(ClosedLoopMainSafetyTests._Logger, "__init__", init_with_drops):
            code, summary, _runtime = self._run_main(ok)
        self.assertEqual(code, 1)
        self.assertFalse(summary["valid"])
        self.assertEqual(summary["error"], "event log incomplete")


if __name__ == "__main__":
    unittest.main()
