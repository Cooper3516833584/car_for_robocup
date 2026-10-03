from __future__ import annotations

from collections import deque
from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "code"))

from components.c10b_diff_backend import FakeDriveBackend
from components.radar_driver import RadarScan
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import build_runtime
from slam_manual_accuracy import (
    AccuracySession, FloorPose, PoseSample, ScanOnlyD500Source, Snapshot, relative_delta,
    test_config_from_board as make_probe_config, wrapped_delta_rad,
)


class _Logger:
    write_error = None
    dropped_events = 0

    def __init__(self):
        self.events = []

    def emit(self, *_args, **_kwargs):
        self.events.append(_args[0])
        return True


def _sample(t, x=0.0, y=0.0, yaw=0.0):
    return PoseSample(
        t_s=t, x_m=x, y_m=y,
        yaw_rad=(yaw + math.pi) % (2 * math.pi) - math.pi,
        yaw_unwrapped_rad=yaw,
        t265_x_m=x, t265_y_m=y, t265_yaw_rad=yaw,
        t265_confidence=1.0, fusion_state="ok", slam_state="SLAM_OK",
        anchor_age_s=0.1, scan_count=int(t * 5), scan_age_ms=15,
        tf_age_ms=10,
    )


class ManualAccuracyTests(unittest.TestCase):
    def test_signed_local_axes_and_full_turn(self):
        dx, dy, yaw = relative_delta(0, 0, math.pi / 2, -0.5, 0, 5 * math.pi / 2)
        self.assertAlmostEqual(dx, 0.0)
        self.assertAlmostEqual(dy, 0.5)
        self.assertAlmostEqual(math.degrees(yaw), 360.0)
        self.assertAlmostEqual(wrapped_delta_rad(math.radians(179), math.radians(-179)),
                               math.radians(2))

    def test_probe_uses_fake_drive_and_never_enables_hardware_mission(self):
        source = load_v2_config()
        source = replace(source, d500=replace(source.d500, enabled=False))
        config = make_probe_config(source)
        self.assertTrue(config.d500.enabled)
        self.assertFalse(config.d500_localization.enable_wall_absolute)
        self.assertEqual(config.localization.backend, "slam_toolbox")
        self.assertFalse(config.localization.slam.require_field_anchor)
        self.assertFalse(config.localization.slam.hardware_mission_validated)
        runtime = build_runtime(config, RuntimeMode.HARDWARE_PROBE, sensor_only=True)
        try:
            self.assertIsInstance(runtime.drive.backend, FakeDriveBackend)
            self.assertIsNone(runtime.relay)
            self.assertFalse(runtime.drive.is_running)
        finally:
            runtime.close()
        with self.assertRaisesRegex(ValueError, "hardware-probe"):
            build_runtime(config, RuntimeMode.HARDWARE_MISSION, sensor_only=True)

    def test_scan_only_source_queues_scan_without_icp(self):
        config = make_probe_config(load_v2_config())
        runtime = build_runtime(config, RuntimeMode.HARDWARE_PROBE, sensor_only=True)
        logger = _Logger()
        source = ScanOnlyD500Source(runtime, config.d500.port, config.d500.baudrate, logger)
        try:
            scan = RadarScan((), 123, 1800)
            source.assembler.feed = lambda _packet: [scan]
            source._on_packet(object())
            self.assertEqual(runtime.slam_bridge.metrics()["slam.scan_input_count"], 1)
            self.assertEqual(logger.events[-1]["type"], "accuracy_d500_scan")
        finally:
            runtime.close()

    def test_truth_is_independent_of_nominal_distance_and_yaw_unwraps(self):
        with tempfile.TemporaryDirectory() as directory:
            session = AccuracySession(None, _Logger(), Path(directory))
            session.start_time_s = 0.0
            session.preflight_ok = True
            session.samples = deque((_sample(i * 0.05) for i in range(41)), maxlen=12000)
            session.begin("left_turn")
            for i in range(41, 121):
                angle = 2 * math.pi * (i - 40) / 80
                session.samples.append(_sample(i * 0.05, yaw=angle))
            for i in range(121, 162):
                session.samples.append(_sample(i * 0.05, yaw=2 * math.pi))
            session.end()
            row = session.truth(FloorPose(0.0, 0.0, 360.0))
            self.assertEqual(row["status"], "PASS")
            self.assertAlmostEqual(row["estimate_dyaw_deg"], 360.0)
            self.assertAlmostEqual(row["position_error_cm"], 0.0)

    def test_stale_anchor_invalidates_even_when_distance_matches(self):
        with tempfile.TemporaryDirectory() as directory:
            session = AccuracySession(None, _Logger(), Path(directory))
            session.start_time_s = 0.0
            session.preflight_ok = True
            session.samples = deque((_sample(i * 0.05) for i in range(41)), maxlen=12000)
            session.begin("forward")
            for i in range(41, 121):
                good = _sample(i * 0.05, x=0.5 * (i - 40) / 80)
                if i == 80:
                    good = PoseSample(**{**good.__dict__, "slam_state": "SLAM_STALE"})
                session.samples.append(good)
            for i in range(121, 162):
                session.samples.append(_sample(i * 0.05, x=0.5))
            session.end()
            row = session.truth(FloorPose(0.5, 0.0, 0.0))
            self.assertEqual(row["status"], "INVALID")
            self.assertIn("not healthy", row["invalid_reasons"])

    def test_pivot_centre_drift_is_measured_as_position_error(self):
        with tempfile.TemporaryDirectory() as directory:
            session = AccuracySession(None, _Logger(), Path(directory))
            session.start_time_s = 0.0
            session.preflight_ok = True
            session.samples = deque((_sample(i * 0.05) for i in range(41)), maxlen=12000)
            session.begin("turn", "pivot")
            for i in range(41, 162):
                session.samples.append(_sample(i * 0.05))
            session.end()
            row = session.truth(FloorPose(0.04, 0.0, 0.0))
            self.assertEqual(row["kind"], "pivot")
            self.assertEqual(row["status"], "FAIL")
            self.assertAlmostEqual(row["position_error_cm"], 4.0)

    def test_rectangle_closure_uses_its_own_start_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            session = AccuracySession(None, _Logger(), Path(directory))
            session.start_time_s = 0.0
            session.loop_start_snapshot = Snapshot(10.0, 2.0, 3.0, 0.0, 40)
            session.loop_start_truth = FloorPose(0.0, 0.0, 0.0)
            session.loop_start_result_index = 0
            session.last_end_snapshot = Snapshot(20.0, 2.01, 3.0, 0.0, 40)
            session.current_truth = FloorPose(0.0, 0.0, 0.0)
            session.results = [{"status": "PASS"} for _ in range(4)]
            row = session.closure("rectangle")
            self.assertEqual(row["status"], "PASS")
            self.assertAlmostEqual(row["position_error_cm"], 1.0)


if __name__ == "__main__":
    unittest.main()
