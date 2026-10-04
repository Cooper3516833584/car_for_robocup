"""Pure logic checks for the opt-in relative SLAM task path."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.c10b_diff_backend import FakeDriveBackend
from components.slam_scan_source import ScanOnlyD500Source
from components.t265_driver import FakeT265PoseSource, T265RawPose
from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_loader import load_v2_config
from config.v2_models import ConfigV2Error, LocalizationConfig, SlamLocalizationConfig
from config.v2_runtime import RuntimeMode, validate_runtime_readiness
from main_robocup import main
from robocup_runtime import RuntimeReadinessError, build_runtime


class RelativeSlamTaskTests(unittest.TestCase):
    def test_profile_selects_accepted_localization_without_changing_other_gates(self):
        source = load_v2_config()
        config = accepted_relative_slam_profile(source)
        self.assertEqual(source.localization.backend, "legacy")
        self.assertEqual(config.localization.backend, "slam_toolbox")
        self.assertEqual(config.fusion.t265_min_tracker_confidence, 2)
        self.assertTrue(config.localization.slam.relative_goals_only)
        self.assertFalse(config.localization.slam.require_field_anchor)
        self.assertFalse(config.d500_localization.enable_wall_absolute)
        self.assertFalse(config.d500_localization.require_global_for_hardware)
        self.assertEqual(config.calibration, source.calibration)
        self.assertEqual(config.safety, source.safety)
        self.assertEqual(config.drive, source.drive)

    def test_relative_hardware_readiness_keeps_localization_gates(self):
        config = accepted_relative_slam_profile(load_v2_config())
        errors = validate_runtime_readiness(config, RuntimeMode.HARDWARE_MISSION)
        self.assertEqual(errors, [])
        unqualified = replace(config, localization=replace(
            config.localization, slam=replace(config.localization.slam, relative_goals_only=False)))
        self.assertTrue(any("relative_goals_only" in reason for reason in
                            validate_runtime_readiness(unqualified, RuntimeMode.HARDWARE_MISSION)))

    def test_relative_mode_requires_slam_without_field_anchor(self):
        with self.assertRaises(ConfigV2Error):
            LocalizationConfig(backend="legacy", slam=SlamLocalizationConfig(relative_goals_only=True))
        with self.assertRaises(ConfigV2Error):
            LocalizationConfig(backend="slam_toolbox", slam=SlamLocalizationConfig(
                enabled=True, require_field_anchor=True, relative_goals_only=True))

    def test_probe_builds_scan_only_source_with_fake_drive(self):
        config = accepted_relative_slam_profile(load_v2_config())
        runtime = build_runtime(config, RuntimeMode.HARDWARE_PROBE, sensor_only=True)
        try:
            self.assertIsInstance(runtime.d500_source, ScanOnlyD500Source)
            self.assertIsInstance(runtime.drive.backend, FakeDriveBackend)
            self.assertIsNone(runtime.relay)
            self.assertFalse(runtime.drive.is_running)
        finally:
            runtime.close()

    def test_future_t265_stamp_is_clamped_in_runtime(self):
        config = accepted_relative_slam_profile(load_v2_config())
        runtime = build_runtime(config, RuntimeMode.DRY_RUN, clock=lambda: 10.0)
        raw = T265RawPose(
            translation_xyz=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            velocity_xyz=None, angular_velocity_xyz=None,
            tracker_confidence=2, mapper_confidence=0,
            device_timestamp_ms=10003.0,
            received_monotonic_s=10.0,
            measurement_monotonic_s=10.003,
        )
        runtime.t265_source = FakeT265PoseSource([raw])
        runtime.t265_source.start()
        try:
            runtime._consume_t265(10.0, live_time=True)
            self.assertEqual(runtime.fusion.continuous_t265_pose.timestamp_s, 10.0)
        finally:
            runtime.close()

    def test_cli_accepts_current_fused_frame_goal(self):
        self.assertEqual(main(["--relative-slam", "--mode", "dry-run",
                               "--goal-x", "-2", "--goal-y", "3",
                               "--goal-yaw", "1", "--steps", "2"]), 0)

    def test_cli_yaw_requires_position_goal(self):
        self.assertEqual(main(["--mode", "dry-run", "--goal-yaw", "1"]), 2)

    def test_hardware_mission_defaults_to_relative_fusion(self):
        with patch("main_robocup.build_runtime", side_effect=RuntimeReadinessError("test gate")) as builder:
            self.assertEqual(main(["--mode", "hardware-mission"]), 2)
        config = builder.call_args.args[0]
        self.assertEqual(config.localization.backend, "slam_toolbox")
        self.assertTrue(config.localization.slam.relative_goals_only)
        self.assertEqual(config.fusion.t265_min_tracker_confidence, 2)

    def test_config_localization_remains_explicit_option(self):
        with patch("main_robocup.build_runtime", side_effect=RuntimeReadinessError("test gate")) as builder:
            self.assertEqual(main(["--mode", "hardware-mission", "--localization-from-config"]), 2)
        self.assertEqual(builder.call_args.args[0].localization.backend, "legacy")

    def test_dry_run_keeps_config_localization_by_default(self):
        with patch("main_robocup.build_runtime", side_effect=RuntimeReadinessError("test gate")) as builder:
            self.assertEqual(main(["--mode", "dry-run"]), 2)
        self.assertEqual(builder.call_args.args[0].localization.backend, "legacy")


if __name__ == "__main__":
    unittest.main()
