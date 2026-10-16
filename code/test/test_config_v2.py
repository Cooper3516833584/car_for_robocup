from __future__ import annotations

from pathlib import Path
from dataclasses import replace
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workspace_temp import WorkspaceTempDir

from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config
from config.v2_models import ConfigV2Error
from config.v2_runtime import RuntimeMode, runtime_constraints, validate_runtime_readiness


class ConfigV2Tests(unittest.TestCase):
    def test_adaptive_lookahead_defaults_for_older_profiles(self):
        config = self._load_modified({
            "lookahead_speed_reference_m_s = 0.15\n": "",
            "lookahead_speed_gain_s = 1.0\n": "",
            "lookahead_max_extension_m = 0.15\n": "",
            "lookahead_short_segment_m = 0.50\n": "",
        })
        self.assertEqual(config.navigation.lookahead_speed_gain_s, 1.)
        self.assertEqual(config.navigation.lookahead_max_extension_m, .15)
        self.assertEqual(config.navigation.lookahead_short_segment_m, .50)

    def test_adaptive_lookahead_rejects_invalid_parameters_and_allows_disabled_gain(self):
        nav = load_v2_config().navigation
        for name in ("lookahead_speed_reference_m_s", "lookahead_short_segment_m",
                     "lookahead_speed_gain_s", "lookahead_max_extension_m"):
            for value in (-1, float("nan"), float("inf")):
                with self.subTest(name=name,value=value), self.assertRaises(ConfigV2Error):
                    replace(nav, **{name:value})
        self.assertEqual(replace(nav, lookahead_speed_gain_s=0).lookahead_speed_gain_s, 0)
        self.assertEqual(replace(nav, lookahead_max_extension_m=0).lookahead_max_extension_m, 0)

    def test_differential_navigator_factory_has_one_definition(self) -> None:
        factory = Path(__file__).resolve().parents[1] / "config" / "v2_factory.py"
        source = factory.read_text(encoding="utf-8")
        self.assertEqual(source.count("def build_differential_navigator("), 1)

    def test_example_loads_as_schema_v2(self) -> None:
        config = load_v2_config(DEFAULT_V2_CONFIG)
        self.assertEqual(config.schema_version, 2)
        self.assertFalse(config.calibration.c10b_diff_firmware_verified)
        self.assertEqual(config.drive.protocol_mode, "ackermann_firmware_compat")
        self.assertEqual(config.d500_mount.z_m, 0.20)
        self.assertEqual(config.t265_mount.z_m, 0.03225)
        self.assertEqual(config.geometry.drive_track_width_m, 0.200)
        self.assertEqual(config.geometry.body_width_m, 0.3596)

    def _load_modified(self, replacements: dict[str, str]):
        source = DEFAULT_V2_CONFIG.read_text(encoding="utf-8")
        for old, new in replacements.items():
            self.assertIn(old, source)
            source = source.replace(old, new, 1)
        with WorkspaceTempDir() as directory:
            path = Path(directory) / "profile.toml"
            path.write_text(source, encoding="utf-8")
            return load_v2_config(path)

    def test_non_positive_track_width_is_rejected(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "drive_track_width_m"):
            self._load_modified({"drive_track_width_m = 0.200": "drive_track_width_m = 0.0"})

    def test_unknown_protocol_is_rejected(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "protocol_mode"):
            self._load_modified({
                'protocol_mode = "ackermann_firmware_compat"': 'protocol_mode = "unknown"'
            })

    def test_dry_run_allows_unmeasured_geometry(self) -> None:
        config = load_v2_config()
        self.assertEqual(validate_runtime_readiness(config, RuntimeMode.DRY_RUN), [])

    def test_relative_hardware_mission_has_no_retired_map_or_footprint_gates(self) -> None:
        config = accepted_relative_slam_profile(load_v2_config())
        self.assertEqual(validate_runtime_readiness(config, RuntimeMode.HARDWARE_MISSION), [])

    def test_verified_config_passes_hardware_mission_gate(self) -> None:
        config = self._load_modified({
            "c10b_diff_firmware_verified = false": "c10b_diff_firmware_verified = true",
            "reference_measured = false": "reference_measured = true",
            'protocol_mode = "ackermann_firmware_compat"': 'protocol_mode = "differential_vx_vz"',
        })
        self.assertEqual(validate_runtime_readiness(config, RuntimeMode.HARDWARE_MISSION), [])

    def test_unverified_differential_firmware_is_rejected_for_hardware_mission(self) -> None:
        config = self._load_modified({
            'protocol_mode = "ackermann_firmware_compat"': 'protocol_mode = "differential_vx_vz"',
        })
        errors = validate_runtime_readiness(config, RuntimeMode.HARDWARE_MISSION)
        self.assertTrue(any("verified C10B differential firmware" in error for error in errors))

    def test_hardware_probe_is_capped_and_disallows_autonomous_navigation(self) -> None:
        constraints = runtime_constraints(load_v2_config(), RuntimeMode.HARDWARE_PROBE)
        self.assertFalse(constraints.autonomous_navigation_allowed)
        self.assertEqual(constraints.max_linear_speed_m_s, 0.10)
        self.assertEqual(constraints.max_angular_speed_rad_s, 0.35)

    def test_v1_profile_is_rejected_with_migration_instruction(self) -> None:
        with WorkspaceTempDir() as directory:
            path = Path(directory) / "v1.toml"
            path.write_text("schema_version = 1\n", encoding="utf-8")
            with self.assertRaisesRegex(ConfigV2Error, "schema-v2"):
                load_v2_config(path)


    def test_retired_relay_config_is_rejected(self):
        with self.assertRaisesRegex(ConfigV2Error, "unknown device table"):
            self._load_modified({"[devices.t265]": '[devices.relay]\nenabled = true\n\n[devices.t265]'})

    def test_unknown_device_table_is_rejected(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "unknown device table"):
            self._load_modified({"[devices.t265]": '[devices.unknown]\nport = "x"\n\n[devices.t265]'})

    def test_missing_required_device_table_is_rejected(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, r"\[devices.d500\]"):
            self._load_modified({"[devices.d500]": "[devices.renamed_d500]"})


if __name__ == "__main__":
    unittest.main()
