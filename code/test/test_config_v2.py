from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config
from config.v2_models import ConfigV2Error
from config.v2_runtime import RuntimeMode, runtime_constraints, validate_runtime_readiness


class ConfigV2Tests(unittest.TestCase):
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
        with tempfile.TemporaryDirectory() as directory:
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
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "v1.toml"
            path.write_text("schema_version = 1\n", encoding="utf-8")
            with self.assertRaisesRegex(ConfigV2Error, "schema-v2"):
                load_v2_config(path)

    def test_relay_defaults_to_disabled(self) -> None:
        config = load_v2_config()
        self.assertFalse(config.relay.enabled)
        self.assertEqual(config.relay.baudrate, 9600)
        self.assertEqual(config.relay.channel_count, 8)

    def test_profile_without_a_relay_table_still_loads_disabled(self) -> None:
        source = DEFAULT_V2_CONFIG.read_text(encoding="utf-8")
        marker = "# LCUS USB relay"
        self.assertIn(marker, source)
        # [devices.relay] is optional, so profiles written before it must still load.
        trimmed = source[: source.index(marker)] + source[source.index("[sensors.t265.mount]") :]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.toml"
            path.write_text(trimmed, encoding="utf-8")
            config = load_v2_config(path)
        self.assertFalse(config.relay.enabled)
        self.assertEqual(config.relay.channel_count, 8)

    def test_enabled_relay_is_parsed(self) -> None:
        config = self._load_modified({
            'enabled = false\nport = ""': 'enabled = true\nport = "/dev/relay_lcus"',
            "channel_count = 8": "channel_count = 4",
        })
        self.assertTrue(config.relay.enabled)
        self.assertEqual(config.relay.port, "/dev/relay_lcus")
        self.assertEqual(config.relay.channel_count, 4)

    def test_enabled_relay_requires_a_port(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "devices.relay.port"):
            self._load_modified({'enabled = false\nport = ""': 'enabled = true\nport = ""'})

    def test_relay_channel_count_is_bounded(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "channel_count"):
            self._load_modified({"channel_count = 8": "channel_count = 9"})
        with self.assertRaisesRegex(ConfigV2Error, "channel_count"):
            self._load_modified({"channel_count = 8": "channel_count = 0.5"})

    def test_relay_rejects_unknown_keys(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "devices.relay"):
            self._load_modified({"channel_count = 8": "channel_count = 8\npulse_ms = 50"})

    def test_unknown_device_table_is_rejected(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "unknown device table"):
            self._load_modified({"[devices.t265]": '[devices.unknown]\nport = "x"\n\n[devices.t265]'})

    def test_missing_required_device_table_is_rejected(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, r"\[devices.d500\]"):
            self._load_modified({"[devices.d500]": "[devices.renamed_d500]"})


if __name__ == "__main__":
    unittest.main()
