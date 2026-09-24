from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from config.loader import load_car_config
from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config
from config.v2_models import ConfigV2Error
from config.v2_runtime import RuntimeMode, runtime_constraints, validate_runtime_readiness


class ConfigV2Tests(unittest.TestCase):
    def test_example_loads_as_schema_v2(self) -> None:
        config = load_v2_config(DEFAULT_V2_CONFIG)
        self.assertEqual(config.schema_version, 2)
        self.assertFalse(config.calibration.geometry_measured)
        self.assertEqual(config.drive.protocol_mode, "ackermann_firmware_compat")
        self.assertEqual(config.d500_mount.z_m, 0.20)
        self.assertEqual(config.t265_mount.z_m, 0.25)

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
            self._load_modified({"drive_track_width_m = 0.164": "drive_track_width_m = 0.0"})

    def test_unknown_protocol_is_rejected(self) -> None:
        with self.assertRaisesRegex(ConfigV2Error, "protocol_mode"):
            self._load_modified({
                'protocol_mode = "ackermann_firmware_compat"': 'protocol_mode = "unknown"'
            })

    def test_dry_run_allows_unmeasured_geometry(self) -> None:
        config = load_v2_config()
        self.assertEqual(validate_runtime_readiness(config, RuntimeMode.DRY_RUN), [])

    def test_hardware_mission_rejects_unmeasured_geometry_and_extrinsics(self) -> None:
        errors = validate_runtime_readiness(load_v2_config(), RuntimeMode.HARDWARE_MISSION)
        self.assertTrue(any("measured drive geometry" in error for error in errors))
        self.assertTrue(any("sensor extrinsics" in error for error in errors))

    def test_measured_and_verified_config_passes_hardware_mission_gate(self) -> None:
        config = self._load_modified({
            "geometry_measured = false": "geometry_measured = true",
            "sensor_extrinsics_measured = false": "sensor_extrinsics_measured = true",
            "c10b_diff_firmware_verified = false": "c10b_diff_firmware_verified = true",
            'protocol_mode = "ackermann_firmware_compat"': 'protocol_mode = "differential_vx_vz"',
        })
        self.assertEqual(validate_runtime_readiness(config, RuntimeMode.HARDWARE_MISSION), [])

    def test_unverified_differential_firmware_is_rejected_for_hardware_mission(self) -> None:
        config = self._load_modified({
            'protocol_mode = "ackermann_firmware_compat"': 'protocol_mode = "differential_vx_vz"',
            "geometry_measured = false": "geometry_measured = true",
            "sensor_extrinsics_measured = false": "sensor_extrinsics_measured = true",
        })
        errors = validate_runtime_readiness(config, RuntimeMode.HARDWARE_MISSION)
        self.assertTrue(any("verified C10B differential firmware" in error for error in errors))

    def test_hardware_probe_is_capped_and_disallows_autonomous_navigation(self) -> None:
        constraints = runtime_constraints(load_v2_config(), RuntimeMode.HARDWARE_PROBE)
        self.assertFalse(constraints.autonomous_navigation_allowed)
        self.assertEqual(constraints.max_linear_speed_m_s, 0.10)
        self.assertEqual(constraints.max_angular_speed_rad_s, 0.35)

    def test_v1_profile_remains_on_legacy_loader(self) -> None:
        root = Path(__file__).resolve().parents[2]
        legacy = load_car_config(root / "configs" / "car.example.toml")
        self.assertEqual(legacy.schema_version, 1)
        with self.assertRaisesRegex(ConfigV2Error, "load_car_config"):
            load_v2_config(root / "configs" / "car.example.toml")


if __name__ == "__main__":
    unittest.main()
