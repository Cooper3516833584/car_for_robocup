from __future__ import annotations

import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from tools import motion_diag_snapshot, yaw_truth_analyze  # noqa: E402


class MotionDiagToolsTests(unittest.TestCase):
    def test_snapshot_reads_example_config_without_hardware(self) -> None:
        payload = motion_diag_snapshot.snapshot(ROOT / "configs" / "robocup_diffdrive.example.toml")
        self.assertIn("git_head", payload)
        self.assertIn("source_sha256", payload)
        self.assertIn("t265_mount", payload)
        self.assertEqual(payload["protocol_mode"], "ackermann_firmware_compat")
        self.assertFalse(payload["c10b_diff_firmware_verified"])

    def test_yaw_truth_aligns_adapter_fusion_and_external_csv(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            events = root / "events.jsonl"
            truth = root / "truth.csv"
            rows = [
                {"t": 0.0, "type": "fused_pose", "yaw_rad": 0.0},
                {"t": 0.0, "type": "t265_pose", "yaw_rad": 0.0,
                 "raw_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                 "raw_angular_velocity_xyz": [0.0, 0.0, 0.0]},
                {"t": 0.0, "type": "drive_command", "limited_omega_rad_s": 0.2},
                {"t": 0.5, "type": "fused_pose", "yaw_rad": 0.1},
                {"t": 0.5, "type": "t265_pose", "yaw_rad": 0.1,
                 "raw_quaternion_xyzw": [0.0, 0.0, 0.05, 0.9987492178],
                 "raw_angular_velocity_xyz": [0.0, 0.0, 0.2]},
                {"t": 0.5, "type": "slam_anchor", "accepted": False,
                 "rejection_reason": "slam_innovation_gate", "innovation_m": 0.2,
                 "innovation_yaw_rad": 0.03, "candidate_count": 1},
                {"t": 1.5, "type": "fused_pose", "yaw_rad": 0.2},
                {"t": 1.5, "type": "t265_pose", "yaw_rad": 0.2,
                 "raw_quaternion_xyzw": [0.0, 0.0, 0.1, 0.9949874371],
                 "raw_angular_velocity_xyz": [0.0, 0.0, 0.0]},
            ]
            events.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            with truth.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=["event_t", "yaw_deg", "physical_stop"])
                writer.writeheader()
                writer.writerows([
                    {"event_t": 0.0, "yaw_deg": 0, "physical_stop": ""},
                    {"event_t": 0.5, "yaw_deg": 5, "physical_stop": "true"},
                    {"event_t": 1.5, "yaw_deg": 10, "physical_stop": ""},
                    {"event_t": 4.0, "yaw_deg": 10, "physical_stop": ""},
                ])
            result = yaw_truth_analyze.analyze(
                events, truth, time_offset_s=0.0, sync_uncertainty_ms=10.0,
            )
        self.assertEqual(result["summary"]["truth_samples"], 4)
        self.assertAlmostEqual(result["summary"]["external_yaw_change_deg"], 10.0)
        self.assertEqual(result["aligned_samples"][1]["slam_anchor"]["accepted"], False)
        self.assertEqual(result["aligned_samples"][1]["raw_angular_velocity_xyz"], [0.0, 0.0, 0.2])
        self.assertEqual(result["summary"]["external_yaw_after_physical_stop_deg"]["1.0"], 5.0)


if __name__ == "__main__":
    unittest.main()
