"""The bounded T265-only probe must release its sensor without a motor."""

from __future__ import annotations

from contextlib import redirect_stdout
import io
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "code"))

from components.t265_driver import T265RawPose  # noqa: E402
from tools import t265_probe  # noqa: E402


class FakeSource:
    serial = "test-t265"

    def __init__(self) -> None:
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def read(self) -> T265RawPose:
        now = time.monotonic()
        return T265RawPose(
            translation_xyz=(0.0, 0.0, 0.0),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            velocity_xyz=None,
            angular_velocity_xyz=None,
            tracker_confidence=3,
            mapper_confidence=3,
            device_timestamp_ms=0.0,
            received_monotonic_s=now,
        )

    def stop(self) -> None:
        self.stopped = True


class T265ProbeTests(unittest.TestCase):
    def test_bounded_corrected_probe_stops_source(self) -> None:
        source = FakeSource()
        output = io.StringIO()
        profile = REPO_ROOT / "configs" / "robocup_diffdrive.example.toml"
        with patch.object(t265_probe, "RealSenseT265PoseSource", return_value=source), redirect_stdout(output):
            result = t265_probe.main(["--config", str(profile), "--seconds", "0.001"])
        self.assertEqual(result, 0)
        self.assertTrue(source.started)
        self.assertTrue(source.stopped)
        self.assertIn("base_x_cm=", output.getvalue())
        self.assertIn("yaw_ccw_deg=", output.getvalue())


if __name__ == "__main__":
    unittest.main()
