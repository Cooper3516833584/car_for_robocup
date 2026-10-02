from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_driver import RadarScan
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import build_runtime


class _ScanSink:
    def __init__(self) -> None:
        self.calls = []

    def push_d500_scan(self, scan, measurement_s):
        self.calls.append((scan, measurement_s))


class SlamRuntimeWiringTests(unittest.TestCase):
    def test_complete_scan_reaches_bridge_even_when_icp_rejects(self) -> None:
        runtime = build_runtime(load_v2_config(), RuntimeMode.DRY_RUN, clock=lambda: 1.0)
        sink = _ScanSink()
        runtime.slam_bridge = sink
        scan = RadarScan((), 1234, 2500)
        update = SimpleNamespace(
            scan=scan,
            odometry=SimpleNamespace(accepted=False, rejection_reason="icp_rejected"),
            local_pose_valid=False,
            absolute_observation_available=False,
        )
        runtime.on_d500_update(update)
        self.assertEqual(len(sink.calls), 1)
        self.assertIs(sink.calls[0][0], scan)
        runtime.slam_bridge = None
        runtime.close()


if __name__ == "__main__":
    unittest.main()
