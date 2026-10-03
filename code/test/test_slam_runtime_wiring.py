from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_driver import RadarScan
from components.slam_scan_source import ScanOnlyD500Source
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import build_runtime


class _ScanSink:
    def __init__(self) -> None:
        self.calls = []

    def push_d500_scan(self, scan, measurement_s):
        self.calls.append((scan, measurement_s))


class SlamRuntimeWiringTests(unittest.TestCase):
    def test_scan_timestamp_cannot_lead_host_receipt(self) -> None:
        scan = RadarScan((), 1234, 2500)
        for mapped_s, expected_s in ((10.16, 10.0), (9.98, 9.98)):
            with self.subTest(mapped_s=mapped_s):
                sink = _ScanSink()
                runtime = SimpleNamespace(
                    slam_bridge=sink,
                    _map_d500_timestamp=lambda _timestamp, _received: mapped_s,
                )
                source = ScanOnlyD500Source(runtime, "/dev/null", 230400)
                source.assembler.feed = lambda _packet: [scan]
                with patch("components.slam_scan_source.time.monotonic", return_value=10.0):
                    source._on_packet(object())
                self.assertEqual(sink.calls, [(scan, expected_s)])

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
