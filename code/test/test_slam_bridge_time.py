from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.slam_bridge import RosTimeMapper, SlamBridge
from components.radar_driver import RadarScan
from config.v2_loader import load_v2_config
from core.types import Pose2D, PoseQuality


class SlamBridgeTimeTests(unittest.TestCase):
    def test_measurement_time_not_publish_time(self) -> None:
        mapper = RosTimeMapper(10_000_000_000, 2_000_000_000)
        self.assertEqual(mapper.to_ros_ns(2.25), 10_250_000_000)
        self.assertAlmostEqual(mapper.to_monotonic_s(10_250_000_000), 2.25)

    def test_one_slot_scan_and_t265_inputs(self) -> None:
        bridge = SlamBridge(load_v2_config().d500_mount)
        scan = RadarScan((), 0, 1000)
        bridge.push_d500_scan(scan, 1.0)
        bridge.push_d500_scan(scan, 1.1)
        good = PoseQuality("t265", True, False)
        bridge.push_t265(Pose2D(0.0, 0.0, 0.0, 1.0), good)
        bridge.push_t265(Pose2D(0.1, 0.0, 0.0, 1.1), good)
        self.assertEqual(bridge._pending_scan[1], 1.1)
        self.assertAlmostEqual(bridge._pending_t265[0].x_m, 0.1)
        self.assertEqual(bridge.metrics()["slam.bridge_queue_overwrite_count"], 2)


if __name__ == "__main__":
    unittest.main()
