from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.d500_laserscan import D500LaserScanConverter
from components.radar_driver import RadarPoint, RadarScan


class D500LaserScanTests(unittest.TestCase):
    def test_direction_scale_bins_and_nearest_return(self) -> None:
        scan = RadarScan((
            RadarPoint(0.0, 1200, 100),
            RadarPoint(90.0, 2000, 100),
            RadarPoint(270.0, 3000, 100),
            RadarPoint(0.1, 900, 100),
            RadarPoint(45.0, 0, 100),
        ), 10, 2571)
        grid = D500LaserScanConverter().convert(scan)
        self.assertEqual(len(grid.ranges_m), 720)
        self.assertAlmostEqual(grid.ranges_m[360], 0.9)
        self.assertAlmostEqual(grid.ranges_m[180], 2.0)
        self.assertAlmostEqual(grid.ranges_m[540], 3.0)
        self.assertTrue(math.isinf(grid.ranges_m[361]))
        self.assertFalse(any(math.isnan(value) for value in grid.ranges_m))
        self.assertAlmostEqual(grid.time_increment_s, grid.scan_time_s / 720)

    def test_speed_fallback_uses_last_valid_speed(self) -> None:
        converter = D500LaserScanConverter()
        empty = lambda speed: RadarScan((), 0, speed)
        self.assertAlmostEqual(converter.convert(empty(0)).scan_time_s, 0.14)
        self.assertAlmostEqual(converter.convert(empty(1800)).scan_time_s, 0.2)
        self.assertAlmostEqual(converter.convert(empty(0)).scan_time_s, 0.2)


if __name__ == "__main__":
    unittest.main()
