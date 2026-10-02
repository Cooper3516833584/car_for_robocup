"""Convert one complete D500 revolution to a regular ROS scan grid.

This module has no ROS or serial dependency.  Angles remain in the lidar frame;
the configured lidar mount is published separately as a static transform.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .radar_driver import RadarScan


BEAM_COUNT = 720
ANGLE_MIN_RAD = -math.pi
ANGLE_INCREMENT_RAD = 2.0 * math.pi / BEAM_COUNT
RANGE_MIN_M = 0.10
RANGE_MAX_M = 10.0
FALLBACK_SCAN_TIME_S = 0.14


@dataclass(frozen=True, slots=True)
class LaserScanGrid:
    ranges_m: tuple[float, ...]
    scan_time_s: float
    time_increment_s: float
    angle_min_rad: float = ANGLE_MIN_RAD
    angle_increment_rad: float = ANGLE_INCREMENT_RAD
    range_min_m: float = RANGE_MIN_M
    range_max_m: float = RANGE_MAX_M

    @property
    def angle_max_rad(self) -> float:
        # LaserScan samples include angle_min and exclude +pi for 720 bins.
        return self.angle_min_rad + (BEAM_COUNT - 1) * self.angle_increment_rad


class D500LaserScanConverter:
    def __init__(self) -> None:
        self._last_scan_time_s = FALLBACK_SCAN_TIME_S

    def convert(self, scan: RadarScan) -> LaserScanGrid:
        if scan.rotation_speed_deg_s > 0:
            self._last_scan_time_s = 360.0 / scan.rotation_speed_deg_s
        ranges = [math.inf] * BEAM_COUNT
        for point in scan.points:
            distance_m = point.distance_mm / 1000.0
            if (not math.isfinite(point.angle_cw_deg)
                    or not math.isfinite(distance_m)
                    or not RANGE_MIN_M <= distance_m <= RANGE_MAX_M):
                continue
            # D500 angle grows clockwise; ROS angle grows counter-clockwise.
            angle = -math.radians(point.angle_cw_deg)
            index = round((angle - ANGLE_MIN_RAD) / ANGLE_INCREMENT_RAD) % BEAM_COUNT
            ranges[index] = min(ranges[index], distance_m)
        scan_time = self._last_scan_time_s
        return LaserScanGrid(tuple(ranges), scan_time, scan_time / BEAM_COUNT)
