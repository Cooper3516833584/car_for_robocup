from __future__ import annotations

from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.sensor_clock import DeviceClockMapper


class DeviceClockMapperTests(unittest.TestCase):
    def test_t265_received_jitter_does_not_change_measurement_intervals(self) -> None:
        mapper = DeviceClockMapper()
        device_ms = (1000, 1050, 1100, 1150)
        received_s = (10.005, 10.110, 10.165, 10.240)
        mapped = [mapper.map_milliseconds(d, r) for d, r in zip(device_ms, received_s)]
        intervals = [right - left for left, right in zip(mapped, mapped[1:])]
        for interval in intervals:
            self.assertAlmostEqual(interval, 0.05, places=9)

    def test_device_clock_reset_reanchors_without_time_rollback(self) -> None:
        mapper = DeviceClockMapper()
        samples = ((1000, 10.0), (1050, 10.05), (1100, 10.1),
                   (10, 10.2), (60, 10.25), (110, 10.3))
        mapped = [mapper.map_milliseconds(device, received) for device, received in samples]
        self.assertEqual(mapped, sorted(mapped))
        self.assertAlmostEqual(mapped[4] - mapped[3], 0.05)
        self.assertAlmostEqual(mapped[5] - mapped[4], 0.05)

    def test_wrapping_counter_and_counter_reset(self) -> None:
        mapper = DeviceClockMapper(modulus_ms=0x10000)
        first = mapper.map_milliseconds(65530, 10.0)
        wrapped = mapper.map_milliseconds(10, 10.2)
        reset = mapper.map_milliseconds(5, 10.3)
        self.assertAlmostEqual(wrapped - first, 0.016)
        self.assertGreaterEqual(reset, wrapped)
        self.assertAlmostEqual(reset, 10.3)


if __name__ == "__main__":
    unittest.main()
