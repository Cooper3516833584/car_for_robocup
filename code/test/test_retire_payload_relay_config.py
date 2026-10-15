"""Pure TOML migration tests; no board or hardware access."""
from pathlib import Path
import sys
import tomllib
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.retire_payload_relay_config import without_relay


class RelayProfileRetirementTests(unittest.TestCase):
    def test_only_relay_table_removed_preserving_real_calibration(self):
        source = ('schema_version = 2\n[devices.c10b]\nport="/dev/ttyACM0"\n'
                  '[devices.relay]\nenabled=false\nport="/dev/old"\n'
                  'channel_count=4\n\n# Camera calibration\n[devices.servo]\n'
                  'chip_device_match=["febd0030.pwm"]\npulse_min_us=500\n'
                  '[sensors.d500.mount]\nx_m=0.123\ny_m=-0.047\n')
        result = without_relay(source)
        self.assertNotIn('[devices.relay]', result)
        self.assertIn('# Camera calibration', result)
        before = tomllib.loads(source)
        del before['devices']['relay']
        self.assertEqual(tomllib.loads(result), before)

    def test_no_retired_table_is_byte_for_byte_unchanged(self):
        source = '# Real profile\n[devices.servo]\nenabled=true\n'
        self.assertEqual(without_relay(source), source)

    def test_last_section_and_crlf(self):
        source = '[devices.c10b]\r\nport="motor"\r\n[devices.relay]\r\nenabled=false\r\n'
        self.assertEqual(without_relay(source), '[devices.c10b]\r\nport="motor"\r\n')

    def test_unhandled_nested_relay_table_fails_before_writing(self):
        with self.assertRaises(ValueError):
            without_relay('[devices.relay]\nenabled=false\n[devices.relay.extra]\nx=1\n')


if __name__ == '__main__':
    unittest.main()
