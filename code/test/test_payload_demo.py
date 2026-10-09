"""The filming shortcut must route to the normal localized patrol."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from tools import run_yellow_payload_demo as filming
from tools import run_center_target_route as entry

class ClosedLoopFilmingTests(unittest.TestCase):
    def test_filming_defaults_and_explicit_slot_override(self):
        with patch.object(filming,'route_main',return_value=0) as run:
            self.assertEqual(filming.main(['--payload-slot','3']),0)
            args=run.call_args.args[0]
            self.assertEqual(args[args.index('--payload-slot')+1],'2')
            self.assertEqual(args[-2:],['--payload-slot','3'])
            self.assertIn('yellow',args)
            self.assertNotIn('--demo-no-position-checks',args)
    def test_old_open_loop_switch_rejected(self):
        with self.assertRaises(SystemExit): entry.main(['--demo-no-position-checks'])
    def test_no_open_loop_factory(self):
        from config import v2_factory
        self.assertFalse(hasattr(v2_factory,'build_payload_demo_session'))

if __name__ == '__main__': unittest.main()
