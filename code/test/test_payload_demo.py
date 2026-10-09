"""The filming shortcut must route to the normal localized patrol."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch, Mock
import tempfile
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from tools import run_yellow_payload_demo as filming
from tools import run_center_target_route as entry
from components.rear_motor import DriverStateError

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

    def test_usb_stop_failure_keeps_failed_result_and_closes_every_resource(self):
        with tempfile.TemporaryDirectory() as tmp:
            weights=Path(tmp)/'weights.pt'
            weights.touch()
            log=Path(tmp)/'run'
            runtime=Mock()
            runtime.drive.stop.side_effect=DriverStateError('USB disconnected')
            runtime.relay.connected=True
            runtime.relay.all_off.return_value=True
            servo=Mock(is_running=True)
            vision=Mock()
            logger=Mock()
            with patch.object(entry,'build_runtime',return_value=runtime), \
                 patch.object(entry,'build_servo',return_value=servo), \
                 patch.object(entry,'YoloVision',return_value=vision), \
                 patch.object(entry,'JsonlEventLogger',return_value=logger), \
                 patch.object(entry,'run_route',side_effect=ValueError('original route failure')), \
                 patch.object(entry.signal,'signal'):
                code=entry.main(['--confirm-motor-test','--release-mode','relay','--relay-port','/dev/test-relay',
                                 '--weights',str(weights),'--log-dir',str(log)])
            self.assertEqual(code,1)
            self.assertEqual((log/'result.txt').read_text(),'FAILED: ValueError: original route failure\n')
            runtime.relay.all_off.assert_called_once_with(verify=True)
            runtime.close.assert_called_once()
            vision.close.assert_called_once()
            servo.close.assert_called_once_with(hold=True)
            logger.close.assert_called_once()
            self.assertTrue(logger.emit.called)

    def test_cleanup_failure_after_route_finish_changes_result_to_failed(self):
        with tempfile.TemporaryDirectory() as tmp:
            weights=Path(tmp)/'weights.pt'; weights.touch()
            log=Path(tmp)/'run'
            runtime=Mock()
            runtime.relay.connected=True
            runtime.relay.all_off.side_effect=OSError('relay failed')
            servo=Mock(is_running=True)
            with patch.object(entry,'build_runtime',return_value=runtime), \
                 patch.object(entry,'build_servo',return_value=servo), \
                 patch.object(entry,'YoloVision',return_value=Mock()), \
                 patch.object(entry,'JsonlEventLogger',return_value=Mock()), \
                 patch.object(entry,'run_route',return_value=0), \
                 patch.object(entry.signal,'signal'):
                code=entry.main(['--confirm-motor-test','--release-mode','relay','--relay-port','/dev/test-relay',
                                 '--weights',str(weights),'--log-dir',str(log)])
            self.assertEqual(code,1)
            self.assertIn('FAILED: cleanup: relay failed',(log/'result.txt').read_text())
            runtime.close.assert_called_once()

if __name__ == '__main__': unittest.main()
