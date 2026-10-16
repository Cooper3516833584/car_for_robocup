"""Mocked permissions, sysfs export and CLI tests; no real device output."""
from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch, call

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools import setup_payload_pwm_permissions as permissions
from tools import payload_servo_test as cli


class PermissionTests(unittest.TestCase):
    def fixture(self, exported=True):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.chips = [root / ('pwmchip%d' % n) for n in (8, 2, 11, 15, 18)]
        for chip in self.chips:
            chip.mkdir()
            for name in ('export', 'unexport'): (chip / name).write_text('')
            if exported: self.export(chip)
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        self.resolve = self.stack.enter_context(patch.object(permissions, 'resolve_chip', side_effect=self.chips))
        self.chmod = self.stack.enter_context(patch.object(Path, 'chmod'))
        self.settle = self.stack.enter_context(patch.object(permissions, '_settle_udev'))
        self.stack.enter_context(patch.object(permissions.stat, 'S_IMODE', return_value=0o777))
        self.stack.enter_context(patch('builtins.print'))
        return root

    def export(self, chip):
        pwm = chip / 'pwm0'
        pwm.mkdir()
        for name, value in (('enable', '0'), ('period', 'old period'),
                            ('duty_cycle', 'old duty'), ('polarity', 'normal')):
            (pwm / name).write_text(value)

    def test_exact_five_nodes_and_permissions_without_setting_pwm(self):
        root = self.fixture()
        permissions.setup(root)
        self.assertEqual(self.resolve.call_args_list, [call(root, node) for node in
            ('fd8b0000.pwm', 'fd8b0010.pwm', 'febe0030.pwm', 'febf0020.pwm', 'febd0030.pwm')])
        self.assertEqual(self.chmod.call_count, 35)
        self.assertTrue(all(c.args == (0o777,) for c in self.chmod.call_args_list))
        for chip in self.chips:
            self.assertEqual((chip / 'export').read_text(), '')
            self.assertEqual((chip / 'pwm0/enable').read_text(), '0')
            self.assertEqual((chip / 'pwm0/duty_cycle').read_text(), 'old duty')
            self.assertEqual((chip / 'pwm0/period').read_text(), 'old period')

    def test_export_waits_for_channel_and_does_not_enable_it(self):
        root = self.fixture(False)
        pending = iter(self.chips)
        def settled():
            self.assertTrue(all((chip / 'pwm0').exists() for chip in self.chips))
            self.chmod.assert_not_called()
        self.settle.side_effect = settled
        permissions.setup(root, sleep=lambda _: self.export(next(pending)))
        self.settle.assert_called_once_with()
        for chip in self.chips:
            self.assertEqual((chip / 'export').read_text(), '0\n')
            self.assertEqual((chip / 'pwm0/enable').read_text(), '0')

    def test_missing_overlay_is_reported_and_other_nodes_attempted(self):
        root = self.fixture()
        self.resolve.side_effect = [RuntimeError('missing overlay'), *self.chips[1:]]
        with self.assertRaisesRegex(RuntimeError, 'fd8b0000.pwm: missing overlay'):
            permissions.setup(root)
        self.assertEqual(self.resolve.call_count, 5)
        self.assertEqual(self.chmod.call_count, 28)

    def test_chmod_error_and_ineffective_chmod_are_failures(self):
        root = self.fixture()
        self.chmod.side_effect = PermissionError('kernel denied chmod')
        with self.assertRaisesRegex(RuntimeError, 'kernel denied chmod'): permissions.setup(root)
        self.resolve.side_effect = self.chips
        self.chmod.side_effect = None
        with patch.object(permissions.stat, 'S_IMODE', return_value=0o644), \
             self.assertRaisesRegex(RuntimeError, 'did not take effect'):
            permissions.setup(root)

    def test_udev_failure_does_not_claim_permissions_are_ready(self):
        root = self.fixture()
        self.settle.side_effect = RuntimeError('events did not settle')
        with self.assertRaisesRegex(RuntimeError, 'events did not settle'):
            permissions.setup(root)
        self.chmod.assert_not_called()

    def test_udev_wait_is_bounded_and_checked(self):
        import subprocess
        with patch.object(permissions.subprocess, 'run') as run:
            permissions._settle_udev()
        run.assert_called_once_with(['/usr/bin/udevadm', 'settle', '--timeout=10'],
                                    check=True, timeout=12)
        with patch.object(permissions.subprocess, 'run', side_effect=subprocess.TimeoutExpired('udevadm', 12)), \
             self.assertRaisesRegex(RuntimeError, 'did not settle'):
            permissions._settle_udev()

    def test_service_runs_root_preparation_and_installer_rewrites_paths(self):
        service = (ROOT / 'deploy/competition-boot.service').read_text()
        self.assertIn('ExecStartPre=+/usr/bin/python3 /home/radxa/car/tools/setup_payload_pwm_permissions.py', service)
        self.assertIn('User=radxa', service)
        self.assertIn('/code/competition_boot.py', service)
        self.assertNotIn('/code/car_full_flow_test.py', service)
        self.assertNotIn('--listen', service)
        self.assertNotIn('no drone or payload output', service)
        install = (ROOT / 'deploy/install_car_services.sh').read_text()
        self.assertIn('import competition_boot, main_robocup;', install)
        self.assertNotIn('import competition_boot, car_full_flow_test;', install)
        self.assertIn('s#/home/radxa/car#${REPO_ROOT}#g', install)
        self.assertIn('install_one "$REPO_ROOT/deploy/competition-boot.service"', install)


class PayloadCLITests(unittest.TestCase):
    def test_actions_delegate_to_existing_component_and_hold_on_exit(self):
        with patch.object(cli, 'open_payload', return_value=True) as opened, \
             patch.object(cli, 'close_payload', return_value=True) as closed:
            for slot in ('0', '1', '2', '3', 'all'): self.assertEqual(cli.main(['open', slot]), 0)
            self.assertEqual(opened.call_args_list, [call(0), call(1), call(2), call(3), call(None)])
            self.assertEqual(cli.main(['close', 'all']), 0)
            closed.assert_called_once_with()
            self.assertEqual(opened.call_count, 5)  # no opposite command on normal exit

    def test_false_exception_and_invalid_slot_are_failures(self):
        with patch.object(cli, 'open_payload', return_value=False): self.assertEqual(cli.main(['open', '0']), 1)
        with patch.object(cli, 'close_payload', side_effect=OSError('PWM denied')), \
             patch.object(cli, 'open_payload', return_value=True) as opened:
            self.assertEqual(cli.main(['close', 'all']), 1)
        opened.assert_called_once_with()
        with patch.object(cli, 'close_payload') as closed, self.assertRaises(SystemExit): cli.main(['close', '0'])
        closed.assert_not_called()


if __name__ == '__main__': unittest.main()
