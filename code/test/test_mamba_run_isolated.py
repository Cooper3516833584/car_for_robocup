"""Listener starts must survive damaged prior registries without changing vision."""

from pathlib import Path
import json
import os
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import mamba_run_isolated as launcher


class IsolatedMambaTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def launch(self, environment):
        return launcher.prepare_launch(self.root, '/mamba', '/environment',
                                       ['/python', '/listener.py'], environ=environment)

    def test_each_start_ignores_empty_malformed_and_valid_old_process_files(self):
        previous = self.root / 'launch-previous' / 'mamba' / 'proc'
        previous.mkdir(parents=True)
        contents = {'1.json': b'', '2.json': b'{broken', '3.json': b'{"name":"active"}'}
        for name, data in contents.items():
            (previous / name).write_bytes(data)
        _, first = self.launch({})
        _, second = self.launch({})
        self.assertNotEqual(first['XDG_CACHE_HOME'], second['XDG_CACHE_HOME'])
        for environment in (first, second):
            cache = Path(environment['XDG_CACHE_HOME'])
            self.assertEqual(cache.parent, self.root)
            self.assertEqual(list(cache.iterdir()), [])
        for name, data in contents.items():
            self.assertEqual((previous / name).read_bytes(), data)

    def test_child_unsets_cache_override_when_originally_absent(self):
        original = {'PATH': '/original/bin', 'PYTHONPATH': '/vision', 'MAMBA_ROOT_PREFIX': '/root'}
        command, environment = self.launch(original)
        self.assertEqual(command, ['/mamba', 'run', '-p', '/environment', '/usr/bin/env',
                                   '-u', 'XDG_CACHE_HOME', '/python', '/listener.py'])
        self.assertEqual({k: v for k, v in environment.items() if k != 'XDG_CACHE_HOME'}, original)
        self.assertNotIn('XDG_CACHE_HOME', original)

    def test_child_restores_existing_cache_including_spaces_and_empty_value(self):
        for original_cache in ('/saved vision/cache', ''):
            with self.subTest(cache=original_cache):
                original = {'XDG_CACHE_HOME': original_cache, 'PATH': '/original/bin'}
                command, environment = self.launch(original)
                self.assertEqual(command[5:], ['XDG_CACHE_HOME=' + original_cache,
                                               '/python', '/listener.py'])
                self.assertNotEqual(environment['XDG_CACHE_HOME'], original_cache)
                self.assertEqual(original['XDG_CACHE_HOME'], original_cache)

    def test_launcher_executes_once_without_wait_retry_or_background_process(self):
        with patch.object(launcher.os, 'execvpe') as execute:
            launcher.main(['--cache-root', str(self.root), '/mamba', '/environment',
                           '/python', '/listener.py', '--flag'])
        execute.assert_called_once()
        executable, command, environment = execute.call_args.args
        self.assertEqual(executable, '/mamba')
        self.assertEqual(command[-3:], ['/python', '/listener.py', '--flag'])
        self.assertTrue(Path(environment['XDG_CACHE_HOME']).is_dir())

    def test_missing_child_fails_before_creating_cache(self):
        with self.assertRaises(ValueError):
            launcher.prepare_launch(self.root / 'unused', '/mamba', '/env', [], environ={})
        self.assertFalse((self.root / 'unused').exists())

    def test_service_uses_runtime_cache_and_existing_formal_listener(self):
        service = (ROOT / 'deploy/competition-boot.service').read_text()
        self.assertIn('RuntimeDirectory=robocup-car', service)
        self.assertIn('RuntimeDirectoryPreserve=no', service)
        self.assertIn('--cache-root /run/robocup-car/mamba-cache', service)
        start = next(line for line in service.splitlines() if line.startswith('ExecStart='))
        self.assertIn('/tools/mamba_run_isolated.py', start)
        self.assertTrue(start.endswith('/code/competition_boot.py'))
        self.assertIn('KillMode=control-group', service)


@unittest.skipUnless(os.environ.get('MAMBA_TEST_EXE') and os.environ.get('MAMBA_TEST_PREFIX'),
                     'opt-in real micromamba test; uses temporary caches and no hardware')
class RealMambaIsolationTests(unittest.TestCase):
    def test_corrupt_shared_and_previous_launch_caches_do_not_block_start(self):
        mamba, prefix = os.environ['MAMBA_TEST_EXE'], os.environ['MAMBA_TEST_PREFIX']
        with tempfile.TemporaryDirectory(prefix='mamba-isolation-test-') as directory:
            shared = Path(directory) / 'shared'
            proc = shared / 'mamba' / 'proc'
            proc.mkdir(parents=True)
            private = Path(directory) / 'private'
            probe = ('import json, os; print(json.dumps({key: os.environ.get(key) for key in '
                     "('XDG_CACHE_HOME', 'CONDA_PREFIX', 'PYTHONPATH', 'ROS_LOCALHOST_ONLY')}))")
            for damaged_data in (b'', b'{broken'):
                (proc / '99999999.json').write_bytes(damaged_data)
                for restore_existing in (False, True):
                    with self.subTest(data=damaged_data, restore_existing=restore_existing):
                        environment = dict(os.environ, PYTHONPATH='/preserved/vision/path',
                                           ROS_LOCALHOST_ONLY='1')
                        environment.pop('XDG_CACHE_HOME', None)
                        if restore_existing:
                            environment['XDG_CACHE_HOME'] = str(shared)
                        baseline = subprocess.run(
                            [mamba, 'run', '-p', prefix, sys.executable, '-c', 'pass'],
                            env=dict(environment, XDG_CACHE_HOME=str(shared)),
                            capture_output=True, text=True, timeout=20)
                        self.assertNotEqual(baseline.returncode, 0)
                        self.assertIn('parse_error', baseline.stderr)
                        launched = subprocess.run(
                            [sys.executable, str(ROOT / 'tools/mamba_run_isolated.py'),
                             '--cache-root', str(private), mamba, prefix,
                             sys.executable, '-c', probe], env=environment,
                            capture_output=True, text=True, timeout=20)
                        self.assertEqual(launched.returncode, 0, launched.stderr)
                        child = json.loads(launched.stdout.splitlines()[-1])
                        self.assertEqual(child['XDG_CACHE_HOME'],
                                         str(shared) if restore_existing else None)
                        self.assertEqual(child['CONDA_PREFIX'], prefix)
                        self.assertEqual(child['PYTHONPATH'], '/preserved/vision/path')
                        self.assertEqual(child['ROS_LOCALHOST_ONLY'], '1')
                        self.assertEqual((proc / '99999999.json').read_bytes(), damaged_data)
                        # Mimic a crashed earlier invocation in the private root.
                        # The next launch must use a different, clean directory.
                        for old in private.iterdir():
                            old_proc = old / 'mamba' / 'proc'
                            old_proc.mkdir(parents=True, exist_ok=True)
                            (old_proc / '99999998.json').write_bytes(damaged_data)


if __name__ == '__main__':
    unittest.main()
