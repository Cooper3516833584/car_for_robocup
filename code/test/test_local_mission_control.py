"""Local launcher and IPC checks; never create a real mission or open hardware."""
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workspace_temp import WorkspaceTempDir
import car_full_flow_test as entry
from components import local_mission_control as control
from components.local_mission_control import LocalMissionControl, request_local


class LocalEntryTests(unittest.TestCase):
    def test_client_connection_error_does_not_send_a_stop(self):
        with patch.object(control.shutil, 'which', return_value=None), \
                patch.object(control, 'request_local', side_effect=[dict(ok=True), OSError('lost')]) as request:
            self.assertEqual(control.run_local(), 2)
        self.assertEqual([c.args[0] for c in request.call_args_list], ['start', 'status'])

    def test_options_are_serialized_with_the_start_request(self):
        from unittest.mock import MagicMock
        connection = MagicMock()
        connection.__enter__.return_value = connection
        connection.makefile.return_value.__enter__.return_value.readline.return_value = b'{"ok":true}\n'
        with patch.object(socket, 'AF_UNIX', 1, create=True), \
                patch.object(socket, 'socket', return_value=connection):
            request_local('start', options={'task_counts': [1, 2, 1], 'standard_drop_m': .51})
        data = json.loads(connection.__enter__.return_value.sendall.call_args.args[0])
        self.assertEqual(data['action'], 'start')
        self.assertEqual(data['options']['task_counts'], [1, 2, 1])
        self.assertEqual(data['options']['standard_drop_m'], .51)

    def test_no_arguments_from_vscode_start_local_run(self):
        with patch.object(entry, 'run_local', return_value=0) as run:
            self.assertEqual(entry.main([]), 0)
        run.assert_called_once_with()

    def test_status_never_starts_a_mission(self):
        with patch.object(entry, 'request_local', return_value=dict(ok=True, running=False)) as request, \
             patch.object(entry, 'run_local') as run:
            self.assertEqual(entry.main(['--status']), 0)
        request.assert_called_once_with('status')
        run.assert_not_called()

    def test_ctrl_c_stops_the_locally_started_mission(self):
        with patch.object(control.shutil, 'which', return_value=None), \
             patch.object(control, 'request_local', side_effect=[dict(ok=True), KeyboardInterrupt(), dict(ok=True)]) as request:
            self.assertEqual(control.run_local(), 130)
        self.assertEqual([call.args[0] for call in request.call_args_list], ['start', 'status', 'stop'])

    def test_busy_does_not_stop_someone_elses_mission(self):
        with patch.object(control.shutil, 'which', return_value=None), \
             patch.object(control, 'request_local', return_value=dict(ok=False, reason='BUSY')) as request:
            self.assertEqual(control.run_local(), 1)
        request.assert_called_once_with('start')


@unittest.skipUnless(os.name == 'posix' and hasattr(socket, 'AF_UNIX'), 'Unix socket integration runs on Linux')
class LocalSocketTests(unittest.TestCase):
    def setUp(self):
        self.directory = WorkspaceTempDir()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'control.sock'
        self.handler = Mock(side_effect=lambda action: dict(ok=True, action=action))
        self.server = LocalMissionControl(self.handler, self.path)
        self.server.bind()
        self.server.start()
        self.addCleanup(self.server.close)

    def test_round_trip_permissions_and_close(self):
        self.assertEqual(request_local('status', self.path), dict(ok=True, action='status'))
        self.handler.assert_called_once_with('status')
        self.assertEqual(self.path.stat().st_mode & 0o777, 0o600)
        self.server.close()
        self.assertFalse(self.path.exists())

    def test_second_listener_cannot_replace_running_socket(self):
        other = LocalMissionControl(Mock(), self.path)
        with self.assertRaises(OSError):
            other.bind()
        other.close()
        self.assertEqual(request_local('status', self.path)['action'], 'status')

    def test_bad_request_does_not_kill_listener_or_start_task(self):
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.connect(str(self.path))
            connection.sendall(b'bad json\n')
            self.assertFalse(json.loads(connection.makefile('rb').readline())['ok'])
        self.handler.assert_not_called()
        self.assertTrue(request_local('status', self.path)['ok'])
