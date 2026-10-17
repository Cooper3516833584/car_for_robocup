"""Fake clock, radio and outputs: never runs systemctl or opens hardware."""
from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from workspace_temp import WorkspaceTempDir, cleanup_temp_dir, temp_dir
import competition_boot as boot
from components.fleet_models import (AckReason, AckStatus, CommandId, CommandPayload, Frame,
                                    MessageKind, NodeId)
from components.fleet_protocol import encode_command, pack_frame, unpack_frame, decode_ack


class ClockEvent:
    def __init__(self, owner, cancel_at=None):
        self.owner, self.cancel_at, self.cancelled = owner, cancel_at, False
    def is_set(self): return self.cancelled
    def set(self): self.cancelled=True
    def wait(self, duration):
        target = self.owner.now+duration
        if self.cancel_at is not None and target >= self.cancel_at:
            self.owner.now=self.cancel_at
            self.set()
        else:
            self.owner.now=target
        return self.cancelled


class BootTests(unittest.TestCase):
    def test_listener_run_and_cleanup_command_all_five_real_payload_axes(self):
        from components import payload_servos as payload
        outputs = [Mock(is_running=False) for _ in range(5)]
        for output in outputs:
            output.start.side_effect = lambda output=output: setattr(output, 'is_running', True)
        self.stack.enter_context(patch.object(payload, '_axes', [None]*5))
        self.stack.enter_context(patch.object(payload, 'LinuxSysfsPWMOutput', side_effect=outputs))
        self.open.side_effect = payload.open_payload
        self.close.side_effect = payload.close_payload
        self.listener._car_only_test = True
        self.listener.start()
        self.run_worker()
        self.listener._cancel_current()
        self.listener.close()
        for output in outputs:
            pulses = [c.args[0] for c in output.set_pulse_us.call_args_list]
            self.assertEqual(pulses[:2], [1000,1500])
            self.assertTrue(pulses[2:])
            self.assertTrue(all(p == 1000 for p in pulses[2:]))

    def test_formal_and_car_only_worker_pass_the_real_profile_to_runtime_parser(self):
        from main_robocup import build_parser
        expected = Path(boot.__file__).resolve().parents[1] / 'configs/robocup_diffdrive.toml'
        for car_only in (False, True):
            with self.subTest(car_only=car_only):
                self.listener._car_only_test = car_only
                self.now = 0.
                self.mission.reset_mock()
                self.run_worker()
                self.mission.assert_called_once()
                args = build_parser().parse_args(self.mission.call_args.args[0])
                self.assertEqual(Path(args.config), expected)
                self.assertTrue(Path(args.config).is_absolute())
                self.assertTrue(args.competition)
                self.assertEqual(args.mode, 'hardware-mission')
                self.assertFalse(args.relative_slam,
                                 'the competition entry must not request the fused profile')

    def test_missing_explicit_real_profile_fails_without_falling_back_to_example(self):
        from main_robocup import main
        with WorkspaceTempDir() as directory:
            args = list(boot.MISSION_ARGS)
            args[args.index('--config') + 1] = str(Path(directory) / 'missing.toml')
            with patch('main_robocup.build_runtime') as build, \
                 patch('main_robocup.open_payload') as opened:
                self.assertEqual(main(args), 2)
            build.assert_not_called()
            opened.assert_not_called()

    def test_explicit_simulation_skips_all_payload_outputs_and_is_forwarded(self):
        self.listener._simulate_payload = True
        self.listener._car_only_test = True
        self.listener.start()
        self.run_worker()
        self.listener.close()
        self.open.assert_not_called()
        self.close.assert_not_called()
        self.assertTrue(self.mission.call_args.kwargs['simulate_payload'])

    def test_startup_false_open_is_logged_without_claiming_success(self):
        self.open.side_effect = None
        self.open.return_value = False
        with self.assertLogs(boot.LOG, level='INFO') as logs:
            self.listener.start()
        self.assertTrue(any('startup payload OPEN failed' in line for line in logs.output))
        self.assertFalse(any('startup payload OPEN command completed' in line for line in logs.output))

    def test_completed_worker_leaves_radio_listener_ready_for_next_run(self):
        self.listener._car_only_test = True
        self.listener.start()
        raw = lambda seq: pack_frame(Frame(1, NodeId.GROUND, NodeId.CAR, MessageKind.COMMAND,
            0, 10, seq, encode_command(CommandPayload(CommandId.CAR_START_MISSION))))
        with patch.object(boot, 'RUN_DELAY_S', 0), patch.object(boot, 'BUZZ_LAST_S', 0):
            for seq in (1, 2):
                self.listener._on_bytes(raw(seq))
                self.listener._worker.join(1)
                self.assertFalse(self.listener._worker.is_alive())
                ack = decode_ack(unpack_frame(self.driver.write.call_args.args[0]).payload)
                self.assertEqual(ack.status, AckStatus.ACCEPTED)
        self.assertEqual(self.mission.call_count, 2)
        self.assertEqual(self.close.call_count, 2)
        self.assertEqual(self.open.call_count, 3)
        self.driver.close.assert_not_called()

    def test_car_only_entry_always_selects_test_mode(self):
        import car_full_flow_test
        with patch.object(boot, 'main') as entry:
            car_full_flow_test.main(['--listen'])
        entry.assert_called_once_with(car_only_test=True, local_control=True)
        service = (Path(__file__).resolve().parents[2] / 'deploy/competition-boot.service').read_text()
        self.assertIn('/home/radxa/car/code/competition_boot.py', service)
        self.assertNotIn('/code/car_full_flow_test.py', service)
        self.assertNotIn('--listen', service)
        self.assertNotIn('--competition', service)
        self.assertIn('RuntimeDirectory=robocup-car', service)

    def test_default_main_only_starts_formal_listener(self):
        done = Mock()
        with patch.object(boot.threading, 'Event', return_value=done), \
             patch.object(boot.signal, 'signal'), \
             patch.object(boot, 'CompetitionBoot') as listener, \
             patch('components.local_mission_control.LocalMissionControl') as control:
            boot.main()
        listener.assert_called_once_with(car_only_test=False)
        listener.return_value.start.assert_called_once_with()
        listener.return_value._start_mission.assert_not_called()
        done.wait.assert_called_once_with()
        listener.return_value.close.assert_called_once_with()

    def test_local_run_and_ground_run_share_busy_gate_and_ground_stop(self):
        self.listener._car_only_test = True
        entered = threading.Event()
        def mission(*args, **kwargs):
            entered.set()
            kwargs['stop_event'].wait(2)
            return 0
        self.mission.side_effect = mission
        with patch.object(boot, 'RUN_DELAY_S', 0), patch.object(boot, 'BUZZ_LAST_S', 0):
            result = self.listener.local_command('start')
            self.assertTrue(result['ok'])
            self.assertTrue(entered.wait(1))
            try:
                self.assertTrue(self.listener.local_command('status')['running'])
                self.assertEqual(self.listener.local_command('start')['reason'], 'BUSY')
                for seq, command, expected in ((1, CommandId.CAR_START_MISSION, AckStatus.REJECTED),
                                                (2, CommandId.TARGETED_STOP, AckStatus.ACCEPTED)):
                    self.listener._on_bytes(pack_frame(Frame(1, NodeId.GROUND, NodeId.CAR,
                        MessageKind.COMMAND, 0, 50, seq, encode_command(CommandPayload(command)))))
                    ack = decode_ack(unpack_frame(self.driver.write.call_args.args[0]).payload)
                    self.assertEqual(ack.status, expected)
            finally:
                self.listener.local_command('stop')
                self.listener._worker.join(2)
        self.assertFalse(self.listener.local_command('status')['running'])
        self.assertEqual(self.mission.call_count, 1)
        self.driver.close.assert_not_called()

    def test_formal_listener_accepts_local_status_without_starting(self):
        self.assertTrue(self.listener.local_command('status')['ok'])
        self.mission.assert_not_called()

    def test_local_run_options_are_forwarded_once_without_changing_radio_defaults(self):
        from components.competition_options import CompetitionOptions
        settings = CompetitionOptions(standard_drop_m=.51, right_mount_drop_m=.37,
                                      task_counts=(1, 2, 1)).as_dict()
        self.listener._run_mission(ClockEvent(self), 7, 0., options=settings)
        self.assertEqual(self.mission.call_args.kwargs['run_options'], settings)
        self.mission.reset_mock()
        self.run_worker()
        self.assertNotIn('run_options', self.mission.call_args.kwargs)

    def test_radio_disconnect_or_callback_error_does_not_cancel_the_running_task(self):
        with patch.object(self.listener, '_cancel_current') as cancel:
            self.listener._on_disconnected(OSError('link down'))
            self.listener._on_callback_error(ValueError('bad frame'))
        cancel.assert_not_called()

    def test_run_captures_stop_signature_before_worker_and_preserves_snapshot(self):
        self.signature.side_effect = self.real_signature
        with WorkspaceTempDir() as directory:
            marker = Path(directory) / 'STOP'
            marker.write_text('previous round')
            baseline = self.real_signature(marker)
            with patch.object(boot, 'Path', return_value=marker) as path, \
                 patch.object(boot.threading, 'Thread') as thread:
                self.assertEqual(self.listener._start_mission(), (AckStatus.ACCEPTED, AckReason.NONE))
                path.assert_called_once_with('logs/competition/STOP')
                thread.return_value.start.assert_called_once_with()
                args = thread.call_args.kwargs['args']
                self.assertEqual(args[3], baseline)
            marker.write_text('updated during countdown')
            self.assertNotEqual(self.real_signature(marker), baseline)
            self.listener._run_mission(ClockEvent(self), *args[1:])
            self.assertEqual(self.mission.call_args.kwargs['stop_file_baseline'], baseline)
            self.assertEqual(marker.read_text(), 'updated during countdown')

    def test_run_stat_error_does_not_reject_or_cancel_start(self):
        self.signature.side_effect = OSError('denied')
        frame = Frame(1, NodeId.GROUND, NodeId.CAR, MessageKind.COMMAND, 0,
                      123, 1, encode_command(CommandPayload(CommandId.CAR_START_MISSION)))
        with patch.object(self.listener, '_run_mission') as worker:
            self.listener._on_bytes(pack_frame(frame))
            self.listener._worker.join(1)
        ack = decode_ack(unpack_frame(self.driver.write.call_args.args[0]).payload)
        self.assertEqual((ack.status, ack.reason), (AckStatus.ACCEPTED, AckReason.NONE))
        self.assertIsNone(worker.call_args.args[3])

    def setUp(self):
        self.now=0.
        self.events=[]
        self.stack=ExitStack(); self.addCleanup(self.stack.close)
        # TemporaryDirectory defaults to a sandbox path that is not writable
        # here, so every temp dir is created inside the repository instead.
        self.tmp_root = Path(__file__).resolve().parents[1] / 'logs' / 'tmp'
        self.tmp_root.mkdir(parents=True, exist_ok=True)
        self.tmpdir = temp_dir('boot')
        self.addCleanup(cleanup_temp_dir, self.tmpdir)
        from main_robocup import stop_marker_signature
        self.real_signature = stop_marker_signature
        self.signature = self.stack.enter_context(patch('main_robocup.stop_marker_signature', return_value=None))
        self.open=self.stack.enter_context(patch.object(boot,'open_payload',side_effect=lambda:self.events.append(('open',self.now))))
        self.close=self.stack.enter_context(patch.object(boot,'close_payload',side_effect=lambda:self.events.append(('close',self.now))))
        self.alarm=Mock()
        self.alarm.on.side_effect=lambda:self.events.append(('buzz_on',self.now))
        self.alarm.off.side_effect=lambda:self.events.append(('buzz_off',self.now))
        self.driver=Mock()
        self.mission=Mock(return_value=0)
        self.listener=boot.CompetitionBoot(driver_factory=Mock(return_value=self.driver),
            mission_main=self.mission,alarm=self.alarm,clock=lambda:self.now)
        # The competition chain must never restart the car SLAM sidecar.
        self.sidecar = self.stack.enter_context(
            patch.object(boot, 'restart_car_slam_sidecar'))

    def tearDown(self):
        self.sidecar.assert_not_called()

    def run_worker(self,cancel_at=None):
        self.listener._run_mission(ClockEvent(self,cancel_at),7,0.)

    def test_start_opens_and_silences_before_listening(self):
        self.listener.start()
        self.alarm.initialize.assert_called_once_with(active=False)
        self.assertEqual(self.events,[('buzz_off',0.),('open',0.)])
        self.driver.start.assert_called_once()

    def test_mission_starts_immediately_after_close_without_beep(self):
        def mission(*args,**kwargs):
            self.events.append(('mission',self.now))
            self.assertNotIn('send_drone_task', kwargs)
            return 0
        self.mission.side_effect=mission
        self.run_worker()
        self.assertEqual([name for name, _ in self.events], ['close', 'mission', 'open'])
        self.assertEqual(dict(self.events)['mission'], 0.)

    def test_false_payload_close_is_reported_but_does_not_abort_mission(self):
        self.close.side_effect=None
        self.close.return_value=False
        self.run_worker()
        self.mission.assert_called_once()
        self.assertEqual(self.events[-1][0],'open')

    def test_attempt_false_fails_but_none_is_success(self):
        with self.assertLogs(boot.LOG, level='ERROR'):
            self.assertFalse(self.listener._attempt('PWM', lambda: False))
        self.assertTrue(self.listener._attempt('alarm', lambda: None))

    def test_invalid_and_non_ground_frames_cannot_start_a_mission(self):
        for source,body in ((NodeId.DRONE,b''),(NodeId.GROUND,b'unexpected')):
            raw=pack_frame(Frame(1,source,NodeId.CAR,MessageKind.COMMAND,0,10,1,
                encode_command(CommandPayload(CommandId.CAR_START_MISSION,command_body=body))))
            self.listener._on_bytes(raw)
        self.assertIsNone(self.listener._worker)
        self.close.assert_not_called()

    def test_duplicate_run_cache_and_busy_do_not_create_workers(self):
        blocker=Mock(); blocker.is_alive.return_value=True
        self.listener._worker=blocker
        raw=pack_frame(Frame(1,NodeId.GROUND,NodeId.CAR,MessageKind.COMMAND,0,10,1,
            encode_command(CommandPayload(CommandId.CAR_START_MISSION))))
        self.listener._on_bytes(raw); self.listener._on_bytes(raw)
        replies=[call.args[0] for call in self.driver.write.call_args_list]
        self.assertEqual(replies[0],replies[1])
        self.assertEqual(decode_ack(unpack_frame(replies[0]).payload).status,AckStatus.REJECTED)
        self.close.assert_not_called()

    def test_run_ack_returns_while_one_worker_is_blocked_and_stop_still_arrives(self):
        entered,release=threading.Event(),threading.Event()
        def work(*args):
            entered.set()
            release.wait(1)
        raw=pack_frame(Frame(1,NodeId.GROUND,NodeId.CAR,MessageKind.COMMAND,0,10,1,
            encode_command(CommandPayload(CommandId.CAR_START_MISSION))))
        with patch.object(self.listener,'_run_mission',side_effect=work) as worker:
            try:
                self.listener._on_bytes(raw)
                self.assertTrue(entered.wait(.5))
                reply=self.driver.write.call_args.args[0]
                self.assertEqual(decode_ack(unpack_frame(reply).payload).status,AckStatus.ACCEPTED)
                stop=self.listener._stop_event
                self.listener._on_bytes(raw)
                self.assertIs(self.listener._stop_event,stop)
                worker.assert_called_once()
                args = worker.call_args.args
                self.assertGreater(args[1], 0)          # run_id
                self.assertIsInstance(args[2], float)   # t0
                self.assertIsInstance(args[0], object)  # stop event
                self.listener._cancel_current()
                self.assertTrue(stop.is_set())
                self.open.assert_called_once()
            finally:
                release.set()
                self.listener._worker.join(1)

    def test_stop_on_a_wall_runtime_calls_the_stop_latch(self):
        # The competition runtime has no ``motion`` attribute at all; touching it
        # would raise before the latch was ever set.
        runtime = Mock(spec=['drive', 'mission', 'request_stop'])
        self.listener._runtime = runtime
        self.listener._stop_event = threading.Event()
        self.listener._cancel_current()
        runtime.request_stop.assert_called_once_with('remote STOP')
        self.assertTrue(self.listener._stop_event.is_set())

    def test_stop_on_a_legacy_fused_runtime_keeps_its_own_branch(self):
        # A diagnostic tool built on the old runtime must not be broken by the
        # wall-runtime change.
        runtime = Mock(spec=['drive', 'motion', 'mission'])
        self.listener._runtime = runtime
        self.listener._stop_event = threading.Event()
        self.listener._cancel_current()
        runtime.drive.stop.assert_called_once()
        runtime.motion.stop.assert_called_once()
        runtime.mission.request_safe_stop.assert_called_once_with('remote STOP')

    def test_car_only_stop_disconnect_and_failure_opens_payload_without_drone(self):
        self.listener._car_only_test = True
        self.run_worker(cancel_at=18.)
        self.mission.assert_not_called()
        self.now = 0.
        self.mission.side_effect = RuntimeError('test failure')
        self.run_worker()
        self.listener._on_disconnected()
        self.listener._on_callback_error(RuntimeError('radio failed'))
        self.open.assert_called()
        self.close.assert_called()
        self.driver.write.assert_not_called()


class SidecarTests(unittest.TestCase):
    def test_requires_a_new_active_instance(self):
        before=Mock(stdout='ActiveState=active\nMainPID=1\nInvocationID=old')
        after=Mock(stdout='ActiveState=active\nMainPID=2\nInvocationID=new')
        process=Mock(returncode=0); process.poll.return_value=0
        with patch.object(boot.subprocess,'run',side_effect=[before,after]), \
             patch.object(boot.subprocess,'Popen',return_value=process):
            self.assertEqual(boot.restart_car_slam_sidecar(threading.Event())['InvocationID'],'new')
            self.assertEqual(boot.subprocess.Popen.call_args.args[0],
                ['sudo', '-n', boot.SYSTEMCTL, 'restart', boot.SLAM_SERVICE])
        with patch.object(boot.subprocess,'run',return_value=before), \
             patch.object(boot.subprocess,'Popen',return_value=process):
            with self.assertRaisesRegex(RuntimeError,'new SLAM'):
                boot.restart_car_slam_sidecar(threading.Event())

    def test_failed_restart_command_is_rejected(self):
        before=Mock(stdout='ActiveState=active\nMainPID=1\nInvocationID=old')
        process=Mock(returncode=1); process.poll.return_value=1
        with patch.object(boot.subprocess,'run',return_value=before), \
             patch.object(boot.subprocess,'Popen',return_value=process):
            with self.assertRaisesRegex(RuntimeError,'restart failed'):
                boot.restart_car_slam_sidecar(threading.Event())

    def test_missing_slam_unit_cannot_be_confirmed(self):
        missing=Mock(stdout='')
        process=Mock(returncode=1); process.poll.return_value=1
        with patch.object(boot.subprocess,'run',return_value=missing), \
             patch.object(boot.subprocess,'Popen',return_value=process):
            with self.assertRaisesRegex(RuntimeError,'restart failed'):
                boot.restart_car_slam_sidecar(threading.Event())
