"""Fake clock, radio and outputs: never runs systemctl or opens hardware."""
from contextlib import ExitStack
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
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
        self.sender.send_task_and_start.assert_not_called()

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
                self.assertTrue(args.relative_slam)

    def test_missing_explicit_real_profile_fails_without_falling_back_to_example(self):
        from main_robocup import main
        with tempfile.TemporaryDirectory() as directory:
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
        self.assertFalse(self.sender.mock_calls)

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
             patch.object(boot, 'CompetitionBoot') as listener:
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
        self.assertFalse(self.sender.mock_calls)

    def test_formal_listener_refuses_local_control(self):
        with self.assertRaises(ValueError):
            self.listener.local_command('start')
        self.mission.assert_not_called()

    def test_run_captures_stop_signature_before_worker_and_preserves_snapshot(self):
        self.signature.side_effect = self.real_signature
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / 'STOP'
            marker.write_text('previous round')
            baseline = self.real_signature(marker)
            with patch.object(boot, 'Path', return_value=marker) as path, \
                 patch.object(boot.threading, 'Thread') as thread, \
                 patch.object(boot, 'FleetTaskSender', return_value=self.sender):
                self.assertEqual(self.listener._start_mission(), (AckStatus.ACCEPTED, AckReason.NONE))
                path.assert_called_once_with('logs/competition/STOP')
                thread.return_value.start.assert_called_once_with()
                args = thread.call_args.kwargs['args']
                self.assertEqual(args[4], baseline)
            marker.write_text('updated during countdown')
            self.assertNotEqual(self.real_signature(marker), baseline)
            self.listener._run_mission(ClockEvent(self), *args[1:])
            self.assertEqual(self.mission.call_args.kwargs['stop_file_baseline'], baseline)
            self.assertEqual(marker.read_text(), 'updated during countdown')

    def test_run_stat_error_returns_failed_ack_without_starting_worker(self):
        self.signature.side_effect = PermissionError('STOP stat denied')
        raw = pack_frame(Frame(1, NodeId.GROUND, NodeId.CAR, MessageKind.COMMAND,
            0, 10, 1, encode_command(CommandPayload(CommandId.CAR_START_MISSION))))
        with patch.object(boot.threading, 'Thread') as thread, \
             self.assertLogs(boot.LOG, level='ERROR'):
            self.listener._on_bytes(raw)
        ack = decode_ack(unpack_frame(self.driver.write.call_args.args[0]).payload)
        self.assertEqual((ack.status, ack.reason), (AckStatus.FAILED, AckReason.INTERNAL_ERROR))
        thread.assert_not_called()
        self.assertIsNone(self.listener._worker)
        self.assertIsNone(self.listener._stop_event)
        self.close.assert_not_called()
        self.mission.assert_not_called()

    def setUp(self):
        self.now=0.
        self.events=[]
        self.stack=ExitStack(); self.addCleanup(self.stack.close)
        from main_robocup import stop_marker_signature
        self.real_signature = stop_marker_signature
        self.signature = self.stack.enter_context(patch('main_robocup.stop_marker_signature', return_value=None))
        self.open=self.stack.enter_context(patch.object(boot,'open_payload',side_effect=lambda:self.events.append(('open',self.now))))
        self.close=self.stack.enter_context(patch.object(boot,'close_payload',side_effect=lambda:self.events.append(('close',self.now))))
        self.remap=self.stack.enter_context(patch.object(boot,'restart_car_slam_sidecar',side_effect=lambda stop:self.events.append(('car_map',self.now))))
        self.alarm=Mock()
        self.alarm.on.side_effect=lambda:self.events.append(('buzz_on',self.now))
        self.alarm.off.side_effect=lambda:self.events.append(('buzz_off',self.now))
        self.driver=Mock()
        self.mission=Mock(return_value=0)
        self.listener=boot.CompetitionBoot(driver_factory=Mock(return_value=self.driver),
            mission_main=self.mission,alarm=self.alarm,clock=lambda:self.now)
        self.sender=Mock()
        self.sender.close_drone_payload.return_value=True
        self.sender.begin_drone_remap.return_value=True
        self.sender.wait_drone_remap.return_value=True

    def run_worker(self,cancel_at=None):
        self.listener._run_mission(ClockEvent(self,cancel_at),self.sender,7,0.)

    def test_start_opens_and_silences_before_listening(self):
        self.listener.start()
        self.alarm.initialize.assert_called_once_with(active=False)
        self.assertEqual(self.events,[('buzz_off',0.),('open',0.)])
        self.driver.start.assert_called_once()

    def test_countdown_then_maps_then_full_beep_and_ocr_gate(self):
        def mission(*args,**kwargs):
            self.events.append(('mission',self.now))
            self.assertTrue(kwargs['send_drone_task']('counts'))
            return 0
        self.mission.side_effect=mission
        self.run_worker()
        self.assertIn(('buzz_on',20.),self.events)
        self.assertIn(('buzz_off',25.),self.events)
        self.assertIn(('car_map',20.),self.events)
        self.assertIn(('mission',25.),self.events)
        self.sender.wait_drone_remap.assert_called_once()
        self.sender.send_task_and_start.assert_called_once()
        call = self.sender.send_task_and_start.call_args
        self.assertEqual(call.args, ('counts',))
        self.assertTrue(call.kwargs['start_drone'])
        self.assertTrue(callable(call.kwargs['on_task_sent']))
        self.assertTrue(callable(call.kwargs['on_task_ack']))
        self.assertEqual(self.events[-1][0], 'open')
        self.assertAlmostEqual(self.events[-1][1], 25. + boot.PROMPT_BEEP_S)

    def test_full_twenty_seconds_start_at_close_ack_even_when_ack_is_late(self):
        for ack_at in (0., 18., 25.):
            with self.subTest(ack_at=ack_at):
                self.now = 0.
                self.events.clear()
                self.sender.reset_mock()
                self.mission.reset_mock()
                self.sender.close_drone_payload.side_effect = (
                    lambda run: setattr(self, 'now', ack_at) or True)
                self.sender.begin_drone_remap.side_effect = (
                    lambda run: self.events.append(('drone_map', self.now)) or True)
                self.mission.side_effect = lambda *a, **k: self.events.append(('mission', self.now)) or 0
                self.run_worker()
                self.assertEqual(self.events[:5], [('close', 0.), ('drone_map', ack_at + 20.),
                    ('car_map', ack_at + 20.), ('buzz_on', ack_at + 20.), ('buzz_off', ack_at + 25.)])
                self.assertIn(('mission', ack_at + 25.), self.events)
                self.sender.begin_drone_remap.assert_called_once_with(7)

    def test_both_maps_and_drone_completed_ack_precede_five_second_beep_and_ocr(self):
        self.sender.close_drone_payload.side_effect = lambda run: setattr(self, 'now', 7.) or True
        self.sender.begin_drone_remap.side_effect = lambda run: self.events.append(('drone_map', self.now)) or True
        def car_map(stop):
            self.events.append(('car_map', self.now))
            self.now += 2.
        def drone_ready(*args):
            self.now += 5.
            self.events.append(('drone_ready', self.now))
            return True
        self.remap.side_effect = car_map
        self.sender.wait_drone_remap.side_effect = drone_ready
        self.mission.side_effect = lambda *a, **k: self.events.append(('ocr', self.now)) or 0
        self.run_worker()
        self.assertEqual(self.events[:7], [('close', 0.), ('drone_map', 27.),
            ('car_map', 27.), ('drone_ready', 34.), ('buzz_on', 34.),
            ('buzz_off', 39.), ('ocr', 39.)])

    def test_ocr_cue_then_two_sent_pulses_then_one_ack_pulse_before_start(self):
        def send(counts, *, start_drone, on_task_sent, on_task_ack):
            self.assertEqual(counts, 'counts')
            self.assertTrue(start_drone)
            self.events.append(('task_write', self.now))
            if not on_task_sent():
                return False
            self.now += 3.
            self.events.append(('task_ack', self.now))
            if not on_task_ack():
                return False
            self.events.append(('drone_start', self.now))
            return True
        def mission(*args, **kwargs):
            self.events.append(('ocr', self.now))
            self.now += 1.
            self.assertTrue(kwargs['send_drone_task']('counts'))
            self.events.append(('car_patrol', self.now))
            return 0
        self.sender.send_task_and_start.side_effect = send
        self.mission.side_effect = mission
        self.run_worker()
        names = [name for name, _ in self.events]
        self.assertEqual(names[:16], ['close', 'car_map', 'buzz_on', 'buzz_off', 'ocr',
            'buzz_on', 'buzz_off', 'task_write', 'buzz_on', 'buzz_off', 'buzz_on',
            'buzz_off', 'task_ack', 'buzz_on', 'buzz_off', 'drone_start'])
        buzz_on = [at for name, at in self.events if name == 'buzz_on']
        buzz_off = [at for name, at in self.events if name == 'buzz_off']
        self.assertEqual(len(buzz_on), 5)
        for actual, expected in zip(buzz_on, (20., 26., 26.15, 26.45, 29.6)):
            self.assertAlmostEqual(actual, expected)
        for started, ended, duration in zip(buzz_on, buzz_off, (5., .15, .15, .15, .15)):
            self.assertAlmostEqual(ended - started, duration)
        self.assertAlmostEqual(dict(self.events)['drone_start'], 29.75)
        self.assertEqual(dict(self.events)['car_patrol'], dict(self.events)['drone_start'])

    def test_stop_during_five_second_map_beep_prevents_ocr(self):
        self.run_worker(22.)
        self.mission.assert_not_called()
        self.assertIn(('buzz_on', 20.), self.events)
        self.assertIn(('buzz_off', 22.), self.events)
        self.assertEqual(self.events[-1], ('open', 22.))

    def test_stop_during_sent_double_beep_prevents_ack_cue_and_start(self):
        started = Mock()
        def send(counts, *, on_task_sent, on_task_ack, **kwargs):
            if not on_task_sent():
                return False
            on_task_ack()
            started()
            return True
        self.sender.send_task_and_start.side_effect = send
        self.mission.side_effect = lambda *a, **k: 0 if k['send_drone_task']('counts') else 1
        self.run_worker(25.4)
        started.assert_not_called()
        self.assertEqual(len([x for x in self.events if x[0] == 'buzz_on']), 3)
        self.assertEqual(self.events[-1], ('open', 25.4))

    def test_stop_during_ack_pulse_prevents_task_start(self):
        stop, started = ClockEvent(self), Mock()
        def send(counts, *, on_task_sent, on_task_ack, **kwargs):
            self.assertTrue(on_task_sent())
            stop.cancel_at = self.now + .05
            if not on_task_ack():
                return False
            started()
            return True
        self.sender.send_task_and_start.side_effect = send
        self.mission.side_effect = lambda *a, **k: 0 if k['send_drone_task']('counts') else 1
        self.listener._run_mission(stop, self.sender, 7, 0.)
        started.assert_not_called()
        self.assertTrue(stop.is_set())
        self.assertEqual(self.events[-1][0], 'open')

    def test_remap_rejection_timeout_or_stop_never_beeps_or_enters_ocr(self):
        for mode in ('rejected', 'timeout', 'stop'):
            with self.subTest(mode=mode):
                self.now = 0.
                self.alarm.reset_mock()
                self.mission.reset_mock()
                stop = ClockEvent(self)
                if mode == 'rejected':
                    self.sender.begin_drone_remap.return_value = False
                    self.sender.wait_drone_remap.side_effect = None
                elif mode == 'timeout':
                    self.sender.begin_drone_remap.return_value = True
                    self.sender.wait_drone_remap.side_effect = TimeoutError('new map ACK')
                else:
                    self.sender.wait_drone_remap.side_effect = lambda *a: stop.set() or False
                self.listener._run_mission(stop, self.sender, 7, 0.)
                self.alarm.on.assert_not_called()
                self.mission.assert_not_called()
                self.alarm.off.assert_called()

    def test_stop_after_map_beep_never_enters_ocr(self):
        stop = ClockEvent(self)
        self.alarm.off.side_effect = stop.set
        self.listener._run_mission(stop, self.sender, 7, 0.)
        self.alarm.off.assert_called()
        self.sender.begin_drone_remap.assert_called_once()
        self.remap.assert_called_once()
        self.mission.assert_not_called()

    def test_stop_while_waiting_for_payload_lock_prevents_beep(self):
        stop = ClockEvent(self)
        lock = Mock()
        def acquire():
            if lock.__enter__.call_count == 2:
                stop.set()
        lock.__enter__ = Mock(side_effect=acquire)
        lock.__exit__ = Mock(return_value=False)
        self.listener._payload_action_lock = lock
        self.listener._run_mission(stop, self.sender, 7, 0.)
        self.alarm.on.assert_not_called()
        self.alarm.off.assert_called()
        self.sender.begin_drone_remap.assert_called_once()
        self.remap.assert_called_once()
        self.mission.assert_not_called()

    def test_remote_close_rejection_or_timeout_never_beeps_or_launches(self):
        for outcome in (False, TimeoutError('CLOSE ACK timed out')):
            with self.subTest(outcome=outcome):
                self.sender.close_drone_payload.side_effect = (
                    outcome if isinstance(outcome, Exception) else lambda run: False)
                self.run_worker()
                self.alarm.on.assert_not_called()
                self.alarm.off.assert_called()
                self.sender.begin_drone_remap.assert_not_called()
                self.remap.assert_not_called()
                self.mission.assert_not_called()

    def test_stop_at_four_or_eighteen_never_starts_maps_or_mission(self):
        for at in (4.,18.):
            with self.subTest(at=at):
                self.now=0.; self.events.clear()
                self.run_worker(at)
                self.remap.assert_not_called()
                self.mission.assert_not_called()
                self.assertEqual(self.events[-1],('open',at))
                self.assertIn(('buzz_off', at), self.events)
                self.sender.begin_drone_remap.assert_not_called()

    def test_missing_pwm_close_and_failed_remote_close_never_launch(self):
        self.close.side_effect=RuntimeError('PWM None')
        self.run_worker()
        self.mission.assert_not_called(); self.remap.assert_not_called()
        self.assertEqual(self.events[-1][0],'open')
        self.close.side_effect=None; self.sender.close_drone_payload.return_value=False
        self.run_worker()
        self.mission.assert_not_called(); self.remap.assert_not_called()

    def test_false_payload_close_prevents_mission(self):
        self.close.side_effect=None
        self.close.return_value=False
        self.run_worker()
        self.remap.assert_not_called()
        self.mission.assert_not_called()
        self.assertEqual(self.events[-1][0],'open')

    def test_attempt_false_fails_but_none_is_success(self):
        with self.assertLogs(boot.LOG, level='ERROR'):
            self.assertFalse(self.listener._attempt('PWM', lambda: False))
        self.assertTrue(self.listener._attempt('alarm', lambda: None))

    def test_idle_stop_still_opens_and_forwards(self):
        self.listener._cancel_current(); self.listener._forward_stop()
        self.open.assert_called_once(); self.alarm.off.assert_called_once()
        self.driver.write.assert_called_once()

    def test_success_does_not_stop_the_drone_before_its_own_mission_finishes(self):
        self.run_worker()
        self.driver.write.assert_not_called()

    def test_cancelled_sender_cannot_write_after_stop(self):
        self.listener._stop_event=threading.Event()
        self.listener._stop_event.set()
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            self.listener.write(b'task')
        self.driver.write.assert_not_called()

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
                self.assertGreater(worker.call_args.args[2],0)
                self.listener._cancel_current()
                self.assertTrue(stop.is_set())
                self.open.assert_called_once()
            finally:
                release.set()
                self.listener._worker.join(1)

    def test_stop_during_sidecar_restart_never_enters_mission(self):
        def restart(stop):
            stop.set()
        self.remap.side_effect=restart
        self.run_worker()
        self.mission.assert_not_called()
        self.assertEqual(self.events[-1],('open',20.))

    def test_active_runtime_stop_preserves_base_stop_and_safe_stop(self):
        runtime=Mock()
        self.listener._runtime=runtime
        self.listener._stop_event=threading.Event()
        self.listener._cancel_current()
        self.assertTrue(self.listener._stop_event.is_set())
        runtime.drive.stop.assert_called_once()
        runtime.motion.stop.assert_called_once()
        runtime.mission.request_safe_stop.assert_called_once_with('remote STOP')

    def test_remap_failure_or_not_ready_never_hands_off(self):
        self.remap.side_effect=RuntimeError('old instance')
        self.run_worker(); self.mission.assert_not_called()
        self.remap.side_effect=None; self.now=0.
        self.sender.wait_drone_remap.return_value=False
        def mission(*args,**kwargs):
            self.assertFalse(kwargs['send_drone_task']('counts')); return 1
        self.mission.side_effect=mission
        self.run_worker()
        self.sender.send_task_and_start.assert_not_called()
        self.mission.assert_not_called()
        self.alarm.on.assert_not_called()


    def test_car_only_full_flow_keeps_timing_slam_and_disables_drone_with_real_payload(self):
        self.listener._car_only_test = True
        self.listener.start()
        self.run_worker()
        self.assertIn(('buzz_on', 20.), self.events)
        self.assertIn(('buzz_off', 25.), self.events)
        self.remap.assert_called_once()
        self.mission.assert_called_once()
        self.assertTrue(self.mission.call_args.kwargs['car_only_test'])
        self.assertIsNone(self.mission.call_args.kwargs['send_drone_task'])
        self.listener.close()
        self.open.assert_called()
        self.close.assert_called()
        self.assertFalse(self.sender.mock_calls)
        self.driver.write.assert_not_called()

    def test_car_only_never_constructs_sender_and_only_acks_ground_commands(self):
        self.listener._car_only_test = True
        with patch.object(boot, 'FleetTaskSender', side_effect=AssertionError('drone sender')), \
             patch.object(self.listener, '_run_mission') as run:
            for seq, command in enumerate((CommandId.CAR_START_MISSION,
                                           CommandId.TARGETED_STOP, CommandId.PING), 1):
                raw = pack_frame(Frame(1, NodeId.GROUND, NodeId.CAR, MessageKind.COMMAND,
                    0, 10, seq, encode_command(CommandPayload(command))))
                self.listener._on_bytes(raw)
                if seq == 1:
                    self.listener._worker.join(1)
            run.assert_called_once()
            self.assertIsNone(run.call_args.args[1])
        self.assertEqual(self.driver.write.call_count, 3)
        for call in self.driver.write.call_args_list:
            frame = unpack_frame(call.args[0])
            self.assertEqual((frame.dst, frame.kind), (NodeId.GROUND, MessageKind.ACK))
        self.open.assert_called()
        self.close.assert_not_called()

    def test_car_only_stop_disconnect_and_failure_opens_payload_without_drone(self):
        self.listener._car_only_test = True
        self.run_worker(cancel_at=18.)
        self.mission.assert_not_called()
        self.remap.assert_not_called()
        self.now = 0.
        self.mission.side_effect = RuntimeError('test failure')
        self.run_worker()
        self.listener._on_disconnected()
        self.listener._on_callback_error(RuntimeError('radio failed'))
        self.open.assert_called()
        self.close.assert_called()
        self.assertFalse(self.sender.mock_calls)
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
