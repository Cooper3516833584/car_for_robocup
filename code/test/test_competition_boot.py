"""Fake clock, radio and outputs: never runs systemctl or opens hardware."""
from contextlib import ExitStack
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import competition_boot as boot
from components.fleet_models import (AckStatus, CommandId, CommandPayload, Frame,
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
    def setUp(self):
        self.now=0.
        self.events=[]
        self.stack=ExitStack(); self.addCleanup(self.stack.close)
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

    def test_absolute_timing_and_ocr_map_gate(self):
        def mission(*args,**kwargs):
            self.events.append(('mission',self.now))
            self.assertTrue(kwargs['send_drone_task']('counts'))
            return 0
        self.mission.side_effect=mission
        self.run_worker()
        self.assertIn(('buzz_on',15.),self.events)
        self.assertIn(('buzz_off',20.),self.events)
        self.assertIn(('car_map',20.),self.events)
        self.assertIn(('mission',20.),self.events)
        self.sender.wait_drone_remap.assert_called_once()
        self.sender.send_task_and_start.assert_called_once_with('counts',start_drone=True)
        self.assertEqual(self.events[-1],('open',20.))

    def test_slow_close_does_not_shift_deadline_or_extend_beep(self):
        self.sender.close_drone_payload.side_effect=lambda run: setattr(self,'now',18.) or True
        self.run_worker()
        self.assertIn(('buzz_on',18.),self.events)
        self.assertIn(('mission',20.),[('mission',self.mission.call_args.kwargs['stop_event'].owner.now)])
        self.assertIn(('car_map',20.),self.events)

    def test_stop_at_four_or_eighteen_never_starts_maps_or_mission(self):
        for at in (4.,18.):
            with self.subTest(at=at):
                self.now=0.; self.events.clear()
                self.run_worker(at)
                self.remap.assert_not_called()
                self.mission.assert_not_called()
                self.assertEqual(self.events[-1],('open',at))

    def test_missing_pwm_close_and_failed_remote_close_never_launch(self):
        self.close.side_effect=RuntimeError('PWM None')
        self.run_worker()
        self.mission.assert_not_called(); self.remap.assert_not_called()
        self.assertEqual(self.events[-1][0],'open')
        self.close.side_effect=None; self.sender.close_drone_payload.return_value=False
        self.run_worker()
        self.mission.assert_not_called(); self.remap.assert_not_called()

    def test_unconfigured_payload_skips_close_but_runs_the_mission(self):
        self.close.side_effect=None
        self.close.return_value=False
        self.run_worker()
        self.remap.assert_called_once()
        self.mission.assert_called_once()
        self.assertEqual(self.events[-1][0],'open')

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
