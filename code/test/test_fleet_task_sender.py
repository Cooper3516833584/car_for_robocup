"""Allocation handoff regressions; fake transport only."""
from pathlib import Path
import sys
import threading
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.drop_task_protocol import (DropTask, decode_drop_task, decode_drop_start,
                                            encode_drop_task, encode_drop_start)
from components.fleet_task_sender import FleetTaskSender, send_task_and_start
from components.fleet_models import AckPayload, AckStatus, CommandId, Frame, MessageKind, NodeId
from components.fleet_protocol import decode_command, encode_ack, pack_frame, unpack_frame


class HandoffTests(unittest.TestCase):
    counts = SimpleNamespace(red=0, blue=0, green=1)

    def test_v2_single_quota_roundtrip_and_old_rule_rejection(self):
        for counts in ((1,0,0),(0,1,0),(0,0,1)):
            task = DropTask(42, *counts)
            self.assertEqual(decode_drop_task(encode_drop_task(task)), task)
        self.assertEqual(encode_drop_task(task)[0], 2)
        self.assertEqual(decode_drop_start(encode_drop_start(42)), 42)
        for body in (struct.pack('<BI3B',1,42,1,0,0), struct.pack('<BI3B',2,42,2,0,1),
                     struct.pack('<BI3B',2,42,0,0,0)):
            with self.assertRaises(ValueError): decode_drop_task(body)
        with self.assertRaises(ValueError): decode_drop_start(struct.pack('<BI',1,42))

    def make_sender(self, send, retries=1, stop_event=None):
        driver = Mock()
        driver.write.side_effect = send
        sender = FleetTaskSender(driver, response_timeout_s=.01, retries=retries,
                                 session=123, stop_event=stop_event)
        return sender

    @staticmethod
    def ack(sender, raw, status=AckStatus.COMPLETED, *, src=NodeId.DRONE, session=None):
        request = unpack_frame(raw)
        command = decode_command(request.payload)
        ack = AckPayload(request.session if session is None else session, request.seq, command.command_id, status)
        sender.feed_bytes(pack_frame(Frame(1, src, NodeId.CAR, MessageKind.ACK, 0, 777, request.seq, encode_ack(ack))))

    def test_actual_counts_then_matching_start_only_after_completed_ack(self):
        sent = []
        def send(raw):
            sent.append(raw)
            self.ack(sender, raw)
        sender = self.make_sender(send)
        self.assertTrue(sender.send_task_and_start(self.counts, task_id=42))
        tasks = [decode_command(unpack_frame(raw).payload) for raw in sent]
        self.assertEqual(decode_drop_task(tasks[0].command_body), DropTask(42, 0, 0, 1))
        self.assertEqual(decode_drop_start(tasks[1].command_body), 42)
        self.assertEqual([t.command_id for t in tasks], [CommandId.DRONE_SET_DROP_TASK, CommandId.DRONE_START_DROP_TASK])
        self.assertTrue(all(unpack_frame(raw).dst == NodeId.DRONE and unpack_frame(raw).src == NodeId.CAR for raw in sent))

    def test_task_sent_and_completed_ack_cues_precede_start_even_with_immediate_ack(self):
        events = []
        def send(raw):
            command = decode_command(unpack_frame(raw).payload)
            events.append('task_write' if command.command_id == CommandId.DRONE_SET_DROP_TASK else 'start_write')
            self.ack(sender, raw)
        sender = self.make_sender(send)
        self.assertTrue(sender.send_task_and_start(self.counts, task_id=42,
            on_task_sent=lambda: events.append('sent_cue'),
            on_task_ack=lambda: events.append('ack_cue')))
        self.assertEqual(events, ['task_write', 'sent_cue', 'ack_cue', 'start_write'])

    def test_retry_cues_once_and_only_completed_matching_ack_confirms(self):
        sent, cues = [], []
        def send(raw):
            sent.append(raw)
            if len(sent) == 1:
                self.ack(sender, raw, AckStatus.RECEIVED)
                self.ack(sender, raw, AckStatus.ACCEPTED)
                self.ack(sender, raw, src=NodeId.GROUND)
            else:
                self.ack(sender, raw)
        sender = self.make_sender(send)
        self.assertTrue(sender.send_task_and_start(self.counts, task_id=42,
            on_task_sent=lambda: cues.append('sent'), on_task_ack=lambda: cues.append('ack')))
        self.assertEqual(sent[0], sent[1])
        self.assertEqual(cues, ['sent', 'ack'])
        self.assertEqual(len(sent), 3)

    def test_rejected_or_missing_task_ack_never_plays_confirmation(self):
        for status in (None, AckStatus.RECEIVED, AckStatus.ACCEPTED, AckStatus.REJECTED, AckStatus.FAILED):
            with self.subTest(status=status):
                sent_cue, ack_cue = Mock(), Mock()
                def send(raw):
                    if status is not None:
                        self.ack(sender, raw, status)
                sender = self.make_sender(send, retries=0)
                self.assertFalse(sender.send_task_and_start(self.counts,
                    on_task_sent=sent_cue, on_task_ack=ack_cue))
                sent_cue.assert_called_once_with()
                ack_cue.assert_not_called()

    def test_cue_failure_or_stop_prevents_start(self):
        for cue_name in ('on_task_sent', 'on_task_ack'):
            for cancel in (False, True):
                with self.subTest(cue=cue_name, cancel=cancel):
                    stop, sent = threading.Event(), []
                    def send(raw):
                        sent.append(raw)
                        self.ack(sender, raw)
                    def cue():
                        if cancel:
                            stop.set()
                            return True
                        return False
                    sender = self.make_sender(send, stop_event=stop)
                    self.assertFalse(sender.send_task_and_start(self.counts, **{cue_name:cue}))
                    self.assertEqual(len(sent), 1)

    def test_cue_exception_never_sends_start(self):
        sent = []
        def send(raw):
            sent.append(raw)
            self.ack(sender, raw)
        sender = self.make_sender(send)
        with self.assertRaisesRegex(RuntimeError, 'buzzer failed'):
            sender.send_task_and_start(self.counts, on_task_ack=Mock(side_effect=RuntimeError('buzzer failed')))
        self.assertEqual(len(sent), 1)

    def test_rx_callback_can_deliver_ack_while_sent_cue_is_running(self):
        sent = []
        def cue():
            worker = threading.Thread(target=lambda: self.ack(sender, sent[-1]))
            worker.start()
            worker.join(.5)
            self.assertFalse(worker.is_alive(), 'sent cue must not hold the ACK receive lock')
        def send(raw):
            sent.append(raw)
        sender = self.make_sender(send)
        self.assertTrue(sender.send_task_and_start(self.counts, start_drone=False, on_task_sent=cue))

    def test_no_start_after_missing_received_only_or_rejected_task_ack(self):
        for status in (None, AckStatus.RECEIVED, AckStatus.ACCEPTED, AckStatus.REJECTED, AckStatus.FAILED):
            with self.subTest(status=status):
                sent = []
                def send(raw):
                    sent.append(raw)
                    if status is not None:
                        self.ack(sender, raw, status)
                sender = self.make_sender(send)
                self.assertFalse(sender.send_task_and_start(self.counts, task_id=42))
                self.assertTrue(all(decode_command(unpack_frame(raw).payload).command_id == CommandId.DRONE_SET_DROP_TASK for raw in sent))
                self.assertTrue(all(raw == sent[0] for raw in sent))

    def test_wrong_source_or_session_does_not_confirm(self):
        sent = []
        def send(raw):
            sent.append(raw)
            self.ack(sender, raw, src=NodeId.GROUND)
            self.ack(sender, raw, session=124)
        sender = self.make_sender(send, retries=0)
        self.assertFalse(sender.send_task_and_start(self.counts, task_id=42))
        self.assertEqual(len(sent), 1)

    def test_received_and_accepted_can_advance_to_completed_without_downgrade(self):
        def send(raw):
            self.ack(sender, raw, AckStatus.RECEIVED)
            self.ack(sender, raw, AckStatus.ACCEPTED)
            self.ack(sender, raw, AckStatus.COMPLETED)
            self.ack(sender, raw, AckStatus.RECEIVED)
        sender = self.make_sender(send, retries=0)
        self.assertTrue(sender.send_task_and_start(self.counts, task_id=42))

    def test_loss_retry_keeps_same_frame_then_starts_once(self):
        sent = []
        def send(raw):
            sent.append(raw)
            if len(sent) > 1:
                self.ack(sender, raw)
        sender = self.make_sender(send)
        self.assertTrue(sender.send_task_and_start(self.counts, task_id=42))
        self.assertEqual(sent[0], sent[1])
        self.assertEqual(len(sent), 3)

    def test_data_only_and_cancellation_never_start(self):
        sent = []
        def send(raw):
            sent.append(raw)
            self.ack(sender, raw)
        sender = self.make_sender(send)
        self.assertTrue(sender.send_task_and_start(self.counts, task_id=42, start_drone=False))
        self.assertEqual(len(sent), 1)

    def test_cancellation_after_data_ack_prevents_start(self):
        event = threading.Event()
        sent = []
        def send(raw):
            sent.append(raw)
            self.ack(sender, raw)
            event.set()
        sender = self.make_sender(send, stop_event=event)
        self.assertFalse(sender.send_task_and_start(self.counts, task_id=42))
        self.assertEqual(len(sent), 1)
        event = threading.Event()
        event.set()
        sender = self.make_sender(send, stop_event=event)
        self.assertFalse(sender.send_task_and_start(self.counts, task_id=42))
        self.assertEqual(len(sent), 1)

    def test_bad_counts_or_task_id_never_write(self):
        for counts, task_id in ((SimpleNamespace(red=2, blue=1, green=1), 1), (self.counts, 0),
                                (SimpleNamespace(red=True, blue=1, green=2), 1)):
            send = Mock()
            with self.assertRaises(ValueError):
                self.make_sender(send).send_task_and_start(counts, task_id=task_id)
            send.assert_not_called()

    def test_driver_wrapper_enables_bridge_and_closes_after_timeout(self):
        driver = Mock()
        driver.wait_connected.return_value = False
        factory = Mock(return_value=driver)
        self.assertFalse(send_task_and_start(self.counts, driver_factory=factory))
        self.assertTrue(factory.call_args.kwargs["bridge_envelope"])
        driver.write.assert_not_called()
        driver.close.assert_called_once()

    def test_run_close_and_new_sequence_remap_queries(self):
        sent=[]
        statuses=iter((AckStatus.COMPLETED,AckStatus.ACCEPTED,AckStatus.COMPLETED))
        def send(raw):
            sent.append(raw)
            self.ack(sender,raw,next(statuses))
        sender=self.make_sender(send)
        self.assertTrue(sender.close_drone_payload(7))
        self.assertTrue(sender.begin_drone_remap(7))
        self.assertTrue(sender.wait_drone_remap(7))
        frames=[unpack_frame(raw) for raw in sent]
        self.assertEqual(len({f.seq for f in frames}),3)
        commands=[decode_command(f.payload) for f in frames]
        self.assertEqual([c.command_id for c in commands],[0x28,0x29,0x29])
        self.assertTrue(all(c.command_body==struct.pack('<BI',1,7) for c in commands))

    def test_remap_rejection_and_cancellation_never_ready(self):
        sent=[]
        def send(raw):
            sent.append(raw); self.ack(sender,raw,AckStatus.FAILED)
        sender=self.make_sender(send)
        self.assertFalse(sender.wait_drone_remap(7))
        sender.stop_event.set()
        self.assertFalse(sender.close_drone_payload(7))
        self.assertFalse(sender.begin_drone_remap(7))
        self.assertEqual(len(sent),1)


if __name__ == "__main__":
    unittest.main()
