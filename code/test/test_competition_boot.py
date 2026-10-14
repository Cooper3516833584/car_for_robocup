"""Hardware-free checks for the one-radio competition listener."""

from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from competition_boot import CompetitionBoot, MISSION_ARGS
from components.fleet_models import (
    AckPayload, AckReason, AckStatus, CommandId, CommandPayload, Frame,
    MessageKind, NodeId,
)
from components.fleet_protocol import (
    decode_ack, decode_command, encode_ack, encode_command, pack_frame, unpack_frame,
)


class FakeDriver:
    def __init__(self, *, on_bytes, **settings):
        self.on_bytes = on_bytes
        self.settings = settings
        self.writes = []
        self.on_write = None
        self.started = self.closed = False

    def start(self):
        self.started = True

    def close(self):
        self.closed = True

    def write(self, raw):
        self.writes.append(unpack_frame(raw))
        if self.on_write is not None:
            self.on_write(self.writes[-1])


def ground_command(seq, command_id, *, session=10, body=b""):
    return pack_frame(Frame(1, NodeId.GROUND, NodeId.CAR, MessageKind.COMMAND,
        0, session, seq, encode_command(CommandPayload(command_id, command_body=body))))


class CompetitionBootTests(unittest.TestCase):
    def make_boot(self, runner):
        drivers = []

        def factory(**settings):
            driver = FakeDriver(**settings)
            drivers.append(driver)
            return driver

        boot = CompetitionBoot(driver_factory=factory, mission_main=runner).start()
        self.assertEqual(len(drivers), 1)
        self.assertTrue(drivers[0].settings["bridge_envelope"])
        return boot, drivers[0]

    def test_run_stop_replay_and_restart_after_worker_exit(self):
        calls = []
        entered = threading.Event()
        runtime = SimpleNamespace(mission=Mock(), drive=Mock())

        def runner(argv, *, stop_event, send_drone_task, on_runtime_ready):
            calls.append((argv, stop_event))
            on_runtime_ready(runtime)
            entered.set()
            stop_event.wait(2)
            on_runtime_ready(None)
            return 1

        boot, driver = self.make_boot(runner)
        try:
            run = ground_command(1, CommandId.CAR_START_MISSION)
            driver.on_bytes(run)
            self.assertTrue(entered.wait(1))
            self.assertEqual(calls[0][0], MISSION_ARGS)
            self.assertEqual(decode_ack(driver.writes[-1].payload).status, AckStatus.ACCEPTED)
            driver.on_bytes(run)
            self.assertEqual(len(calls), 1)
            driver.on_bytes(ground_command(2, CommandId.CAR_START_MISSION))
            busy = decode_ack(driver.writes[-1].payload)
            self.assertEqual((busy.status, busy.reason), (AckStatus.REJECTED, AckReason.BUSY))
            driver.on_bytes(ground_command(3, CommandId.TARGETED_STOP))
            self.assertTrue(calls[0][1].is_set())
            runtime.mission.request_safe_stop.assert_called()
            runtime.drive.stop.assert_called()
            forwarded = [frame for frame in driver.writes if frame.dst == NodeId.DRONE]
            self.assertEqual(len(forwarded), 1)
            stop_command = decode_command(forwarded[0].payload)
            self.assertEqual((stop_command.command_id, stop_command.command_body),
                             (CommandId.TARGETED_STOP, b""))
            self.assertEqual(decode_ack(driver.writes[-1].payload).status, AckStatus.ACCEPTED)
            boot._worker.join(1)
            self.assertFalse(boot._worker.is_alive())
            driver.on_bytes(ground_command(5, CommandId.TARGETED_STOP))
            self.assertEqual(len([frame for frame in driver.writes
                                  if frame.dst == NodeId.DRONE]), 2)
            entered.clear()
            driver.on_bytes(ground_command(4, CommandId.CAR_START_MISSION))
            self.assertTrue(entered.wait(1))
            driver.on_bytes(run)  # An old RUN remains a replay after the next round starts.
            self.assertEqual(len(calls), 2)
        finally:
            boot.close()
        self.assertTrue(driver.closed)

    def test_invalid_or_non_ground_commands_cannot_start_a_mission(self):
        runner = Mock()
        boot, driver = self.make_boot(runner)
        try:
            driver.on_bytes(ground_command(1, CommandId.CAR_START_MISSION, body=b"bad"))
            ack = decode_ack(driver.writes[-1].payload)
            self.assertEqual((ack.status, ack.reason),
                             (AckStatus.REJECTED, AckReason.BAD_PAYLOAD))
            driver.on_bytes(ground_command(2, CommandId.TARGETED_STOP, body=b"bad"))
            self.assertEqual(decode_ack(driver.writes[-1].payload).status, AckStatus.REJECTED)
            drone_run = pack_frame(Frame(1, NodeId.DRONE, NodeId.CAR,
                MessageKind.COMMAND, 0, 55, 1,
                encode_command(CommandPayload(CommandId.CAR_START_MISSION))))
            driver.on_bytes(drone_run)
            self.assertEqual(len(driver.writes), 2)
            runner.assert_not_called()
        finally:
            boot.close()

    def test_shared_sender_waits_for_set_completed_before_start(self):
        done = threading.Event()
        result = []
        counts = SimpleNamespace(red=2, blue=0, green=2)

        def runner(argv, *, stop_event, send_drone_task, on_runtime_ready):
            result.append(send_drone_task(counts))
            done.set()
            return 0

        boot, driver = self.make_boot(runner)

        def answer(frame):
            if frame.dst != NodeId.DRONE:
                return
            command = decode_command(frame.payload)
            status = (AckStatus.COMPLETED if command.command_id == CommandId.DRONE_SET_DROP_TASK
                      else AckStatus.ACCEPTED)
            ack = AckPayload(frame.session, frame.seq, command.command_id, status)
            driver.on_bytes(pack_frame(Frame(1, NodeId.DRONE, NodeId.CAR,
                MessageKind.ACK, 0, 20, frame.seq, encode_ack(ack))))

        driver.on_write = answer
        try:
            driver.on_bytes(ground_command(1, CommandId.CAR_START_MISSION))
            self.assertTrue(done.wait(1))
            self.assertEqual(result, [True])
            sent = [decode_command(frame.payload) for frame in driver.writes
                    if frame.dst == NodeId.DRONE]
            self.assertEqual([item.command_id for item in sent],
                [CommandId.DRONE_SET_DROP_TASK, CommandId.DRONE_START_DROP_TASK])
        finally:
            boot.close()

    def test_stop_cancels_pending_set_without_start(self):
        entered = threading.Event()
        done = threading.Event()
        result = []

        def runner(argv, *, stop_event, send_drone_task, on_runtime_ready):
            entered.set()
            result.append(send_drone_task(SimpleNamespace(red=1, blue=2, green=1)))
            done.set()
            return 1

        boot, driver = self.make_boot(runner)
        try:
            driver.on_bytes(ground_command(1, CommandId.CAR_START_MISSION))
            self.assertTrue(entered.wait(1))
            driver.on_bytes(ground_command(2, CommandId.TARGETED_STOP))
            self.assertTrue(done.wait(1))
            self.assertEqual(result, [False])
            ids = [decode_command(frame.payload).command_id for frame in driver.writes
                   if frame.dst == NodeId.DRONE]
            self.assertIn(CommandId.TARGETED_STOP, ids)
            self.assertNotIn(CommandId.DRONE_START_DROP_TASK, ids)
        finally:
            boot.close()


if __name__ == "__main__":
    unittest.main()
