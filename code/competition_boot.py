"""Listen for ground RUN/STOP and run one complete competition mission at a time.

This is an opt-in entry point. It owns the HC-15 throughout its lifetime; the
competition worker uses that same transport for the existing SET/START handoff.
"""

from collections import OrderedDict
import logging
import signal
import threading
import time

from components.fleet_models import (
    AckPayload, AckReason, AckStatus, CommandId, CommandPayload, Frame,
    MessageKind, NodeId,
)
from components.fleet_protocol import (
    FrameParser, SequenceCounter, decode_command, encode_ack, encode_command,
    new_session, pack_frame,
)
from components.fleet_task_sender import FleetTaskSender
from components.serial_communication import DEFAULT_BAUDRATE, DEFAULT_HC15_PORT, HC15SerialDriver
from main_robocup import main as competition_main


LOG = logging.getLogger(__name__)
MISSION_ARGS = ["--mode", "hardware-mission", "--competition"]


class CompetitionBoot:
    def __init__(self, *, driver_factory=HC15SerialDriver, mission_main=competition_main):
        self._mission_main = mission_main
        self._parser = FrameParser(local_node=NodeId.CAR)
        self._session = new_session()
        self._seq = SequenceCounter()
        self._responses = OrderedDict()
        self._state_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._worker = None
        self._stop_event = None
        self._runtime = None
        self._sender = None
        self._driver = driver_factory(
            port=DEFAULT_HC15_PORT, baudrate=DEFAULT_BAUDRATE, bridge_envelope=True,
            on_bytes=self._on_bytes, on_connected=lambda: LOG.info("HC-15 connected"),
            on_disconnected=self._on_disconnected,
            on_callback_error=self._on_callback_error,
        )

    def start(self):
        self._driver.start()
        return self

    def write(self, raw):
        """FleetTaskSender transport: cancellation and one-frame writes are atomic."""
        with self._write_lock:
            if self._stop_event is None or self._stop_event.is_set():
                raise RuntimeError("competition task send was cancelled")
            self._driver.write(raw)

    def _on_bytes(self, data):
        frames = self._parser.feed(data)
        # Handle ground STOP before releasing a sender waiting on a drone ACK.
        for frame in frames:
            if (frame.src == NodeId.GROUND and frame.dst == NodeId.CAR
                    and frame.kind == MessageKind.COMMAND):
                self._handle_ground(frame)
        with self._state_lock:
            sender = self._sender
        if sender is not None:
            sender.feed_bytes(data)

    def _handle_ground(self, frame):
        key = (frame.session, frame.seq)
        cached = self._responses.get(key)
        if cached is not None:
            self._responses.move_to_end(key)
            self._driver.write(cached)
            return
        command_id = frame.payload[0] if frame.payload else 0
        status, reason = AckStatus.REJECTED, AckReason.BAD_PAYLOAD
        try:
            command = decode_command(frame.payload)
            command_id = command.command_id
            if command.command_flags or command.command_body:
                raise ValueError("RUN/STOP flags and bodies must be empty")
            if command_id == CommandId.CAR_START_MISSION:
                status, reason = self._start_mission()
            elif command_id == CommandId.TARGETED_STOP:
                self._cancel_current()
                self._forward_stop()
                status, reason = AckStatus.ACCEPTED, AckReason.NONE
            else:
                status, reason = AckStatus.REJECTED, AckReason.UNSUPPORTED
        except ValueError:
            pass
        reply = pack_frame(Frame(1, NodeId.CAR, NodeId.GROUND, MessageKind.ACK, 0,
            self._session, self._seq.next(), encode_ack(AckPayload(
                frame.session, frame.seq, command_id, status, reason))))
        self._responses[key] = reply
        if len(self._responses) > 64:
            self._responses.popitem(last=False)
        self._driver.write(reply)

    def _start_mission(self):
        with self._state_lock:
            if self._worker is not None and self._worker.is_alive():
                return AckStatus.REJECTED, AckReason.BUSY
            stop_event = threading.Event()
            sender = FleetTaskSender(self, stop_event=stop_event)
            worker = threading.Thread(target=self._run_mission, args=(stop_event, sender),
                                      name="competition-mission")
            self._stop_event = stop_event
            self._sender = sender
            self._worker = worker
            try:
                worker.start()
            except RuntimeError:
                self._worker = self._sender = self._stop_event = None
                LOG.exception("could not start competition worker")
                return AckStatus.FAILED, AckReason.INTERNAL_ERROR
        return AckStatus.ACCEPTED, AckReason.NONE

    def _run_mission(self, stop_event, sender):
        def publish_runtime(runtime):
            with self._state_lock:
                if self._stop_event is stop_event:
                    self._runtime = runtime
            if runtime is not None and stop_event.is_set():
                self._stop_runtime(runtime)

        try:
            result = self._mission_main(list(MISSION_ARGS), stop_event=stop_event,
                send_drone_task=lambda counts: sender.send_task_and_start(counts, start_drone=True),
                on_runtime_ready=publish_runtime)
            LOG.info("competition worker exited with status %s", result)
        except Exception:
            LOG.exception("competition worker failed")
        finally:
            with self._state_lock:
                if self._stop_event is stop_event:
                    self._runtime = self._sender = self._stop_event = None

    def _stop_runtime(self, runtime):
        try:
            runtime.mission.request_safe_stop("ground STOP")
        except Exception:
            LOG.exception("mission safe stop failed")
        try:
            runtime.drive.stop()
        except Exception:
            LOG.exception("drive stop failed")

    def _cancel_current(self):
        with self._state_lock:
            event, runtime = self._stop_event, self._runtime
            if event is not None:
                event.set()
        if runtime is not None:
            self._stop_runtime(runtime)

    def _forward_stop(self):
        raw = pack_frame(Frame(1, NodeId.CAR, NodeId.DRONE, MessageKind.COMMAND,
            0, self._session, self._seq.next(),
            encode_command(CommandPayload(CommandId.TARGETED_STOP))))
        try:
            with self._write_lock:
                self._driver.write(raw)
        except (OSError, RuntimeError):
            # The ground ACK below acknowledges the car's stop request only.
            LOG.exception("could not forward STOP to drone")

    def _on_disconnected(self, error):
        if error is not None:
            LOG.warning("HC-15 disconnected: %s", error)
        self._cancel_current()

    def _on_callback_error(self, error):
        LOG.error("HC-15 callback failed: %s", error)
        self._cancel_current()

    def close(self):
        with self._state_lock:
            active = self._worker is not None and self._worker.is_alive()
        self._cancel_current()
        if active:
            self._forward_stop()
        with self._state_lock:
            worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=10.0)
            if worker.is_alive():
                raise RuntimeError("competition worker did not stop; HC-15 remains open")
        self._driver.close()


def main():
    logging.basicConfig(level=logging.INFO)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def terminate(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    boot = None
    try:
        boot = CompetitionBoot().start()
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        LOG.info("competition listener stopping")
    finally:
        try:
            if boot is not None:
                boot.close()
        finally:
            signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()
