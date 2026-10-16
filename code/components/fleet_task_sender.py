"""Send actual drop quotas, then a matching start, using verified FleetBus frames."""
import logging
import math
import secrets
import struct
import threading
import time

from .drop_task_protocol import DropTask, encode_drop_start, encode_drop_task
from .fleet_models import AckStatus, CommandId, CommandPayload, Frame, MessageKind, NodeId
from .fleet_protocol import FrameParser, SequenceCounter, decode_ack, encode_command, new_session, pack_frame
from .serial_communication import HC15SerialDriver

LOG = logging.getLogger(__name__)


class FleetTaskSender:
    def __init__(self, transport, *, response_timeout_s=6., retries=3, session=None, stop_event=None):
        if (isinstance(response_timeout_s, bool) or not isinstance(response_timeout_s, (int, float))
                or not math.isfinite(response_timeout_s) or response_timeout_s <= 0
                or isinstance(retries, bool) or not isinstance(retries, int) or not 0 <= retries <= 10):
            raise ValueError("invalid task ACK timing")
        self.transport = transport
        self.timeout = response_timeout_s
        self.retries = retries
        self.session = new_session() if session is None else session
        self.stop_event = stop_event or threading.Event()
        self._parser = FrameParser(local_node=NodeId.CAR)
        self._seq = SequenceCounter()
        self._condition = threading.Condition()
        self._pending = None
        self._ack = None
        self._send_lock = threading.Lock()

    def feed_bytes(self, data):
        for frame in self._parser.feed(data):
            if frame.src != NodeId.DRONE or frame.dst != NodeId.CAR or frame.kind != MessageKind.ACK:
                continue
            try:
                ack = decode_ack(frame.payload)
            except ValueError:
                continue
            with self._condition:
                key = (ack.request_session, ack.request_seq, ack.command_id)
                can_advance = (self._ack is None or self._ack.status == AckStatus.RECEIVED
                    or (self._ack.status == AckStatus.ACCEPTED
                        and ack.status in (AckStatus.COMPLETED, AckStatus.REJECTED, AckStatus.FAILED)))
                if key == self._pending and can_advance:
                    self._ack = ack
                    self._condition.notify_all()

    def _request(self, command_id, body, *, require_completed=False, return_status=False,
                 deadline_s=None, on_sent=None):
        seq = self._seq.next()
        raw = pack_frame(Frame(1, NodeId.CAR, NodeId.DRONE, MessageKind.COMMAND, 0,
            self.session, seq, encode_command(CommandPayload(command_id, command_body=body))))
        with self._condition:
            self._pending = (self.session, seq, int(command_id))
            self._ack = None
        sent_notified = False
        try:
            for attempt in range(1, self.retries+2):
                if self.stop_event.is_set() or (deadline_s is not None and time.monotonic() >= deadline_s):
                    return False
                written = False
                try:
                    self.transport.write(raw)
                    written = True
                    LOG.info("Drone command=0x%02x seq=%d attempt=%d", command_id, seq, attempt)
                except (OSError, RuntimeError):
                    LOG.exception("Drone task write failed")
                deadline = time.monotonic()+self.timeout
                if deadline_s is not None:
                    deadline = min(deadline, deadline_s)
                if self.stop_event.is_set():
                    return False
                # Notify once per logical request, after a successful UART write.
                # Run outside the ACK condition so RX can continue during cues.
                if written and not sent_notified:
                    sent_notified = True
                    if on_sent is not None and on_sent() is False:
                        return False
                with self._condition:
                    while not self.stop_event.is_set():
                        ack = self._ack
                        if ack is not None:
                            if ack.status == AckStatus.COMPLETED or (not require_completed and ack.status == AckStatus.ACCEPTED):
                                return ack.status if return_status else True
                            if ack.status in (AckStatus.REJECTED, AckStatus.FAILED):
                                LOG.error("Drone rejected command=0x%02x reason=%d", command_id, ack.reason)
                                return False
                        remaining = deadline-time.monotonic()
                        if remaining <= 0:
                            break
                        self._condition.wait(min(remaining, .1))
            LOG.error("Drone ACK timeout: command=0x%02x seq=%d", command_id, seq)
            return False
        finally:
            with self._condition:
                self._pending = None

    @staticmethod
    def _run_body(run_id):
        if isinstance(run_id, bool) or not isinstance(run_id, int) or not 1 <= run_id <= 0xffffffff:
            raise ValueError("run_id must be a nonzero u32")
        return struct.pack('<BI', 1, run_id)

    def close_drone_payload(self, run_id):
        with self._send_lock:
            return self._request(CommandId.DRONE_PAYLOAD_CLOSE, self._run_body(run_id),
                                 require_completed=True)

    def begin_drone_remap(self, run_id):
        with self._send_lock:
            return self._request(CommandId.DRONE_REBUILD_MAP, self._run_body(run_id))

    def wait_drone_remap(self, run_id, stop_event=None, *, timeout_s=75., poll_s=.5):
        stop = self.stop_event if stop_event is None else stop_event
        deadline = time.monotonic() + timeout_s
        while not stop.is_set() and not self.stop_event.is_set() and time.monotonic() < deadline:
            with self._send_lock:
                status = self._request(CommandId.DRONE_REBUILD_MAP, self._run_body(run_id),
                                       return_status=True, deadline_s=deadline)
            if status == AckStatus.COMPLETED:
                return True
            if status != AckStatus.ACCEPTED:
                return False
            if stop.wait(min(poll_s, max(0, deadline-time.monotonic()))):
                return False
        LOG.error("Drone new-map readiness not confirmed")
        return False

    def send_task_and_start(self, counts, *, task_id=None, start_drone=True,
                            on_task_sent=None, on_task_ack=None):
        task = DropTask(secrets.randbelow(0xffffffff)+1 if task_id is None else task_id,
                        counts.red, counts.blue, counts.green)
        with self._send_lock:
            # No start unless the drone confirms the validated allocation was stored.
            if not self._request(CommandId.DRONE_SET_DROP_TASK, encode_drop_task(task),
                                 require_completed=True, on_sent=on_task_sent):
                return False
            if self.stop_event.is_set():
                return False
            if on_task_ack is not None and on_task_ack() is False:
                return False
            if self.stop_event.is_set():
                return False
            if not start_drone:
                return True
            return self._request(CommandId.DRONE_START_DROP_TASK, encode_drop_start(task.task_id))


def send_task_and_start(counts, *, port="/dev/ttyS4", baudrate=115200,
                        connect_wait_s=2., response_timeout_s=6., retries=3,
                        driver_factory=None, stop_event=None, start_drone=True):
    holder = {}
    factory = HC15SerialDriver if driver_factory is None else driver_factory
    driver = factory(port=port, baudrate=baudrate, bridge_envelope=True,
                     on_bytes=lambda data: holder["sender"].feed_bytes(data))
    sender = FleetTaskSender(driver, response_timeout_s=response_timeout_s,
                             retries=retries, stop_event=stop_event)
    holder["sender"] = sender
    try:
        driver.start()
        if not driver.wait_connected(connect_wait_s):
            LOG.error("HC-15 did not connect; drone was not started")
            return False
        return sender.send_task_and_start(counts, start_drone=start_drone)
    except (OSError, RuntimeError, ValueError):
        LOG.exception("Drone task handoff failed")
        return False
    finally:
        driver.close()
