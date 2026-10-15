"""HC-15 competition listener; the mission worker shares its single radio owner."""
import logging
import secrets
import signal
import subprocess
import threading
import time

from components.fleet_models import (AckPayload, AckReason, AckStatus, CommandId,
    CommandPayload, Frame, MessageKind, NodeId)
from components.fleet_protocol import (FrameParser, RecentResponseCache, SequenceCounter,
    decode_command, encode_ack, encode_command, new_session, pack_frame)
from components.fleet_task_sender import FleetTaskSender
from components.payload_servos import open_payload, close_payload
from components.serial_communication import HC15SerialDriver
from components.sound_light_alarm import SoundLightAlarm

LOG = logging.getLogger(__name__)
RUN_DELAY_S = 20.0
BUZZ_LAST_S = 5.0
SLAM_SERVICE = 'robocup-slam.service'
MISSION_ARGS = ('--competition', '--mode', 'hardware-mission', '--relative-slam')


def restart_car_slam_sidecar(stop_event):
    def identity():
        result = subprocess.run(['systemctl', 'show', SLAM_SERVICE,
            '--property=ActiveState,MainPID,InvocationID'], check=True,
            capture_output=True, text=True, timeout=5)
        return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)

    if stop_event.is_set():
        raise RuntimeError('SLAM restart cancelled')
    before = identity()
    if stop_event.is_set():
        raise RuntimeError('SLAM restart cancelled')
    process = subprocess.Popen(['systemctl', 'restart', SLAM_SERVICE],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 30
        while process.poll() is None:
            if stop_event.wait(.05) or time.monotonic() >= deadline:
                raise RuntimeError('SLAM restart cancelled or timed out')
        if process.returncode:
            raise RuntimeError('SLAM service restart failed')
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)
    after = identity()
    if (stop_event.is_set() or after.get('ActiveState') != 'active' or
            int(after.get('MainPID', '0')) <= 0 or not after.get('InvocationID') or
            after['InvocationID'] == before.get('InvocationID')):
        raise RuntimeError('new SLAM instance was not confirmed')
    LOG.info('CAR NEW SLAM instance=%s; runtime must still await fresh anchor', after)
    return after


class CompetitionBoot:
    def __init__(self, *, driver_factory=HC15SerialDriver, mission_main=None,
                 alarm=None, clock=time.monotonic):
        if mission_main is None:
            from main_robocup import main
            mission_main = main
        self._mission_main = mission_main
        self._clock = clock
        self._alarm = SoundLightAlarm() if alarm is None else alarm
        self._state_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._tx_seq_lock = threading.Lock()
        self._payload_action_lock = threading.Lock()
        self._worker = self._stop_event = self._sender = self._runtime = None
        self._closed = False
        self._parser = FrameParser(local_node=NodeId.CAR)
        self._cache = RecentResponseCache()
        self._session = new_session()
        self._seq = SequenceCounter()
        self._driver = driver_factory(on_bytes=self._on_bytes, port='/dev/ttyS4',
            baudrate=115200, bridge_envelope=True, on_disconnected=self._on_disconnected,
            on_callback_error=self._on_callback_error)

    def write(self, data):
        with self._write_lock:
            if self._stop_event is None or self._stop_event.is_set():
                raise RuntimeError('competition task send was cancelled')
            self._driver.write(data)

    def _write_frame(self, data):
        with self._write_lock:
            self._driver.write(data)

    def _next_seq(self):
        with self._tx_seq_lock:
            return self._seq.next()

    def _attempt(self, label, action):
        try:
            action()
            return True
        except Exception:
            LOG.exception('%s failed', label)
            return False

    def start(self):
        self._attempt('alarm initialize OFF', lambda: self._alarm.initialize(active=False))
        self._attempt('alarm OFF', self._alarm.off)
        self._attempt('startup payload OPEN', open_payload)
        self._driver.start()
        return self

    def _on_bytes(self, data):
        for frame in self._parser.feed(data):
            if frame.src == NodeId.GROUND and frame.dst == NodeId.CAR and frame.kind == MessageKind.COMMAND:
                self._handle_ground(frame)
        with self._state_lock:
            sender = self._sender
        if sender is not None:
            sender.feed_bytes(data)

    def _handle_ground(self, frame):
        self._cache.begin_ground_session(frame.session)
        cached = self._cache.get(frame.session, frame.seq)
        if cached is not None:
            self._attempt('cached ACK', lambda: self._write_frame(cached))
            return
        command_id = frame.payload[0] if frame.payload else 0
        try:
            command = decode_command(frame.payload)
            command_id = command.command_id
            if command.command_flags or command.command_body:
                raise ValueError('command flags/body must be empty')
            if command_id == CommandId.CAR_START_MISSION:
                status, reason = self._start_mission()
            elif command_id == CommandId.TARGETED_STOP:
                self._cancel_current()
                self._forward_stop()
                status, reason = AckStatus.ACCEPTED, AckReason.NONE
            elif command_id == CommandId.PING:
                status, reason = AckStatus.COMPLETED, AckReason.NONE
            else:
                status, reason = AckStatus.REJECTED, AckReason.UNSUPPORTED
        except ValueError:
            status, reason = AckStatus.REJECTED, AckReason.BAD_PAYLOAD
        raw = pack_frame(Frame(1, NodeId.CAR, frame.src, MessageKind.ACK, 0,
            self._session, self._next_seq(), encode_ack(AckPayload(
                frame.session, frame.seq, command_id, status, reason))))
        self._cache.put(frame.session, frame.seq, raw)
        self._attempt('ground ACK', lambda: self._write_frame(raw))

    def _start_mission(self):
        with self._state_lock:
            if self._closed or (self._worker is not None and self._worker.is_alive()):
                return AckStatus.REJECTED, AckReason.BUSY
            stop = threading.Event()
            run_id = secrets.randbelow(0xffffffff) + 1
            t0 = self._clock()
            sender = FleetTaskSender(self, stop_event=stop)
            self._stop_event, self._sender = stop, sender
            self._worker = threading.Thread(target=self._run_mission,
                args=(stop, sender, run_id, t0), name='competition-mission')
            try:
                self._worker.start()
            except RuntimeError:
                self._worker = self._sender = self._stop_event = None
                LOG.exception('could not start competition worker')
                return AckStatus.FAILED, AckReason.INTERNAL_ERROR
        LOG.info('RUN ACCEPTED run_id=%d', run_id)
        return AckStatus.ACCEPTED, AckReason.NONE

    def _run_mission(self, stop, sender, run_id, t0):
        completed = False
        try:
            with self._payload_action_lock:
                if stop.is_set():
                    return
                closed = close_payload()
            LOG.info('CAR CLOSE %s run_id=%d (no mechanical feedback)',
                     'SKIPPED: PWM unconfigured' if closed is False else 'SENT', run_id)
            if not sender.close_drone_payload(run_id):
                raise RuntimeError('drone CLOSE was not confirmed by software ACK')
            if stop.wait(max(0, t0 + RUN_DELAY_S - BUZZ_LAST_S - self._clock())):
                return
            with self._payload_action_lock:
                if stop.is_set():
                    return
                if self._clock() < t0 + RUN_DELAY_S:
                    self._alarm.on()
                    LOG.info('BUZZ ON')
            if stop.wait(max(0, t0 + RUN_DELAY_S - self._clock())):
                return
            self._alarm.off()
            LOG.info('BUZZ OFF')
            if stop.is_set():
                return
            if not sender.begin_drone_remap(run_id):
                raise RuntimeError('drone REMAP not accepted')
            restart_car_slam_sidecar(stop)
            if stop.is_set():
                return

            def handoff(counts):
                return (sender.wait_drone_remap(run_id, stop) and
                        sender.send_task_and_start(counts, start_drone=True))

            result = self._mission_main(list(MISSION_ARGS), stop_event=stop,
                send_drone_task=handoff, on_runtime_ready=self._runtime_ready)
            if result:
                raise RuntimeError('competition mission failed: %s' % result)
            completed = not stop.is_set()
        except Exception:
            LOG.exception('competition worker failed run_id=%d', run_id)
        finally:
            stop.set()
            self._attempt('finally alarm OFF', self._alarm.off)
            with self._payload_action_lock:
                self._attempt('finally payload OPEN', open_payload)
            if not completed:
                self._forward_stop()
            with self._state_lock:
                self._runtime = self._sender = self._stop_event = None
            LOG.info('FULL CLEANUP run_id=%d', run_id)

    def _runtime_ready(self, runtime):
        with self._state_lock:
            self._runtime = runtime
            cancelled = self._closed or (self._stop_event is not None and self._stop_event.is_set())
        if cancelled:
            self._stop_runtime(runtime)

    def _stop_runtime(self, runtime):
        if runtime is not None:
            self._attempt('base stop', runtime.drive.stop)
            self._attempt('motion stop', runtime.motion.stop)
            self._attempt('mission safe stop', lambda: runtime.mission.request_safe_stop('remote STOP'))

    def _cancel_current(self):
        with self._state_lock:
            stop, runtime = self._stop_event, self._runtime
            if stop is not None:
                stop.set()
        self._attempt('STOP alarm OFF', self._alarm.off)
        with self._payload_action_lock:
            self._attempt('STOP payload OPEN', open_payload)
        self._stop_runtime(runtime)

    def _forward_stop(self):
        raw = pack_frame(Frame(1, NodeId.CAR, NodeId.DRONE, MessageKind.COMMAND, 0,
            self._session, self._next_seq(), encode_command(CommandPayload(CommandId.TARGETED_STOP))))
        self._attempt('remote STOP write', lambda: self._write_frame(raw))
        LOG.warning('Remote OPEN unconfirmed: STOP delivery has no awaited ACK')

    def _on_disconnected(self, error=None):
        self._cancel_current()
        self._forward_stop()

    def _on_callback_error(self, error):
        LOG.error('HC-15 callback failed: %s', error)
        self._cancel_current()
        self._forward_stop()

    def close(self):
        with self._state_lock:
            self._closed = True
            worker = self._worker
        self._cancel_current()
        self._forward_stop()
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=35)
            if worker.is_alive():
                raise RuntimeError('Mission worker still cancelling; base stopped and HC-15 remains open')
        self._driver.close()


def main():
    logging.basicConfig(level=logging.INFO)
    done = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: done.set())
    boot = CompetitionBoot()
    try:
        boot.start()
        done.wait()
    finally:
        boot.close()


if __name__ == '__main__':
    main()
