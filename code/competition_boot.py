"""HC-15 competition listener; the mission worker shares its single radio owner.

The car competition chain contacts no drone. This listener keeps the ground
station's RUN / STOP / PING handling and their ACKs, the single radio owner, the
20 s preparation wait and the 5 s completion beep; it creates no
``FleetTaskSender``, restarts no car SLAM sidecar and waits for no drone ACK, so
a drone that is powered off, absent or on a broken link cannot block startup.
"""
import logging
from pathlib import Path
import secrets
import signal
import shutil
import subprocess
import threading
import time

from components.fleet_models import (AckPayload, AckReason, AckStatus, CommandId,
    Frame, MessageKind, NodeId)
from components.fleet_protocol import (FrameParser, RecentResponseCache, SequenceCounter,
    decode_command, encode_ack, new_session, pack_frame)
from components.payload_servos import open_payload, close_payload
from components.serial_communication import HC15SerialDriver
from components.sound_light_alarm import SoundLightAlarm

LOG = logging.getLogger(__name__)
RUN_DELAY_S = 20.0
BUZZ_LAST_S = 5.0
PROMPT_BEEP_S = 0.15
PROMPT_GAP_S = 0.15
SLAM_SERVICE = 'robocup-slam.service'
SYSTEMCTL = shutil.which('systemctl') or '/usr/bin/systemctl'
MISSION_CONFIG = Path(__file__).resolve().parents[1] / 'configs' / 'robocup_diffdrive.toml'
# No --relative-slam: the competition entry builds the wall runtime, and the
# fused/SLAM profile must not be applied on the way in.
MISSION_ARGS = ('--competition', '--mode', 'hardware-mission',
                '--config', str(MISSION_CONFIG))


def restart_car_slam_sidecar(stop_event):
    """Manual diagnostic helper only; the competition chain never calls it."""
    def identity():
        result = subprocess.run([SYSTEMCTL, 'show', SLAM_SERVICE,
            '--property=ActiveState,MainPID,InvocationID'], check=True,
            capture_output=True, text=True, timeout=5)
        return dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)

    if stop_event.is_set():
        raise RuntimeError('SLAM restart cancelled')
    before = identity()
    if stop_event.is_set():
        raise RuntimeError('SLAM restart cancelled')
    process = subprocess.Popen(['sudo', '-n', SYSTEMCTL, 'restart', SLAM_SERVICE],
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
                 alarm=None, clock=time.monotonic, car_only_test=False, simulate_payload=False):
        if mission_main is None:
            from main_robocup import main
            mission_main = main
        self._mission_main = mission_main
        self._car_only_test = car_only_test
        self._simulate_payload = simulate_payload
        self._clock = clock
        self._alarm = SoundLightAlarm() if alarm is None else alarm
        self._state_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._tx_seq_lock = threading.Lock()
        self._payload_action_lock = threading.Lock()
        self._worker = self._stop_event = self._runtime = None
        self._closed = False
        self._parser = FrameParser(local_node=NodeId.CAR)
        self._cache = RecentResponseCache()
        self._session = new_session()
        self._seq = SequenceCounter()
        self._driver = driver_factory(on_bytes=self._on_bytes, port='/dev/ttyS4',
            baudrate=115200, bridge_envelope=True, on_disconnected=self._on_disconnected,
            on_callback_error=self._on_callback_error)

    def _write_frame(self, data):
        with self._write_lock:
            self._driver.write(data)

    def _next_seq(self):
        with self._tx_seq_lock:
            return self._seq.next()

    def _attempt(self, label, action):
        try:
            if action() is False:
                LOG.error('%s failed: returned False', label)
                return False
            LOG.info('%s command completed', label)
            return True
        except Exception:
            LOG.exception('%s failed', label)
            return False

    def _payload_command(self, action):
        if self._simulate_payload:
            LOG.info('SIMULATION payload output skipped')
            return True
        return action()

    def start(self):
        LOG.info('competition mission config: %s', MISSION_CONFIG)
        self._attempt('alarm initialize OFF', lambda: self._alarm.initialize(active=False))
        self._attempt('alarm OFF', self._alarm.off)
        self._attempt('startup payload OPEN', lambda: self._payload_command(open_payload))
        self._driver.start()
        return self

    def _on_bytes(self, data):
        for frame in self._parser.feed(data):
            if frame.src == NodeId.GROUND and frame.dst == NodeId.CAR and frame.kind == MessageKind.COMMAND:
                self._handle_ground(frame)
        # Frames addressed to (or from) the drone are read and discarded: they
        # are not car task input and they never trigger a car-to-drone write.

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

    def _start_mission(self, options=None):
        with self._state_lock:
            if self._closed or (self._worker is not None and self._worker.is_alive()):
                return AckStatus.REJECTED, AckReason.BUSY
            from main_robocup import stop_marker_signature
            try:
                stop_file_baseline = stop_marker_signature(Path('logs/competition/STOP'))
            except OSError:
                LOG.exception('could not capture RUN STOP file baseline')
                stop_file_baseline = None
            stop = threading.Event()
            run_id = secrets.randbelow(0xffffffff) + 1
            t0 = self._clock()
            self._stop_event = stop
            self._worker = threading.Thread(target=self._run_mission,
                args=(stop, run_id, t0, stop_file_baseline, options), name='competition-mission')
            try:
                self._worker.start()
            except RuntimeError:
                self._worker = self._stop_event = None
                LOG.exception('could not start competition worker')
                return AckStatus.FAILED, AckReason.INTERNAL_ERROR
        LOG.info('RUN ACCEPTED run_id=%d', run_id)
        return AckStatus.ACCEPTED, AckReason.NONE

    def local_command(self, action, *, options=None):
        if options is not None:
            from components.competition_options import CompetitionOptions
            options = CompetitionOptions(**options).as_dict()
        if action == 'start':
            status, reason = self._start_mission(options)
            LOG.info('LOCAL RUN %s reason=%s', status.name, reason.name)
            return dict(ok=status == AckStatus.ACCEPTED, status=status.name, reason=reason.name)
        if action == 'stop':
            self._cancel_current()
            LOG.info('LOCAL STOP completed')
            return dict(ok=True, status='STOP_REQUESTED')
        if action == 'status':
            with self._state_lock:
                return dict(ok=True, running=self._worker is not None and self._worker.is_alive(),
                            closed=self._closed)
        raise ValueError('Unknown local action')

    def _beep(self, stop, *, label, count=1, duration=None):
        duration = PROMPT_BEEP_S if duration is None else duration
        for index in range(count):
            with self._payload_action_lock:
                if stop.is_set():
                    return False
                self._attempt('alarm ON', self._alarm.on)
                LOG.info('BUZZ ON cue=%s pulse=%d/%d', label, index + 1, count)
            try:
                if stop.wait(duration):
                    return False
            finally:
                self._attempt('alarm OFF', self._alarm.off)
                LOG.info('BUZZ OFF cue=%s pulse=%d/%d', label, index + 1, count)
            if index + 1 < count and stop.wait(PROMPT_GAP_S):
                return False
        return not stop.is_set()

    def _run_mission(self, stop, run_id, t0, stop_file_baseline=None, options=None):
        completed = False
        try:
            with self._payload_action_lock:
                if stop.is_set():
                    return
                closed = self._attempt('RUN payload CLOSE', lambda: self._payload_command(close_payload))
            LOG.info('RUN payload CLOSE %s run_id=%d (no mechanical feedback)',
                     'SIMULATED' if self._simulate_payload else 'written' if closed else 'failed; continuing', run_id)
            close_confirmed_at = self._clock()
            LOG.info('CLOSE attempt finished; full %.1f s countdown starts at %.3f',
                     RUN_DELAY_S, close_confirmed_at)
            if stop.wait(RUN_DELAY_S):
                return
            if stop.is_set():
                return
            # No drone CLOSE/REMAP wait and no SLAM sidecar restart: the wall
            # chain needs neither a drone nor a rebuilt map. The 5 s beep is the
            # completion cue and must not be shortened by anything in parallel.
            if not self._beep(stop, label='payload_ready', duration=BUZZ_LAST_S):
                return

            kwargs = dict(stop_event=stop, on_runtime_ready=self._runtime_ready,
                          stop_file_baseline=stop_file_baseline,
                          # Preserve RUN timing for diagnostics, without a task deadline.
                          run_started_at_s=t0)
            if options is not None:
                kwargs['run_options'] = options
            if self._simulate_payload:
                kwargs['simulate_payload'] = True
            # The compatibility test listener also selects fixed 1/2/1 counts.
            if self._car_only_test:
                kwargs['car_only_test'] = True
                LOG.info('CAR FULL-FLOW TEST: fixed red/blue/green 1/2/1, task board skipped')
            result = self._mission_main(list(MISSION_ARGS), **kwargs)
            if result:
                LOG.warning('competition returned status=%s run_id=%d', result, run_id)
            completed = not stop.is_set()
        except Exception:
            LOG.exception('competition worker failed run_id=%d', run_id)
        finally:
            stop.set()
            self._attempt('finally alarm OFF', self._alarm.off)
            with self._payload_action_lock:
                self._attempt('finally payload OPEN', lambda: self._payload_command(open_payload))
            with self._state_lock:
                self._runtime = self._stop_event = None
            LOG.info('FULL CLEANUP run_id=%d', run_id)

    def _runtime_ready(self, runtime):
        with self._state_lock:
            self._runtime = runtime
            cancelled = self._closed or (self._stop_event is not None and self._stop_event.is_set())
        if cancelled:
            self._stop_runtime(runtime)

    def _stop_runtime(self, runtime):
        """Stop through the interface the runtime actually has.

        The competition runtime is a ``WallRuntime``: it has no ``motion``
        attribute, and the interface that actually prevents a stale control tick
        from re-energising the motors is ``request_stop()``, which latches
        irreversibly under the command lock, cancels the action and zeroes the
        drive. The legacy fused runtime keeps its own branch so a diagnostic
        tool cannot be broken by this change.
        """
        if runtime is None:
            return
        if hasattr(runtime, "request_stop"):
            self._attempt("wall runtime stop",
                          lambda: runtime.request_stop("remote STOP"))
            return
        self._attempt('base stop', runtime.drive.stop)
        self._attempt('motion stop', lambda: runtime.motion.stop())
        self._attempt('mission safe stop',
                      lambda: runtime.mission.request_safe_stop('remote STOP'))

    def _cancel_current(self):
        with self._state_lock:
            stop, runtime = self._stop_event, self._runtime
            if stop is not None:
                stop.set()
        with self._payload_action_lock:
            self._attempt('STOP alarm OFF', self._alarm.off)
            self._attempt('STOP payload OPEN', lambda: self._payload_command(open_payload))
        self._stop_runtime(runtime)

    def _on_disconnected(self, error=None):
        LOG.error('HC-15 disconnected: %s', error)

    def _on_callback_error(self, error):
        LOG.error('HC-15 callback failed: %s', error)

    def close(self):
        with self._state_lock:
            self._closed = True
            worker = self._worker
        self._cancel_current()
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=35)
            if worker.is_alive():
                raise RuntimeError('Mission worker still cancelling; base stopped and HC-15 remains open')
        self._driver.close()


def main(*, car_only_test=False, local_control=True):
    logging.basicConfig(level=logging.INFO)
    done = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: done.set())
    boot = CompetitionBoot(car_only_test=car_only_test)
    control = None
    boot_started = False
    try:
        if local_control:
            from components.local_mission_control import LocalMissionControl
            control = LocalMissionControl(boot.local_command)
            control.bind()
        boot_started = True
        boot.start()
        if control is not None:
            control.start()
            LOG.info('Local full-flow control ready: %s', control.path)
        done.wait()
    finally:
        if control is not None:
            control.close()
        if boot_started:
            boot.close()


if __name__ == '__main__':
    main()
