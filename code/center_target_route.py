"""280/420/250 cm route with asynchronous, colour-independent YOLO stops.

Hardware is opened explicitly, never on import. Only the runtime control thread
commands motion/GPIO; the vision worker supplies observations.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import threading
import time

from components.basic_motion_controller import MotionActionState
from competition_task import _step, _usable_pose
from robocup_runtime import RobocupMissionState

PERIOD_S = 0.05
BEEP_S = 1.0
VISION_MAX_AGE_S = 2.0
MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "best_car.pt"


def select_fastest_cpus(policies, allowed):
    """Choose the fastest available frequency-policy group, without hardcoded IDs."""
    available = [(frequency, set(cpus) & set(allowed)) for frequency, cpus in policies]
    available = [(frequency, cpus) for frequency, cpus in available if cpus]
    if not available:
        return set()
    fastest = max(frequency for frequency, _ in available)
    return set().union(*(cpus for frequency, cpus in available if frequency == fastest))


def pin_vision_worker():
    """Restrict this Linux worker, leaving runtime/sensor thread affinity alone."""
    try:
        policies = []
        for policy in Path("/sys/devices/system/cpu/cpufreq").glob("policy*"):
            policies.append((int((policy / "cpuinfo_max_freq").read_text()),
                             {int(cpu) for cpu in (policy / "related_cpus").read_text().split()}))
        cpus = select_fastest_cpus(policies, os.sched_getaffinity(0))
        if cpus:
            os.sched_setaffinity(0, cpus)  # 0 identifies the calling native thread.
        return sorted(cpus)
    except (OSError, ValueError, AttributeError):
        return []


def central_target(boxes, width, height, min_conf=0.5):
    """Return any confident box whose centre is in the middle half of each axis.

    Input boxes are (x1, y1, x2, y2, confidence, class_id); no colour filter.
    """
    for box in boxes:
        x1, y1, x2, y2, confidence, _class_id = box
        if (not all(math.isfinite(float(v)) for v in box[:5])
                or confidence < min_conf or x2 <= x1 or y2 <= y1):
            continue
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        if width * 0.25 <= cx <= width * 0.75 and height * 0.25 <= cy <= height * 0.75:
            return tuple(box)
    return None


class EntryLatch:
    """One alarm per occupancy; re-arm after three consecutive clear frames."""

    def __init__(self):
        self.armed = True
        self.clear_frames = 0

    def update(self, target):
        if target is None:
            self.clear_frames += 1
            if self.clear_frames >= 3:
                self.armed = True
            return None
        self.clear_frames = 0
        if self.armed:
            self.armed = False
            return target
        return None


class YoloVision:
    """Latest fresh inference plus a latched entry, even if the next frame clears.

    A camera/inference exception or stale inference fails closed before more
    movement. The worker cannot access the runtime or motor interfaces.
    """

    def __init__(self, weights=MODEL_PATH, camera=0, *, imgsz=416, confidence=0.5):
        self.weights = Path(weights)
        self.camera = camera
        self.imgsz = imgsz
        self.confidence = confidence
        self._lock = threading.Lock()
        self._quit = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._pending = None
        self._frame_at = None
        self._error = None

    def start(self):
        if not self.weights.is_file():
            raise FileNotFoundError(self.weights)
        self._thread = threading.Thread(target=self._worker, name="route-yolo", daemon=True)
        self._thread.start()

    def _worker(self):
        capture = None
        try:
            cpus = pin_vision_worker()
            import cv2
            import torch
            from components.yellow_yolo_adapter import load_detector

            torch.set_num_threads(1)  # Measured fastest: one thread on big CPU cores.
            detector = load_detector(self.weights)
            capture = cv2.VideoCapture(self.camera, cv2.CAP_V4L2)
            if not capture.isOpened():
                raise RuntimeError("YOLO camera cannot open")
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            latch = EntryLatch()
            frames = 0
            while not self._quit.is_set():
                ok, frame = capture.read()
                frame_at = time.monotonic()
                if not ok or frame is None:
                    raise RuntimeError("YOLO camera frame unavailable")
                result = detector.predict(frame, imgsz=self.imgsz, conf=self.confidence,
                                          device="cpu", verbose=False)[0]
                frame_age = time.monotonic() - frame_at
                frames += 1
                if frames == 1:
                    initial_threads = torch.get_num_threads()
                    # Ultralytics' CPU backend overrides num_threads at setup.
                    # Reapply the measured setting AFTER setup; discard warm-up.
                    torch.set_num_threads(1)
                    print(f"[vision] cpus={cpus}; backend_initial_threads={initial_threads}; "
                          "steady_threads=1", flush=True)
                if frames <= 3:
                    print(f"[vision] frame={frames} inference_s={frame_age:.3f}", flush=True)
                # CPU/model cold-start can exceed the freshness limit. Discard
                # that frame while stationary; only a fresh result declares ready.
                if frames == 1 or frame_age > VISION_MAX_AGE_S:
                    continue
                boxes = [] if result.boxes is None else result.boxes.data.cpu().tolist()
                # Standard detect models expose xyxy, confidence, class in six columns.
                target = central_target(boxes, frame.shape[1], frame.shape[0], self.confidence)
                entry = latch.update(target)
                with self._lock:
                    self._frame_at = frame_at
                    if entry is not None and self._pending is None:
                        self._pending = entry
                self._ready.set()
        except BaseException as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
            self._ready.set()
        finally:
            if capture is not None:
                capture.release()

    @property
    def ready(self):
        return self._ready.is_set()

    def poll(self, now):
        with self._lock:
            if self._error:
                raise RuntimeError(self._error)
            if self._frame_at is None or now - self._frame_at > VISION_MAX_AGE_S:
                raise RuntimeError("YOLO inference unavailable or stale")
            entry, self._pending = self._pending, None
            return entry

    def close(self):
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)


@dataclass(frozen=True)
class RouteAction:
    label: str
    method: str
    args: tuple

    def start(self, runtime):
        if runtime.mission.state is RobocupMissionState.TARGET_OPERATION:
            runtime.mission.on_payload_action_done()
        getattr(runtime.motion, self.method)(*self.args)


def route_from_pose(pose):
    """Fixed endpoints/headings survive interruption, including during a turn."""
    c, s = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)

    def xy(forward, left):
        return (pose.x_m + forward * c - left * s,
                pose.y_m + forward * s + left * c)

    return (
        RouteAction("forward_280cm", "follow_segment", (xy(0, 0), xy(2.8, 0))),
        RouteAction("left_90deg_1", "rotate_to", (pose.yaw_rad + math.pi / 2,)),
        RouteAction("forward_420cm", "follow_segment", (xy(2.8, 0), xy(2.8, 4.2))),
        RouteAction("left_90deg_2", "rotate_to", (pose.yaw_rad + math.pi,)),
        RouteAction("forward_250cm", "follow_segment", (xy(2.8, 4.2), xy(0.3, 4.2))),
    )


def run_route(runtime, vision, alarm, *, abort=lambda: False, max_seconds=300,
              clock=time.monotonic, sleep=time.sleep):
    """Monitor vision before each motion tick; cancel/zero before sounding.

    During the one-second alarm, localization continues with the action
    cancelled. Resuming reuses the original segment/absolute turn target.
    """
    deadline = clock() + max_seconds

    def guard():
        if abort():
            raise RuntimeError("route test interrupted")
        if clock() >= deadline:
            raise RuntimeError("route test time limit expired")

    try:
        runtime.start()
        runtime.motion.stop()  # No implicit synthetic/default navigation goal.
        localization_deadline = min(deadline, clock() + 30)
        while True:
            guard()
            if vision.ready:
                break
            if clock() >= localization_deadline:
                raise RuntimeError("YOLO startup timed out")
            _step(runtime)
            sleep(PERIOD_S)
        # Preserve a startup entry for handling before the first motion tick.
        initial_entry = vision.poll(clock())
        while True:
            guard()
            result = _step(runtime)
            if _usable_pose(result) and runtime.mission.state is RobocupMissionState.READY:
                break
            if clock() >= localization_deadline:
                raise RuntimeError("fused localization startup timed out")
            sleep(PERIOD_S)
        actions = route_from_pose(result.estimate.pose)
        alarm_count = 0
        for action in actions:
            action.start(runtime)
            runtime.record_event("test_route_action_start", label=action.label, args=action.args)
            print(f"[route] START {action.label}", flush=True)
            while True:
                guard()
                entry = initial_entry if initial_entry is not None else vision.poll(clock())
                initial_entry = None
                if entry is not None:
                    runtime.motion.stop()
                    runtime.drive.stop()
                    alarm_count += 1
                    runtime.record_event("test_route_target_stop", label=action.label, box=entry)
                    print(f"[route] TARGET STOP; beep 1s ({alarm_count})", flush=True)
                    alarm.on()
                    try:
                        beep_end = clock() + BEEP_S
                        while clock() < beep_end:
                            guard()
                            _step(runtime)  # Cancelled action: zero output, live fusion.
                            sleep(min(PERIOD_S, max(0, beep_end - clock())))
                    finally:
                        alarm.off()
                    # Check vision before resuming, including stale/error checks.
                    initial_entry = vision.poll(clock())
                    action.start(runtime)
                    runtime.record_event("test_route_resume", label=action.label)
                    continue
                result = _step(runtime)
                if runtime.motion.state is MotionActionState.SUCCEEDED:
                    runtime.drive.stop()
                    runtime.record_event("test_route_action_done", label=action.label,
                                         pose=result.estimate.pose)
                    print(f"[route] DONE {action.label}", flush=True)
                    break
                if runtime.motion.state not in {MotionActionState.RUNNING, MotionActionState.POSE_LOST}:
                    raise RuntimeError(f"motion failed: {runtime.motion.state.value}")
                sleep(PERIOD_S)
        runtime.drive.stop()
        runtime.mission.finish()
        runtime.record_event("test_route_finished", alarm_count=alarm_count)
        print(f"[route] FINISHED; alarms={alarm_count}", flush=True)
        return alarm_count
    except BaseException as exc:
        runtime.drive.stop()
        runtime.mission.request_safe_stop(str(exc))
        raise
    finally:
        # Drive stops before worker joins/log close; alarm is silenced on all exits.
        runtime.drive.stop()
        alarm.off()
