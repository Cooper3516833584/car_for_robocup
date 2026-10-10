"""280/420/250 cm route with YOLO payload detours or legacy stop/beep.

Hardware is opened explicitly, never on import. Only the runtime control thread
commands motion/GPIO; the vision worker supplies observations.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from pathlib import Path
import threading
import time
import sys

from components.basic_motion_controller import MotionActionState
from components.payload_task import payload_channel, prepare_payload
from components.yolo_cpu import (CAMERA_FPS, CAMERA_HEIGHT, CAMERA_WIDTH, IMGSZ,
                                 pin_vision_worker, select_fastest_cpus)
from competition_task import _step, _usable_pose
from payload_detour import run_payload_detour
from robocup_runtime import RobocupMissionState

PERIOD_S = 0.05
BEEP_S = 1.0
VISION_MAX_AGE_S = 2.0
MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "best_car.pt"


def visible_target(boxes, width, height, min_conf=0.5, *, region="frame"):
    """Any confident colour target in the selected region of the native frame."""
    if region not in {"frame", "center"}:
        raise ValueError("target region must be frame or center")
    lo, hi = (0.25, 0.75) if region == "center" else (0., 1.)
    for box in boxes:
        x1, y1, x2, y2, confidence, _class_id = box
        if (not all(math.isfinite(float(v)) for v in box[:5])
                or confidence < min_conf or x2 <= x1 or y2 <= y1):
            continue
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        if width * lo <= cx <= width * hi and height * lo <= cy <= height * hi:
            return tuple(box)
    return None


def central_target(boxes, width, height, min_conf=0.5):
    return visible_target(boxes, width, height, min_conf, region="center")


def horizontal_target(boxes, width, min_conf=0.5, *, center_width_ratio=0.1):
    """Trigger by box centre X only; there is no vertical-position gate."""
    half = width * center_width_ratio / 2
    for box in boxes:
        x1, y1, x2, y2, confidence, _class_id = box
        if (all(math.isfinite(float(v)) for v in box[:5]) and confidence >= min_conf
                and x2 > x1 and y2 > y1
                and abs((x1 + x2) / 2 - width / 2) <= half):
            return tuple(box)
    return None


def filter_target_boxes(boxes, names, class_name=None):
    """Resolve by the weights' class names, never assume yellow's numeric ID."""
    if class_name is None:
        return boxes
    mapping = names if isinstance(names, dict) else dict(enumerate(names))
    ids = {int(index) for index, name in mapping.items() if name == class_name}
    if not ids:
        raise ValueError(f"YOLO weights have no class named {class_name!r}")
    return [box for box in boxes if len(box) >= 6 and math.isfinite(float(box[5]))
            and float(box[5]).is_integer() and int(box[5]) in ids]


class EntryLatch:
    """One alarm per occupancy; re-arm after three consecutive clear frames."""

    def __init__(self):
        self.armed = True
        self.clear_frames = 0

    def require_clear(self):
        self.armed = False
        self.clear_frames = 0

    def update(self, target, *, trigger=True):
        if target is None:
            self.clear_frames += 1
            if self.clear_frames >= 3:
                self.armed = True
            return None
        self.clear_frames = 0
        if self.armed and trigger:
            self.armed = False
            return target
        return None


class YoloVision:
    """Latest fresh inference plus a latched entry, even if the next frame clears.

    A camera/inference exception or stale inference fails closed before more
    movement. The worker cannot access the runtime or motor interfaces.
    """

    def __init__(self, weights=MODEL_PATH, camera=0, *, imgsz=IMGSZ, confidence=0.8,
                 target_region="center", drop_center_width_ratio=0.1, target_class_name=None):
        if target_region not in {"frame", "center"}:
            raise ValueError("target region must be frame or center")
        self.weights = Path(weights)
        self.camera = camera
        self.imgsz = imgsz
        self.confidence = confidence
        self.target_region = target_region
        self.target_class_name = target_class_name
        if not math.isfinite(drop_center_width_ratio) or not 0 < drop_center_width_ratio <= 1:
            raise ValueError("drop center width ratio must be in (0, 1]")
        self.drop_center_width_ratio = drop_center_width_ratio
        self._lock = threading.Lock()
        self._quit = threading.Event()
        self._ready = threading.Event()
        self._thread = None
        self._pending = None
        self._frame_at = None
        self._target = None
        self._visible = None
        self._horizontal = None
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

            detector = load_detector(self.weights)
            capture = cv2.VideoCapture(self.camera, cv2.CAP_V4L2)
            if not capture.isOpened():
                raise RuntimeError("YOLO camera cannot open")
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
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
                    print(f"[vision] cpus={cpus}; imgsz={self.imgsz}; "
                          f"steady_threads={torch.get_num_threads()}; confidence={self.confidence:g}; "
                          f"camera={frame.shape[1]}x{frame.shape[0]} "
                          f"fps={capture.get(cv2.CAP_PROP_FPS):g}", flush=True)
                if frames <= 3:
                    print(f"[vision] frame={frames} inference_s={frame_age:.3f}", flush=True)
                # CPU/model cold-start can exceed the freshness limit. Discard
                # that frame while stationary; only a fresh result declares ready.
                if frames == 1 or frame_age > VISION_MAX_AGE_S:
                    continue
                boxes = [] if result.boxes is None else result.boxes.data.cpu().tolist()
                boxes = filter_target_boxes(boxes, result.names, self.target_class_name)
                # Standard detect models expose xyxy, confidence, class in six columns.
                target = visible_target(boxes, frame.shape[1], frame.shape[0], self.confidence,
                                        region=self.target_region)
                visible = visible_target(boxes, frame.shape[1], frame.shape[0], self.confidence)
                horizontal = horizontal_target(boxes, frame.shape[1], self.confidence,
                                               center_width_ratio=self.drop_center_width_ratio)
                entry = latch.update(target)
                with self._lock:
                    self._frame_at = frame_at
                    self._target = target
                    self._visible = visible
                    self._horizontal = horizontal
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

    def _check_fresh(self, now):
        if self._error:
            raise RuntimeError(self._error)
        if self._frame_at is None or now - self._frame_at > VISION_MAX_AGE_S:
            raise RuntimeError("YOLO inference unavailable or stale")

    def observe(self, now):
        """Latest native-frame observation; fresh frames are identified by timestamp."""
        with self._lock:
            self._check_fresh(now)
            return self._frame_at, self._target

    def observe_drop(self, now):
        with self._lock:
            self._check_fresh(now)
            return self._frame_at, self._visible, self._horizontal

    def poll(self, now):
        with self._lock:
            self._check_fresh(now)
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
        RouteAction("forward_280cm", "track_global_line", (xy(0, 0), xy(2.8, 0))),
        RouteAction("left_90deg_1", "rotate_to", (pose.yaw_rad + math.pi / 2,)),
        RouteAction("forward_420cm", "track_global_line", (xy(2.8, 0), xy(2.8, 4.2))),
        RouteAction("left_90deg_2", "rotate_to", (pose.yaw_rad + math.pi,)),
        RouteAction("forward_250cm", "track_global_line", (xy(2.8, 4.2), xy(0.3, 4.2))),
    )


def run_route(runtime, vision, alarm, *, abort=lambda: False, max_seconds=300,
              clock=time.monotonic, sleep=time.sleep, detour=None):
    """Slow on visibility; detour on horizontal centre, then resume fixed endpoints.

    Optional legacy mode retains the one-second stop/alarm behavior.
    """
    deadline = clock() + max_seconds
    original_drive = runtime.motion.drive
    slow_drive = (replace(original_drive, max_linear_speed_m_s=min(
        original_drive.max_linear_speed_m_s, detour.patrol_slow_speed_m_s))
        if detour is not None else original_drive)
    slowing = False

    def guard():
        if abort():
            raise RuntimeError("route test interrupted")
        if clock() >= deadline:
            raise RuntimeError("route test time limit expired")

    try:
        if detour is not None and (runtime.relay is None
                                  or runtime.relay.channel_count < payload_channel(detour.payload_slot)):
            raise RuntimeError("payload relay must be configured before patrol starts")
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
        initial_entry = vision.poll(clock()) if detour is None else None
        if detour is not None:
            vision.observe(clock())
        drop_latch, last_drop_frame = EntryLatch(), None
        while True:
            guard()
            result = _step(runtime)
            if _usable_pose(result) and runtime.mission.state is RobocupMissionState.READY:
                break
            if clock() >= localization_deadline:
                raise RuntimeError("fused localization startup timed out")
            sleep(PERIOD_S)
        actions = route_from_pose(result.estimate.pose)
        if detour is not None:
            if not prepare_payload(runtime.relay, detour.payload_slot, verify=detour.verify_relay):
                raise RuntimeError("payload holding state was not confirmed; patrol will not move")
            runtime.record_event("payload_hold_ready", slot=detour.payload_slot,
                                 channel=payload_channel(detour.payload_slot))
            guard()
            vision.observe(clock())  # Recheck freshness after relay verification.
        alarm_count = 0
        drop_count = 0
        for action in actions:
            action.start(runtime)
            runtime.record_event("test_route_action_start", label=action.label, args=action.args)
            print(f"[route] START {action.label}", flush=True)
            while True:
                guard()
                if detour is not None:
                    frame_at, visible, centered = vision.observe_drop(clock())
                    entry = None
                    target_visible = visible is not None and action.method == "track_global_line"
                    if target_visible != slowing:
                        slowing = target_visible
                        runtime.motion.drive = slow_drive if slowing else original_drive
                        runtime.record_event("test_route_target_speed", slowing=slowing,
                                             max_speed_m_s=runtime.motion.drive.max_linear_speed_m_s)
                    # Only patrol straight segments trigger; turns do not re-arm.
                    if action.method == "track_global_line" and frame_at != last_drop_frame:
                        last_drop_frame = frame_at
                        if drop_latch.update(visible, trigger=centered is not None) is not None:
                            entry = centered
                else:
                    entry = initial_entry if initial_entry is not None else vision.poll(clock())
                initial_entry = None
                if detour is not None and entry is not None:
                    runtime.record_event("test_route_payload_target", label=action.label, box=entry)
                    print(f"[route] PAYLOAD DETOUR slot={detour.payload_slot}", flush=True)
                    run_payload_detour(runtime, detour, guard=guard,
                                       check_vision=lambda: vision.observe(clock()),
                                       clock=clock, sleep=sleep, travel_drive=original_drive)
                    drop_count += 1
                    # Frames seen while turning/dropping cannot re-arm a target.
                    drop_latch.require_clear()
                    last_drop_frame = vision.observe(clock())[0]
                    action.start(runtime)  # Preserve original endpoints; 7cm counts toward patrol.
                    runtime.record_event("test_route_resume", label=action.label,
                                         reason="payload_detour", drop_count=drop_count)
                    continue
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
        runtime.record_event("test_route_finished", alarm_count=alarm_count, drop_count=drop_count)
        print(f"[route] FINISHED; alarms={alarm_count}; drops={drop_count}", flush=True)
        return drop_count if detour is not None else alarm_count
    except BaseException as exc:
        runtime.mission.request_safe_stop(str(exc))
        raise
    finally:
        # Drive stops before worker joins/log close; alarm is silenced on all exits.
        primary_error = sys.exc_info()[0] is not None
        cleanup_error = None
        for cleanup in (runtime.drive.stop, alarm.off if alarm is not None else lambda: None):
            try:
                cleanup()
            except Exception as exc:
                cleanup_error = cleanup_error or exc
                runtime.record_event("test_route_cleanup_failed", reason=str(exc))
                runtime.mission.request_safe_stop(str(exc))
        runtime.motion.drive = original_drive
        if detour is not None and runtime.relay is not None and runtime.relay.connected:
            try:
                if runtime.relay.all_off(verify=detour.verify_relay) is False:
                    raise RuntimeError("payload relay all_off was not confirmed")
            except Exception as exc:
                runtime.mission.request_safe_stop(str(exc))
                runtime.record_event("test_route_cleanup_failed", reason=str(exc))
                cleanup_error = cleanup_error or exc
        if cleanup_error is not None and not primary_error:
            raise cleanup_error
