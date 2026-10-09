"""2026 competition route: small, independently callable task functions."""

from __future__ import annotations

import logging
import math
from dataclasses import replace
from pathlib import Path
import sys
import time

from components.basic_motion_controller import MotionActionState
from components.hc_task_sender import build_task_message, send_task_once
from components.payload_task import drop_payload as release_payload
from components.payload_task import (PAYLOAD_ACTIVE_ON as DEFAULT_PAYLOAD_ACTIVE_ON,
                                     PAYLOAD_SLOT_TO_RELAY as DEFAULT_PAYLOAD_SLOT_TO_RELAY,
                                     payload_channel, prepare_payload)
from components.pose_fusion import PoseFusionState
from components.yellow_yolo_adapter import load_detector, select_yellow
from components.yolo_cpu import CAMERA_FPS, CAMERA_HEIGHT, CAMERA_WIDTH, IMGSZ
from robocup_runtime import RobocupMissionState

# ============================================================
# FIELD / COMPETITION QUICK TUNING
# TODO(field): all route coordinates below are UNMEASURED placeholders.
# Distances: m. Angles here: deg, converted at the motion boundary.
# Coordinates are the existing runtime fused pose frame; no new transform.
# ============================================================
CONTROL_DT_S = 0.05
LOCALIZATION_WAIT_S = 30.0
TASK_BOARD_CAMERA = 0
YELLOW_CAMERA = 0
# Servo 90 deg points the camera left. No servo control in this task.
# TODO(field): add servo scanning only if an outside marker is out of view.

# START_* are field notes only.
# All route coordinates below use the current fused pose frame directly.
# No START-relative transform is applied.
START_X = 0.0
START_Y = 0.0
START_YAW_DEG = 0.0
LANE_ENTRY_X = 0.0
LANE_ENTRY_Y = 0.0
LANE_ENTRY_YAW_DEG = 0.0
TASK_BOARD_X = 0.0
TASK_BOARD_Y = 0.0
TASK_BOARD_YAW_DEG = 0.0
CORNER_1_X = 0.0
CORNER_1_Y = 0.0
CORNER_1_YAW_DEG = 0.0
CROSS_LANE_YAW_DEG = 0.0
YELLOW_SEARCH_START_X = 0.0
YELLOW_SEARCH_START_Y = 0.0
YELLOW_SEARCH_END_X = 0.0
YELLOW_SEARCH_END_Y = 0.0
CORNER_2_X = 0.0
CORNER_2_Y = 0.0
CORNER_2_YAW_DEG = 0.0
FINISH_X = 0.0
FINISH_Y = 0.0
FINISH_YAW_DEG = 0.0  # Reserved; current finish does not use yaw.

TASK_BOARD_MAX_TRIES = 3
TASK_BOARD_RETRY_DELAY_S = 0.15
TASK_BOARD_FALLBACK = (1, 2, 1)

HC_PORT = "/dev/ttyS4"
HC_BAUDRATE = 115200
# Confirmed by the operator on 2026-10-07: raw ASCII, 115200.
HC_BRIDGE_ENVELOPE = False
HC_TASK_MESSAGE_TEMPLATE = "TASK,{red},{blue},{green}\n"
HC_CONNECT_WAIT_S = 2.0

# Use the same existing, deployed weights as the measured route test.
# The model is not trained or downloaded by this program.
YELLOW_MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "best_car.pt"
YELLOW_CLASS_NAME = "yellow"  # target_yolo/config.py: red, blue, green, yellow
YELLOW_MIN_CONF = 0.55
YELLOW_IMGSZ = IMGSZ
YELLOW_DEVICE = "cpu"
YELLOW_CONFIRM_FRAMES = 2
YELLOW_SEARCH_STEP_M = 0.08
YELLOW_TARGET_CX_PX = CAMERA_WIDTH / 2
YELLOW_CX_TOL_PX = 10  # Same horizontal fraction as 20 px at the old 1280 width.
YELLOW_TARGET_CY_PX = CAMERA_HEIGHT / 2  # Observation/logging only.
ALIGN_PIXEL_TO_DRIVE_SIGN = 1
ALIGN_STEP_M = 0.02
ALIGN_MAX_STEPS = 20
# The existing general route tolerance is 3 cm; a 2 cm alignment action needs
# a smaller tolerance. Applied to move_local_distance only and restored afterwards.
SHORT_MOVE_TOLERANCE_M = 0.005

# TODO(field): measure this sequence from the aligned camera pose to release.
# Only ("drive", m), ("rotate", deg), ("rotate_to", deg) are supported.
FIXED_DROP_ROUTE = [("drive", 0.00)]
PAYLOAD_SLOT_TO_RELAY = dict(DEFAULT_PAYLOAD_SLOT_TO_RELAY)  # 1=右前CH2，2=中间CH3，3=左前CH4。
PAYLOAD_ACTIVE_ON = DEFAULT_PAYLOAD_ACTIVE_ON  # False: 通电吸住，断电释放后保持 OFF。
PAYLOAD_RELEASE_HOLD_S = 0.50
PAYLOAD_VERIFY_RELAY = True
# ===== END FIELD / COMPETITION QUICK TUNING =====

LOG = logging.getLogger(__name__)
STAGES = ("full", "lane", "task-board", "hc-send", "corner1", "cross-lane",
          "yellow-detect", "yellow-search", "yellow-align", "drop-route", "drop",
          "corner2", "finish")


class LocalizationLostError(RuntimeError):
    """Fused localization cannot support further motion."""


def _usable_pose(result):
    estimate = result.estimate
    return (estimate.pose is not None and estimate.state not in {
        PoseFusionState.LOST, PoseFusionState.UNANCHORED, PoseFusionState.INITIALIZING})


def _step(runtime):
    result = runtime.step()
    if result.error or runtime.mission.state in {
            RobocupMissionState.SAFE_STOP, RobocupMissionState.ERROR,
            RobocupMissionState.FINISHED}:
        reason = runtime.mission.last_error or result.error or runtime.mission.state.value
        if not _usable_pose(result):
            raise LocalizationLostError(reason)
        raise RuntimeError(reason)  # Preserve all existing runtime safety paths.
    return result


def wait_for_fused_localization(runtime):
    if not runtime.is_running:
        runtime.start()
    # Suppress the runtime's unrelated synthetic dry-run goal.
    if runtime.motion.state is MotionActionState.IDLE:
        runtime.motion.stop()
    started = float(runtime.clock())
    while True:
        result = _step(runtime)
        if _usable_pose(result):
            return result
        if float(runtime.clock()) - started >= LOCALIZATION_WAIT_S:
            runtime.drive.stop()
            runtime.mission.request_safe_stop("competition fused localization unavailable")
            raise LocalizationLostError("fused localization wait expired")
        time.sleep(CONTROL_DT_S)


def get_current_pose(runtime):
    if not runtime.is_running:
        runtime.start()
    if runtime.motion.state is MotionActionState.IDLE:
        runtime.motion.stop()
    result = _step(runtime)
    return result.estimate.pose if _usable_pose(result) else None


def run_motion_action(runtime, start_action, *, label):
    runtime.record_event("competition_stage_start", stage=label)
    try:
        wait_for_fused_localization(runtime)
        if runtime.mission.state is RobocupMissionState.TARGET_OPERATION:
            runtime.mission.on_payload_action_done()
        start_action()
        while True:
            result = _step(runtime)
            if runtime.motion.state is MotionActionState.SUCCEEDED:
                runtime.drive.stop()
                runtime.record_event("competition_stage_done", stage=label)
                return result
            # Pose dropout is timed and stopped by the existing runtime.
            if runtime.motion.state not in {MotionActionState.RUNNING, MotionActionState.POSE_LOST}:
                raise RuntimeError(f"{label}: motion {runtime.motion.state.value}")
            time.sleep(CONTROL_DT_S)
    except BaseException:
        runtime.drive.stop()
        raise


def move_to_pose(runtime, x_m, y_m, yaw_deg):
    return run_motion_action(runtime, lambda: runtime.motion.navigate_to_pose(
        x_m, y_m, math.radians(yaw_deg)), label="move_to_pose")


def move_to_xy(runtime, x_m, y_m):
    return run_motion_action(runtime, lambda: runtime.motion.navigate_to(
        x_m, y_m), label="move_to_xy")


def track_lane_line(runtime, start_xy, end_xy):
    return run_motion_action(runtime, lambda: runtime.motion.track_global_line(
        start_xy, end_xy), label="track_lane_line")


def turn_to_deg(runtime, yaw_deg):
    return run_motion_action(runtime, lambda: runtime.motion.rotate_to(
        math.radians(yaw_deg)), label="turn_to_deg")


def move_local_distance(runtime, distance_m):
    original = runtime.motion.navigation
    runtime.motion.navigation = replace(original, position_tolerance_m=min(
        original.position_tolerance_m, SHORT_MOVE_TOLERANCE_M))
    def start():
        pose = runtime.current_local_pose()
        if pose is None:
            raise LocalizationLostError("fresh continuous T265 local pose unavailable")
        A = (pose.x_m, pose.y_m)
        B = (A[0] + distance_m * math.cos(pose.yaw_rad),
             A[1] + distance_m * math.sin(pose.yaw_rad))
        runtime.motion.track_local_line(A, B, reverse=distance_m < 0)
    try:
        if not math.isfinite(distance_m):
            raise ValueError("local distance must be finite")
        if distance_m == 0:
            runtime.drive.stop()
            return wait_for_fused_localization(runtime)
        return run_motion_action(runtime, start, label="move_local_distance")
    finally:
        runtime.motion.navigation = original


def go_to_lane(runtime):
    return move_to_pose(runtime, LANE_ENTRY_X, LANE_ENTRY_Y, LANE_ENTRY_YAW_DEG)


def go_to_task_board(runtime):
    # TASK_BOARD_X/Y is the on-lane observation pose, not the board centre.
    result = track_lane_line(runtime, (LANE_ENTRY_X, LANE_ENTRY_Y),
                                 (TASK_BOARD_X, TASK_BOARD_Y))
    turn_to_deg(runtime, TASK_BOARD_YAW_DEG)
    runtime.drive.stop()
    return result


def read_task_board(camera=None, *, reader=None, runtime=None):
    from components.task_board_reader import TaskBoardReader, TaskCounts
    camera = TASK_BOARD_CAMERA if camera is None else camera
    if runtime is not None:
        runtime.drive.stop()
    reason = "no task-board result"
    for attempt in range(TASK_BOARD_MAX_TRIES):
        try:
            if reader is None:
                reader = TaskBoardReader()
            result = reader.recognize_camera(camera)
            if result.valid and result.votes >= reader.config.required_consensus_votes:
                counts = result.counts
                LOG.info("task board: %s", counts.as_dict())
                if runtime is not None:
                    runtime.record_event("task_board_counts", **counts.as_dict(), fallback=False,
                                         votes=result.votes, confidence=result.confidence)
                return counts
            reason = result.reason or "task-board consensus not reached"
        except Exception as exc:
            reason = str(exc)
            LOG.warning("task-board attempt %d: %s", attempt + 1, reason)
        if attempt + 1 < TASK_BOARD_MAX_TRIES:
            time.sleep(TASK_BOARD_RETRY_DELAY_S)
    counts = TaskCounts(*TASK_BOARD_FALLBACK)
    LOG.warning("using fallback task %s: %s", counts.as_dict(), reason)
    if runtime is not None:
        runtime.record_event("task_board_counts", **counts.as_dict(), fallback=True, reason=reason)
    return counts


def send_task_to_drone_once(counts, *, runtime=None):
    ok = send_task_once(counts, port=HC_PORT, baudrate=HC_BAUDRATE,
                        bridge_envelope=HC_BRIDGE_ENVELOPE,
                        connect_wait_s=HC_CONNECT_WAIT_S, template=HC_TASK_MESSAGE_TEMPLATE)
    if runtime is not None:
        try:
            message = build_task_message(counts, HC_TASK_MESSAGE_TEMPLATE).decode("ascii")
        except Exception:
            message = None
        runtime.record_event("hc_task_send", **counts.as_dict(), success=ok,
                             message=message,
                             bridge_envelope=HC_BRIDGE_ENVELOPE)
    return ok


def go_to_first_corner(runtime):
    return track_lane_line(runtime, (TASK_BOARD_X, TASK_BOARD_Y),
                               (CORNER_1_X, CORNER_1_Y))


def enter_cross_lane(runtime):
    turn_to_deg(runtime, CROSS_LANE_YAW_DEG)
    return track_lane_line(runtime, (CORNER_1_X, CORNER_1_Y),
                               (YELLOW_SEARCH_START_X, YELLOW_SEARCH_START_Y))


def detect_yellow_once(camera, detector):
    """Read a stopped-camera frame and use the existing model's predict API."""
    try:
        ok, frame = camera.read()
        if not ok or frame is None:
            return None
        results = detector.predict(frame, imgsz=YELLOW_IMGSZ, conf=YELLOW_MIN_CONF,
                                   device=YELLOW_DEVICE, verbose=False)
        detection = select_yellow(results, class_name=YELLOW_CLASS_NAME, min_conf=YELLOW_MIN_CONF)
        if detection is None:
            return None
        # Drivers/external captures can return a different size. Ultralytics
        # boxes are native-frame pixels; normalize to the tuning reference.
        height, width = frame.shape[:2]
        sx, sy = CAMERA_WIDTH / width, CAMERA_HEIGHT / height
        return replace(detection, cx_px=detection.cx_px * sx, cy_px=detection.cy_px * sy,
                       width_px=detection.width_px * sx, height_px=detection.height_px * sy)
    except Exception:
        LOG.exception("yellow camera/inference failed")
        return None


def _open_yellow_camera(camera):
    if hasattr(camera, "read"):
        return camera, False
    import cv2
    backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
    capture = cv2.VideoCapture(camera, backend)
    try:
        if not capture.isOpened():
            raise RuntimeError(f"cannot open yellow camera {camera!r}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(6):
            capture.read()
        return capture, True
    except BaseException:
        capture.release()
        raise


def detect_yellow_from_camera(camera, detector):
    """One stationary detection; no runtime or motion, release owned capture."""
    capture, owned = _open_yellow_camera(YELLOW_CAMERA if camera is None else camera)
    try:
        return detect_yellow_once(capture, detector)
    finally:
        if owned:
            capture.release()


def _detect_stopped(runtime, camera, detector):
    runtime.drive.stop()
    detection = detect_yellow_once(camera, detector)
    # Refresh queued sensor observations after possibly slow inference.
    wait_for_fused_localization(runtime)
    runtime.record_event("yellow_detect", found=detection is not None,
                         **({} if detection is None else vars(detection)))
    return detection


def search_yellow_drop_zone(runtime, camera, detector):
    """Bounded search from the measured start to end, observing while stopped."""
    runtime.drive.stop()
    if detector is None:
        runtime.record_event("yellow_not_found", reason="detector unavailable")
        return None
    capture, owned = None, False
    try:
        try:
            capture, owned = _open_yellow_camera(camera)
        except Exception as exc:
            runtime.record_event("yellow_not_found", reason=str(exc))
            return None
        move_to_pose(runtime, YELLOW_SEARCH_START_X, YELLOW_SEARCH_START_Y, CROSS_LANE_YAW_DEG)
        start = (YELLOW_SEARCH_START_X, YELLOW_SEARCH_START_Y)
        end = (YELLOW_SEARCH_END_X, YELLOW_SEARCH_END_Y)
        length = math.dist(start, end)
        if YELLOW_SEARCH_STEP_M <= 0 or YELLOW_CONFIRM_FRAMES < 1:
            raise ValueError("yellow search step and confirmation count must be positive")
        travelled = 0.0
        for _ in range(math.ceil(length / YELLOW_SEARCH_STEP_M) + 2):
            confirmed = None
            for _ in range(YELLOW_CONFIRM_FRAMES):
                confirmed = _detect_stopped(runtime, capture, detector)
                if confirmed is None:
                    break
            if confirmed is not None:
                return confirmed
            pose = wait_for_fused_localization(runtime).estimate.pose
            progress = length if length == 0 else (
                (pose.x_m - start[0]) * (end[0] - start[0])
                + (pose.y_m - start[1]) * (end[1] - start[1])) / length
            remaining = min(length - progress, length - travelled)
            if remaining <= 0.01:
                break
            distance = min(YELLOW_SEARCH_STEP_M, remaining)
            move_local_distance(runtime, distance)
            travelled += distance
        runtime.record_event("yellow_not_found", reason="search segment exhausted")
        return None
    finally:
        runtime.drive.stop()
        if owned:
            capture.release()


def align_yellow_drop_zone(runtime, camera, detector, initial_detection=None):
    runtime.drive.stop()
    if detector is None:
        return None
    capture, owned = None, False
    try:
        try:
            capture, owned = _open_yellow_camera(camera)
        except Exception as exc:
            runtime.record_event("yellow_align", success=False, reason=str(exc))
            return None
        detection = initial_detection
        for index in range(ALIGN_MAX_STEPS + 1):
            if detection is None:
                for _ in range(3):
                    detection = _detect_stopped(runtime, capture, detector)
                    if detection is not None:
                        break
            if detection is None:
                runtime.record_event("yellow_align", success=False, reason="target lost")
                return None
            error = detection.cx_px - YELLOW_TARGET_CX_PX
            runtime.record_event("yellow_align", step=index, error_px=error,
                                 cx_px=detection.cx_px, cy_px=detection.cy_px,
                                 cy_error_px=detection.cy_px - YELLOW_TARGET_CY_PX,
                                 success=abs(error) <= YELLOW_CX_TOL_PX)
            if abs(error) <= YELLOW_CX_TOL_PX:
                return detection
            if index == ALIGN_MAX_STEPS:
                break
            distance = math.copysign(ALIGN_STEP_M, error) * ALIGN_PIXEL_TO_DRIVE_SIGN
            move_local_distance(runtime, distance)
            detection = None
        runtime.record_event("yellow_align", success=False, reason="step limit reached")
        return None
    finally:
        runtime.drive.stop()
        if owned:
            capture.release()


def run_fixed_drop_route(runtime):
    result = None
    for index, (action, value) in enumerate(FIXED_DROP_ROUTE):
        runtime.record_event("fixed_drop_route_step", step=index, action=action, value=value)
        if action == "drive":
            result = move_local_distance(runtime, value)
        elif action == "rotate_to":
            result = turn_to_deg(runtime, value)
        elif action == "rotate":
            result = run_motion_action(runtime, lambda: runtime.motion.rotate(
                math.radians(value)), label="rotate")
        else:
            raise ValueError(f"unsupported fixed drop action: {action}")
    return result


def drop_payload(relay, slot=1):
    return release_payload(relay, slot, slot_to_relay=PAYLOAD_SLOT_TO_RELAY,
                           active_on=PAYLOAD_ACTIVE_ON, hold_s=PAYLOAD_RELEASE_HOLD_S,
                           verify=PAYLOAD_VERIFY_RELAY)


def go_to_second_corner(runtime):
    return track_lane_line(runtime, (YELLOW_SEARCH_START_X, YELLOW_SEARCH_START_Y),
                               (CORNER_2_X, CORNER_2_Y))


def go_to_finish(runtime):
    result = track_lane_line(runtime, (CORNER_2_X, CORNER_2_Y),
                                 (FINISH_X, FINISH_Y))
    runtime.drive.stop()
    runtime.mission.finish()
    runtime.record_event("competition_finished", x_m=result.estimate.pose.x_m,
                         y_m=result.estimate.pose.y_m, yaw_rad=result.estimate.pose.yaw_rad)
    return result


def _stage(runtime, name, action):
    LOG.info("competition stage: %s", name)
    runtime.record_event("competition_stage_start", stage=name)
    result = action()
    runtime.record_event("competition_stage_done", stage=name)
    return result


def run_full_mission(runtime, *, task_board_camera=None, yellow_camera=None, detector=None, slot=1):
    task_camera = TASK_BOARD_CAMERA if task_board_camera is None else task_board_camera
    yellow_camera = YELLOW_CAMERA if yellow_camera is None else yellow_camera
    wait_for_fused_localization(runtime)
    channel = payload_channel(slot, PAYLOAD_SLOT_TO_RELAY)
    if runtime.relay is not None:
        if not prepare_payload(runtime.relay, slot, slot_to_relay=PAYLOAD_SLOT_TO_RELAY,
                               active_on=PAYLOAD_ACTIVE_ON, verify=PAYLOAD_VERIFY_RELAY):
            runtime.drive.stop()
            runtime.mission.request_safe_stop("payload holding state was not confirmed")
            raise RuntimeError("payload holding state was not confirmed; mission will not move")
        runtime.record_event("payload_hold_ready", slot=slot, channel=channel)
    _stage(runtime, "lane", lambda: go_to_lane(runtime))
    _stage(runtime, "task-board-position", lambda: go_to_task_board(runtime))
    counts = _stage(runtime, "task-board", lambda: read_task_board(task_camera, runtime=runtime))
    runtime.mission.set_task_counts(counts)
    _stage(runtime, "hc-send", lambda: send_task_to_drone_once(counts, runtime=runtime))
    _stage(runtime, "corner1", lambda: go_to_first_corner(runtime))
    _stage(runtime, "cross-lane", lambda: enter_cross_lane(runtime))
    detection = _stage(runtime, "yellow-search", lambda: search_yellow_drop_zone(
        runtime, yellow_camera, detector))
    aligned = None if detection is None else _stage(runtime, "yellow-align", lambda: align_yellow_drop_zone(
        runtime, yellow_camera, detector, initial_detection=detection))
    if aligned is not None:
        _stage(runtime, "drop-route", lambda: run_fixed_drop_route(runtime))
        ok = _stage(runtime, "drop", lambda: drop_payload(runtime.relay, slot))
        runtime.record_event("payload_drop", slot=slot, channel=channel, success=ok)
        if not ok:
            runtime.drive.stop()
            runtime.mission.request_safe_stop("payload release was not confirmed")
            raise RuntimeError("payload release was not confirmed; mission stopped")
    else:
        runtime.record_event("payload_drop", slot=slot, channel=channel, success=False, skipped=True,
                             reason="yellow search or alignment failed")
    _stage(runtime, "corner2", lambda: go_to_second_corner(runtime))
    return _stage(runtime, "finish", lambda: go_to_finish(runtime))


def run_competition_stage(runtime, stage="full", *, task_board_camera=None,
                          yellow_camera=None, weights=None, slot=1):
    """Thin CLI dispatch. No stage implicitly starts the complete mission."""
    if stage not in STAGES:
        raise ValueError(f"unknown competition stage: {stage}")
    task_camera = TASK_BOARD_CAMERA if task_board_camera is None else task_board_camera
    yellow_camera = YELLOW_CAMERA if yellow_camera is None else yellow_camera
    detector = None
    if stage in {"full", "yellow-detect", "yellow-search", "yellow-align"}:
        # Loading is outside any moving action, including on repeated calls.
        runtime.drive.stop()
        try:
            detector = load_detector(YELLOW_MODEL_PATH if weights is None else weights)
        except Exception:
            LOG.exception("yellow model unavailable; skip visual drop")
    if not runtime.is_running:
        runtime.start()
    runtime.drive.stop()
    if stage == "full":
        return run_full_mission(runtime, task_board_camera=task_camera,
                                yellow_camera=yellow_camera, detector=detector, slot=slot)
    if stage == "hc-send":
        from components.task_board_reader import TaskCounts
        return _stage(runtime, stage, lambda: send_task_to_drone_once(
            runtime.mission.task_counts or TaskCounts(*TASK_BOARD_FALLBACK), runtime=runtime))
    if stage == "task-board":
        # Read-only stage: no navigation to the observation point.
        return _stage(runtime, stage, lambda: read_task_board(task_camera, runtime=runtime))
    if stage == "drop":
        ok = _stage(runtime, stage, lambda: drop_payload(runtime.relay, slot))
        runtime.record_event("payload_drop", slot=slot, success=ok)
        return ok
    if stage == "yellow-detect":
        if detector is None:
            return None
        try:
            detection = _stage(runtime, stage, lambda: detect_yellow_from_camera(yellow_camera, detector))
        except Exception:
            LOG.exception("yellow camera unavailable")
            return None
        runtime.record_event("yellow_detect", found=detection is not None,
                             **({} if detection is None else vars(detection)))
        return detection
    actions = {
        "lane": lambda: go_to_lane(runtime),
        "corner1": lambda: go_to_first_corner(runtime),
        "cross-lane": lambda: enter_cross_lane(runtime),
        "yellow-search": lambda: search_yellow_drop_zone(runtime, yellow_camera, detector),
        "yellow-align": lambda: align_yellow_drop_zone(runtime, yellow_camera, detector),
        "drop-route": lambda: run_fixed_drop_route(runtime),
        "corner2": lambda: go_to_second_corner(runtime),
        "finish": lambda: go_to_finish(runtime),
    }
    return _stage(runtime, stage, actions[stage])
