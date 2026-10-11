"""2026 competition route: small, independently callable task functions."""

from __future__ import annotations

import logging
import math
from statistics import median
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
from mission_control import LocalizationLostError, step_runtime as _step, usable_pose as _usable_pose
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
# Continuous target search holds the calibrated servo at +90 deg, facing left.
# Task-board acquisition still completes before the camera is parked for search.

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
YELLOW_MIN_CONF = 0.8
YELLOW_IMGSZ = IMGSZ
YELLOW_DEVICE = "cpu"
YELLOW_CONFIRM_FRAMES = 2
YELLOW_SEARCH_STEP_M = 0.08
YELLOW_TARGET_CX_PX = CAMERA_WIDTH / 2
YELLOW_CX_TOL_PX = CAMERA_WIDTH * .05  # Verified middle 10%; no vertical gate.
YELLOW_TARGET_CY_PX = CAMERA_HEIGHT / 2  # Observation/logging only.
ALIGN_PIXEL_TO_DRIVE_SIGN = 1
ALIGN_STEP_M = 0.02
ALIGN_MAX_STEPS = 20
# A 2 cm alignment action needs smaller tolerance than the general 3 cm route.
# Relative short moves now construct endpoints in the fused frame.
SHORT_MOVE_TOLERANCE_M = 0.005

# ---- CH3 close-range visual drop alignment (yellow + middle magnet) --------
# Field tuning lives here, not in a config file. No threshold here declares a
# drop failure: every pixel tolerance only says "aligned well enough to score".
TARGET_COLOR = "yellow"
CH3_PAYLOAD_SLOT = 2  # CH3 = middle electromagnet = relay channel 3.
SEARCH_REFERENCE_CX = 320.0  # Calibrate at +90 deg before the 7 cm compensation.
DROP_SAFE_M = 0.47       # Operator-provided safe pivot-to-drop travel, 2026-10-11.
DROP_ADVANCE_M = 0.07    # Keep the existing mechanical compensation until measured.
DROP_FINE_SPEED_M_S = 0.05
DROP_Y_TOL_PX = 10        # success condition only
DROP_FRAME_TRIES = 3      # frames read per observation before calling it arc-free
# Save a new reference here only after CH3 centring and +/-2 cm Y validation.
# The old packaged photo remains comparison material, never an automatic truth.
DROP_REFERENCE = (Path(__file__).resolve().parents[1] / "assets"
                  / "ch3_yellow_servo0_field_reference.jpg")

# Legacy standalone drop-route tuning; full uses the shared fused payload detour.
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
          "drop-align", "corner2", "finish")


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


def run_motion_action(runtime, start_action, *, label, on_tick=None):
    runtime.record_event("competition_stage_start", stage=label)
    try:
        wait_for_fused_localization(runtime)
        if runtime.mission.state is RobocupMissionState.TARGET_OPERATION:
            runtime.mission.on_payload_action_done()
        start_action()
        while True:
            result = _step(runtime)
            if _usable_pose(result) and on_tick is not None and on_tick(result.estimate.pose):
                runtime.motion.stop()
                runtime.drive.stop()
                runtime.record_event("competition_stage_done", stage=label, early_stop=True)
                return _step(runtime)
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


def _fused_motion(runtime):
    """Adapt the routed action runner to the alignment ``motion(label, ...)`` shape."""
    def motion(label, method, *args, **kwargs):
        on_tick = kwargs.pop("on_tick", None)
        result = run_motion_action(runtime,
                                   lambda: getattr(runtime.motion, method)(*args, **kwargs),
                                   label=label, on_tick=on_tick)
        return result.estimate.pose
    return motion


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
    """Relative chassis distance with endpoints in the current fused frame."""
    original = runtime.motion.navigation
    runtime.motion.navigation = replace(original, position_tolerance_m=min(
        original.position_tolerance_m, SHORT_MOVE_TOLERANCE_M))
    def start():
        pose = wait_for_fused_localization(runtime).estimate.pose
        A = (pose.x_m, pose.y_m)
        B = (A[0] + distance_m * math.cos(pose.yaw_rad),
             A[1] + distance_m * math.sin(pose.yaw_rad))
        runtime.motion.track_global_line(A, B, reverse=distance_m < 0)
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


def search_centered_yellow_on_line(runtime, camera, detector, *, servo=None,
                                   color=YELLOW_CLASS_NAME):
    """Reuse tested continuous vision on the competition's own search segment.

    With a borrowed ``servo`` the caller keeps ownership: this function only
    commands the +90 search angle and never closes it. Without one (standalone
    ``yellow-search``) it builds, parks and closes its own axis.
    """
    from target_patrol import YoloVision, find_centered_target
    from config.v2_factory import build_servo
    from components.servo_positioning import park_servo

    if detector is None:
        runtime.drive.stop()
        runtime.record_event("yellow_not_found", reason="detector unavailable")
        return None
    owned_servo = servo is None
    if owned_servo:
        servo = build_servo(runtime.config)
    if servo is None:
        raise RuntimeError("continuous target search requires the calibrated camera servo")
    vision = YoloVision(YELLOW_MODEL_PATH, camera, confidence=YELLOW_MIN_CONF,
                        target_region="frame", target_class_name=color,
                        drop_center_width_ratio=2 * YELLOW_CX_TOL_PX / CAMERA_WIDTH,
                        detector=detector)
    runtime.drive.stop()
    try:
        # park_servo also covers the first PWM export's udev permission delay.
        park_servo(servo, 90)
        runtime.record_event("competition_camera_hold", angle_deg=90, pulse_us=servo.pulse_us,
                             color=color)
        move_to_pose(runtime, YELLOW_SEARCH_START_X, YELLOW_SEARCH_START_Y, CROSS_LANE_YAW_DEG)
        vision.start()
        box = find_centered_target(runtime, vision,
            (YELLOW_SEARCH_START_X, YELLOW_SEARCH_START_Y),
            (YELLOW_SEARCH_END_X, YELLOW_SEARCH_END_Y),
            slow_speed_m_s=.08 * runtime.motion.navigation.translation_speed_scale,
            clock=runtime.clock, sleep=time.sleep)
        if box is None:
            runtime.record_event("yellow_not_found", reason="search segment exhausted")
            return None
        box = fine_center_on_road(runtime, vision, box)
        from components.yellow_yolo_adapter import YellowDetection
        x1,y1,x2,y2,confidence,_ = box
        width,height = vision.frame_size
        sx,sy = CAMERA_WIDTH/width, CAMERA_HEIGHT/height
        return YellowDetection((x1+x2)/2*sx, (y1+y2)/2*sy, (x2-x1)*sx, (y2-y1)*sy, confidence)
    finally:
        primary_error = sys.exc_info()[0] is not None
        cleanup_error = None
        cleanups = [("drive_stop", runtime.drive.stop), ("vision_close", vision.close)]
        if owned_servo:
            cleanups.append(("servo_close",
                             lambda: servo.close(hold=True) if servo.is_running else None))
        for name, cleanup in cleanups:
            try:
                cleanup()
            except Exception as exc:
                cleanup_error = cleanup_error or exc
                runtime.record_event("competition_cleanup_failed", resource=name, reason=str(exc))
        if cleanup_error is not None and not primary_error:
            raise cleanup_error


def fine_center_on_road(runtime, vision, initial_box):
    """Three fresh +90 YOLO centres, one gain probe, then straight fused moves.

    Pixel direction is measured once in this search and never flipped from a
    noisy error. An unusable observation returns to the original trigger stop.
    The worker owns the camera throughout; this never opens a second capture.
    """
    runtime.motion.stop()
    runtime.drive.stop()
    origin = wait_for_fused_localization(runtime).estimate.pose
    try:
        last_frame = vision.observe_drop(runtime.clock())[0]
    except RuntimeError as exc:
        runtime.record_event("search_road_align_done", source="original_search_stop", reason=str(exc))
        return initial_box
    logger_path = getattr(getattr(runtime, "event_logger", None), "path", None)
    evidence_dir = (Path(logger_path).parent / "search_frames"
                    if isinstance(logger_path, (str, Path)) else None)
    saved = 0
    sample_uncertainty = 0.

    def sample():
        nonlocal last_frame, saved, sample_uncertainty
        centres, boxes = [], []
        not_before = runtime.clock()
        deadline = not_before + 3.0
        while runtime.clock() < deadline and len(centres) < 3:
            _step(runtime)  # Includes task STOP/deadline and localization safety.
            try:
                stamp, box, _ = vision.observe_drop(runtime.clock())
            except RuntimeError as exc:
                runtime.record_event("search_road_observation_failed", reason=str(exc))
                return None
            if stamp != last_frame and stamp >= not_before:
                last_frame = stamp
                if box is None:
                    return None
                width, _ = vision.frame_size
                cx = (box[0] + box[2]) / 2 * CAMERA_WIDTH / width
                if not math.isfinite(cx):
                    return None
                centres.append(cx)
                boxes.append(box)
                if evidence_dir is not None:
                    try:
                        vision.save_latest_frame(evidence_dir / f"road_{saved:04d}")
                    except Exception as exc:
                        runtime.record_event("drop_evidence_failed", reason=str(exc))
                    saved += 1
            time.sleep(CONTROL_DT_S)
        if len(centres) != 3 or max(centres) - min(centres) > 12:
            return None
        sample_uncertainty = max(centres)-min(centres)
        pose = wait_for_fused_localization(runtime).estimate.pose
        dx, dy = pose.x_m-origin.x_m, pose.y_m-origin.y_m
        _, height = vision.frame_size
        runtime.record_event("search_road_observation", yaw=pose.yaw_rad,
                             fused_x=pose.x_m, fused_y=pose.y_m,
                             road_projection=dx*math.cos(origin.yaw_rad)+dy*math.sin(origin.yaw_rad),
                             side_projection=-dx*math.sin(origin.yaw_rad)+dy*math.cos(origin.yaw_rad),
                             observed_x=median(centres),
                             observed_y=median((b[1]+b[3])/2*CAMERA_HEIGHT/height for b in boxes),
                             reference_x=SEARCH_REFERENCE_CX, reference_y=None,
                             chosen_action="road_observe", frame_timestamp=last_frame)
        return median(centres), boxes[centres.index(median(centres))]

    original_nav = runtime.motion.navigation
    original_drive = runtime.motion.drive
    runtime.motion.navigation = replace(original_nav, position_tolerance_m=min(
        original_nav.position_tolerance_m, SHORT_MOVE_TOLERANCE_M))
    runtime.motion.drive = replace(original_drive, max_linear_speed_m_s=min(
        original_drive.max_linear_speed_m_s, DROP_FINE_SPEED_M_S))
    motion = _fused_motion(runtime)

    def fallback(reason):
        current = wait_for_fused_localization(runtime).estimate.pose
        offset = ((origin.x_m-current.x_m)*math.cos(origin.yaw_rad)
                  + (origin.y_m-current.y_m)*math.sin(origin.yaw_rad))
        _fused_translation(motion, current, origin.yaw_rad, offset, "search_restore_stop")
        runtime.record_event("search_road_align_done", source="original_search_stop", reason=reason)
        return initial_box

    try:
        before = sample()
        if before is None:
            return fallback("fresh centres unavailable")
        if abs(before[0] - SEARCH_REFERENCE_CX) <= 4:
            return before[1]
        before_uncertainty = sample_uncertainty
        probe = _fused_translation(motion, origin, origin.yaw_rad, .02, "search_gain_probe")
        after = sample()
        actual_m = ((probe.x_m-origin.x_m)*math.cos(origin.yaw_rad)
                    + (probe.y_m-origin.y_m)*math.sin(origin.yaw_rad))
        if (after is None or actual_m <= .005 or
                abs(after[0]-before[0]) < max(4,before_uncertainty+sample_uncertainty)):
            return fallback("pixel gain unidentifiable")
        gain = (after[0]-before[0]) / actual_m
        runtime.record_event("search_road_gain", gain_px_per_m=gain, probe_m=actual_m,
                             cx_before=before[0], cx_after=after[0])
        observation, pose = after, probe
        for index in range(8):
            cx, box = observation
            error = cx - SEARCH_REFERENCE_CX
            runtime.record_event("search_road_align_step", step=index, observed_x=cx,
                                 reference_x=SEARCH_REFERENCE_CX, gain_px_per_m=gain,
                                 yaw=pose.yaw_rad, fused_x=pose.x_m, fused_y=pose.y_m,
                                 chosen_action="hold" if abs(error) <= 4 else "road_straight")
            if abs(error) <= 4:
                return box
            distance = max(-.02, min(.02, -error / gain))
            pose = _fused_translation(motion, pose, origin.yaw_rad, distance, "search_road_adjust")
            observation = sample()
            if observation is None:
                return fallback("target unstable during road correction")
        return fallback("bounded road correction exhausted")
    finally:
        runtime.motion.navigation = original_nav
        runtime.motion.drive = original_drive
        runtime.drive.stop()


def align_yellow_drop_zone(runtime, camera, detector, initial_detection=None):
    runtime.drive.stop()
    if detector is None:
        return None
    if initial_detection is not None and abs(initial_detection.cx_px - YELLOW_TARGET_CX_PX) <= YELLOW_CX_TOL_PX:
        runtime.record_event("yellow_align", success=True, cx_px=initial_detection.cx_px,
                             cy_px=initial_detection.cy_px, source="continuous_search")
        return initial_detection
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


def _fused_translation(motion, pose, direction_yaw, distance_m, label):
    """One fused translation from ``pose`` along ``direction_yaw``.

    The endpoint lives in the saved fused frame and the Pure Pursuit controller
    owns stopping, so no speed-times-time estimate is ever substituted for a
    distance. A negative distance drives backwards, keeping the chassis facing
    the target instead of pivoting away from it.
    """
    if abs(distance_m) < 1e-9:
        return pose
    start = (pose.x_m, pose.y_m)
    end = (start[0] + distance_m * math.cos(direction_yaw),
           start[1] + distance_m * math.sin(direction_yaw))
    return motion(label, "track_global_line", start, end, reverse=distance_m < 0)


def align_drop_position(runtime, camera, side_pose, road_yaw, side_yaw, motion, *,
                        color=TARGET_COLOR, safe_distance_m=None, enable_visual=True):
    """One side-axis approach; stable Y may stop early, X is logging only.

    A new field reference is opt-in by its presence after +/-2 cm validation.
    Without it (or with unstable/lost vision), finish at the measured fused end.
    No rotation, arbitrary XY navigation or per-frame line restart occurs here.
    """
    from components import drop_target_vision as vision

    safe_m = DROP_SAFE_M if safe_distance_m is None else safe_distance_m
    if safe_m is None or not math.isfinite(safe_m) or safe_m <= 0:
        raise ValueError("measured DROP_SAFE_M is required before the payload detour")
    start = (side_pose.x_m, side_pose.y_m)
    end = (start[0] + safe_m * math.cos(side_yaw),
           start[1] + safe_m * math.sin(side_yaw))
    reference = None
    if enable_visual and color == TARGET_COLOR and DROP_REFERENCE.is_file():
        try:
            reference = vision.read_reference(DROP_REFERENCE, color)
        except Exception as exc:
            runtime.record_event("drop_reference_failed", reason=str(exc))
    capture, owned = None, False
    history, stopped = [], False
    last_observation = float('-inf')
    original_drive = runtime.motion.drive
    logger = getattr(runtime, "event_logger", None)
    logger_path = getattr(logger, "path", None)
    evidence_dir = (Path(logger_path).parent / "drop_frames"
                    if isinstance(logger_path, (str, Path)) else None)
    observation_index = 0

    def observe(pose):
        nonlocal stopped, last_observation, observation_index
        if capture is None:
            return False
        now = runtime.clock()
        if now - last_observation < .15:
            return False
        last_observation = now
        try:
            ok, frame = capture.read()
            arc = vision.extract_target_arc(frame, color) if ok else None
        except Exception as exc:
            runtime.record_event("drop_align_camera_failed", reason=str(exc))
            arc, frame = None, None
        history.append(arc)
        del history[:-DROP_FRAME_TRIES]
        stable = vision.stable_feature(history, count=DROP_FRAME_TRIES)
        ex, ey = ((None, None) if stable is None or reference is None
                  else vision.reference_error(stable, reference))
        same_span = (stable is not None and reference is not None and
                     abs(stable[2]-reference[2]) <= max(20,reference[2]*.15))
        stopped = ey is not None and same_span and abs(ey) <= DROP_Y_TOL_PX
        action = "visual_y_stop" if stopped else "continue_safe_line"
        dx, dy = pose.x_m-start[0], pose.y_m-start[1]
        runtime.record_event("drop_align_observation", yaw=pose.yaw_rad,
                             fused_x=pose.x_m, fused_y=pose.y_m,
                             road_projection=dx*math.cos(road_yaw)+dy*math.sin(road_yaw),
                             side_projection=dx*math.cos(side_yaw)+dy*math.sin(side_yaw),
                             observed_x=None if stable is None else stable[0],
                             observed_y=None if stable is None else stable[1],
                             span=None if stable is None else stable[2],
                             reference_x=None if reference is None else reference[0],
                             reference_y=None if reference is None else reference[1],
                             error_x_px=ex, error_y_px=ey, chosen_action=action)
        if frame is not None and evidence_dir is not None:
            try:
                vision.save_observation(frame, evidence_dir / f"side_{observation_index:04d}", color)
            except Exception as exc:
                runtime.record_event("drop_evidence_failed", reason=str(exc))
        observation_index += 1
        return stopped

    try:
        runtime.motion.drive = replace(original_drive, max_linear_speed_m_s=min(
            original_drive.max_linear_speed_m_s, DROP_FINE_SPEED_M_S))
        # The search worker has already released the single camera.
        if enable_visual and color == TARGET_COLOR:
            try:
                capture, owned = _open_yellow_camera(YELLOW_CAMERA if camera is None else camera)
            except Exception as exc:
                runtime.record_event("drop_align_camera_failed", reason=str(exc))
        runtime.record_event("drop_align_start", color=color, reference=reference,
                             safe_m=safe_m, start_xy=start, safe_drop_xy=end,
                             road_yaw_rad=road_yaw, side_yaw_rad=side_yaw,
                             pose_reference="fused")
        pose = motion("safe_side_approach", "track_global_line", start, end, on_tick=observe)
        runtime.record_event("drop_align_done", success=stopped,
                             source="visual_y" if stopped else "fused_safe_endpoint",
                             pose=pose, pose_reference="fused")
        return pose
    finally:
        runtime.motion.drive = original_drive
        if owned and capture is not None:
            capture.release()


def perform_payload_detour(runtime, slot=CH3_PAYLOAD_SLOT, *, servo=None, camera=None,
                           color=TARGET_COLOR, safe_distance_m=None):
    """Road advance, one outward turn, side line, reverse, one inward turn."""
    from payload_detour import DetourSettings, run_payload_detour
    settings = DetourSettings(
        payload_slot=slot, advance_m=DROP_ADVANCE_M,
        approach_m=DROP_SAFE_M if safe_distance_m is None else safe_distance_m,
        release_hold_s=PAYLOAD_RELEASE_HOLD_S,
        verify_relay=runtime.config.relay.verify_writes,
        patrol_slow_speed_m_s=.08 * runtime.motion.navigation.translation_speed_scale)
    original_drive = runtime.motion.drive
    runtime.motion.drive = replace(original_drive, max_linear_speed_m_s=min(
        original_drive.max_linear_speed_m_s, settings.patrol_slow_speed_m_s))

    def fine_align(side_pose, road_yaw, side_yaw, motion):
        # The detour calls this right after the left turn: only now does the
        # camera look forward at the target, so 0 deg is set here.
        if servo is not None:
            servo.set_angle(0, settle=True)
            runtime.record_event("competition_camera_hold", angle_deg=0, pulse_us=servo.pulse_us)
        return align_drop_position(runtime, camera, side_pose, road_yaw, side_yaw, motion,
                                   color=color, safe_distance_m=settings.approach_m,
                                   enable_visual=slot == CH3_PAYLOAD_SLOT)

    try:
        return run_payload_detour(runtime, settings, clock=runtime.clock,
                                  sleep=time.sleep, travel_drive=original_drive,
                                  fine_align=fine_align)
    finally:
        runtime.motion.drive = original_drive
        if servo is not None and servo.is_running:
            try:
                servo.set_angle(90, settle=True)
                runtime.record_event("competition_camera_hold", angle_deg=90, pulse_us=servo.pulse_us)
            except Exception as exc:
                LOG.warning("camera servo did not return to the search angle: %s", exc)
                runtime.record_event("competition_cleanup_failed", resource="servo_search_angle",
                                     reason=str(exc))


def run_drop_align(runtime, camera, *, color=TARGET_COLOR):
    """Stationary 0 deg observation only; the operator may already be at the edge."""
    from config.v2_factory import build_servo
    from components.servo_positioning import park_servo
    from components import drop_target_vision as vision

    runtime.motion.stop()
    runtime.drive.stop()
    servo = build_servo(runtime.config)
    if servo is None:
        raise RuntimeError("drop-align requires the calibrated camera servo")
    capture, owned = None, False
    try:
        park_servo(servo, 0)
        capture, owned = _open_yellow_camera(YELLOW_CAMERA if camera is None else camera)
        observations = []
        for _ in range(DROP_FRAME_TRIES):
            ok, frame = capture.read()
            observations.append(vision.extract_target_arc(frame, color) if ok else None)
        feature = vision.stable_feature(observations, count=DROP_FRAME_TRIES)
        runtime.record_event("drop_align_preview", feature=feature, chosen_action="observe_only")
        return feature
    finally:
        if owned and capture is not None:
            capture.release()
        servo.close(hold=True)


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


def run_full_mission(runtime, *, task_board_camera=None, yellow_camera=None, detector=None,
                     slot=CH3_PAYLOAD_SLOT, color=TARGET_COLOR):
    from config.v2_factory import build_servo

    task_camera = TASK_BOARD_CAMERA if task_board_camera is None else task_board_camera
    yellow_camera = YELLOW_CAMERA if yellow_camera is None else yellow_camera
    wait_for_fused_localization(runtime)
    channel = payload_channel(slot, PAYLOAD_SLOT_TO_RELAY)
    if runtime.relay is None:
        runtime.drive.stop()
        runtime.mission.request_safe_stop("payload relay unavailable")
        raise RuntimeError("payload relay unavailable; mission will not move")
    if runtime.relay is not None:
        if not prepare_payload(runtime.relay, slot, slot_to_relay=PAYLOAD_SLOT_TO_RELAY,
                               active_on=PAYLOAD_ACTIVE_ON, verify=PAYLOAD_VERIFY_RELAY):
            runtime.drive.stop()
            runtime.mission.request_safe_stop("payload holding state was not confirmed")
            raise RuntimeError("payload holding state was not confirmed; mission will not move")
        runtime.record_event("payload_hold_ready", slot=slot, channel=channel)
    # One axis serves the +90 search, the 0 deg refinement and the +90 restore;
    # the search borrows it and must not close it.
    servo = build_servo(runtime.config)
    try:
        _stage(runtime, "lane", lambda: go_to_lane(runtime))
        _stage(runtime, "task-board-position", lambda: go_to_task_board(runtime))
        counts = _stage(runtime, "task-board", lambda: read_task_board(task_camera, runtime=runtime))
        runtime.mission.set_task_counts(counts)
        _stage(runtime, "hc-send", lambda: send_task_to_drone_once(counts, runtime=runtime))
        _stage(runtime, "corner1", lambda: go_to_first_corner(runtime))
        _stage(runtime, "cross-lane", lambda: enter_cross_lane(runtime))
        detection = _stage(runtime, "yellow-search", lambda: search_centered_yellow_on_line(
            runtime, yellow_camera, detector, servo=servo, color=color))
        aligned = None if detection is None else _stage(runtime, "yellow-align", lambda: detection)
        if aligned is not None:
            runtime.record_event("yellow_align", source="continuous_search",
                                 cx_px=aligned.cx_px, cy_px=aligned.cy_px)
        if aligned is not None:
            _stage(runtime, "payload-detour", lambda: perform_payload_detour(
                runtime, slot, servo=servo, camera=yellow_camera, color=color))
            runtime.record_event("payload_drop", slot=slot, channel=channel, success=True,
                                 pose_reference="fused")
        else:
            runtime.record_event("payload_drop", slot=slot, channel=channel, success=False,
                                 skipped=True, reason="yellow search or alignment failed")
        _stage(runtime, "corner2", lambda: go_to_second_corner(runtime))
        return _stage(runtime, "finish", lambda: go_to_finish(runtime))
    finally:
        if servo is not None and servo.is_running:
            try:
                servo.close(hold=True)
            except Exception as exc:
                LOG.exception("camera servo close failed")
                runtime.record_event("competition_cleanup_failed", resource="servo_close",
                                     reason=str(exc))


def run_competition_stage(runtime, stage="full", *, task_board_camera=None,
                          yellow_camera=None, weights=None, slot=CH3_PAYLOAD_SLOT,
                          color=TARGET_COLOR):
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
                                yellow_camera=yellow_camera, detector=detector, slot=slot,
                                color=color)
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
        "yellow-search": lambda: search_centered_yellow_on_line(runtime, yellow_camera, detector),
        "yellow-align": lambda: align_yellow_drop_zone(runtime, yellow_camera, detector),
        "drop-route": lambda: run_fixed_drop_route(runtime),
        "drop-align": lambda: run_drop_align(runtime, yellow_camera, color=color),
        "corner2": lambda: go_to_second_corner(runtime),
        "finish": lambda: go_to_finish(runtime),
    }
    return _stage(runtime, stage, actions[stage])
