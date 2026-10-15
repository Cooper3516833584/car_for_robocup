"""2026 competition route: small, independently callable task functions."""

from __future__ import annotations

import logging
import math
from statistics import median
from dataclasses import dataclass, replace
from pathlib import Path
import sys
import time

from components.basic_motion_controller import MotionActionState
from components.fleet_task_sender import send_task_and_start
from components.payload_servos import open_payload, payload_configured
from mission_control import LocalizationLostError, step_runtime as _step, usable_pose as _usable_pose
from components.yellow_yolo_adapter import load_detector, select_yellow
from components.yolo_cpu import CAMERA_FPS, CAMERA_HEIGHT, CAMERA_WIDTH, IMGSZ
from config.v2_runtime import RuntimeMode
from robocup_runtime import RobocupMissionState

# ============================================================
# FIELD / COMPETITION QUICK TUNING
# Distances: m. Angles here: deg, converted at the motion boundary.
# Field-local origin: D500 centre at launch. +X forward, +Y left at launch.
# Capture the fused base_link origin once; transform targets at the boundary.
# ============================================================
CONTROL_DT_S = 0.05
LOCALIZATION_WAIT_S = 30.0
TASK_BOARD_CAMERA = 0
YELLOW_CAMERA = 0
# Continuous target search holds the calibrated servo at +90 deg, facing left.
# Full mission reads the task board at the start with the camera already at +90°.

START = (0.00, 0.00)
CORNER_1 = (7.60, 0.00)
CORNER_2 = (7.60, 7.36)
FINISH = (0.00, 7.36)
ROUTE_POINTS = (START, CORNER_1, CORNER_2, FINISH)
ROUTE_HEADINGS_DEG = (0.0, 90.0, 180.0)
# Standalone stages retain their caller-owned fused-frame interpretation.
# These aliases do not capture an origin at an arbitrary mid-route position.
START_X, START_Y = LANE_ENTRY_X, LANE_ENTRY_Y = TASK_BOARD_X, TASK_BOARD_Y = START
START_YAW_DEG = LANE_ENTRY_YAW_DEG = TASK_BOARD_YAW_DEG = ROUTE_HEADINGS_DEG[0]
CORNER_1_X, CORNER_1_Y = YELLOW_SEARCH_START_X, YELLOW_SEARCH_START_Y = CORNER_1
CORNER_1_YAW_DEG = CROSS_LANE_YAW_DEG = ROUTE_HEADINGS_DEG[1]
CORNER_2_X, CORNER_2_Y = YELLOW_SEARCH_END_X, YELLOW_SEARCH_END_Y = CORNER_2
FINISH_X, FINISH_Y = FINISH
CORNER_2_YAW_DEG = FINISH_YAW_DEG = ROUTE_HEADINGS_DEG[2]

TASK_BOARD_MAX_TRIES = 3
TASK_BOARD_RETRY_DELAY_S = 0.15
TASK_BOARD_FALLBACK = (1, 2, 1)

HC_PORT = "/dev/ttyS4"
HC_BAUDRATE = 115200
# FleetBus uses the BB33 bridge shared with the drone's FC UART2.
HC_BRIDGE_ENVELOPE = True
HC_CONNECT_WAIT_S = 2.0
HC_RESPONSE_TIMEOUT_S = 6.0
HC_COMMAND_RETRIES = 3

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

# ---- CH3 road visual centring, then blind fused drop ------------------------
# Field tuning lives here; pixels never alter the blind approach endpoint.
TARGET_COLOR = "yellow"
SEARCH_REFERENCE_CX = 320.0  # Horizontal camera centre at +90 deg.
DROP_SAFE_M = 0.43       # Operator revised the blind drop reach, 2026-10-11.
DROP_FINE_SPEED_M_S = 0.05
DROP_TERMINAL_SPEED_M_S = 0.012
DROP_TERMINAL_RANGE_M = 0.06
DROP_FUSED_TOLERANCE_M = 0.003
DROP_SETTLE_S = 0.6
DROP_CORRECTION_TRIES = 4
DROP_TRACK_LOOKAHEAD_M = 0.12
DROP_Y_TOL_PX = 10        # historical preview interface, never controls blind drop
DROP_FRAME_TRIES = 3      # frames read per observation before calling it arc-free
# Save a new reference here only after CH3 centring and +/-2 cm Y validation.
# The old packaged photo remains comparison material, never an automatic truth.
DROP_REFERENCE = (Path(__file__).resolve().parents[1] / "assets"
                  / "ch3_yellow_servo0_field_reference.jpg")

# Legacy standalone drop-route tuning; full uses the shared fused payload detour.
# Only ("drive", m), ("rotate", deg), ("rotate_to", deg) are supported.
FIXED_DROP_ROUTE = [("drive", 0.00)]
PAYLOAD_RELEASE_HOLD_S = 0.50
# ===== END FIELD / COMPETITION QUICK TUNING =====

LOG = logging.getLogger(__name__)
STAGES = ("full", "lane", "task-board", "hc-send", "corner1", "cross-lane",
          "yellow-detect", "yellow-search", "yellow-align", "drop-route", "drop",
          "drop-align", "corner2", "finish")


@dataclass(frozen=True)
class DroneAllocation:
    red: int
    blue: int
    green: int

    def as_dict(self):
        return dict(red=self.red, blue=self.blue, green=self.green)


def split_task_allocation(counts):
    """Assign one single-item color to CAR; drone gets the other three items."""
    values = counts.as_dict()
    if sum(values.values()) != 4:
        raise ValueError('source task board must contain four free-color items')
    car_color = next((color for color in ('red', 'blue', 'green') if values[color] == 1), None)
    if car_color is None:
        raise ValueError('no single-item target can be assigned to the car')
    values[car_color] = 0
    return car_color, DroneAllocation(**values)


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
    if (LANE_ENTRY_X, LANE_ENTRY_Y) == (START_X, START_Y):
        return wait_for_fused_localization(runtime)
    return move_to_pose(runtime, LANE_ENTRY_X, LANE_ENTRY_Y, LANE_ENTRY_YAW_DEG)


def go_to_task_board(runtime):
    if (LANE_ENTRY_X, LANE_ENTRY_Y) == (TASK_BOARD_X, TASK_BOARD_Y):
        runtime.drive.stop()
        return read_task_board(runtime=runtime)
    result = track_lane_line(runtime, (LANE_ENTRY_X, LANE_ENTRY_Y),
                                 (TASK_BOARD_X, TASK_BOARD_Y))
    turn_to_deg(runtime, TASK_BOARD_YAW_DEG)
    runtime.drive.stop()
    return result


def read_task_board(camera=None, *, reader=None, runtime=None, allow_fallback=True):
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
    if not allow_fallback:
        raise RuntimeError("actual task-board counts unavailable: " + reason)
    counts = TaskCounts(*TASK_BOARD_FALLBACK)
    LOG.warning("using fallback task %s: %s", counts.as_dict(), reason)
    if runtime is not None:
        runtime.record_event("task_board_counts", **counts.as_dict(), fallback=True, reason=reason)
    return counts


def send_task_to_drone_once(counts, *, runtime=None, start_drone=True):
    """One logical handoff: actual quotas ACK, then matching start ACK."""
    if sum(counts.as_dict().values()) == 4:
        _, counts = split_task_allocation(counts)
    ok = send_task_and_start(counts, port=HC_PORT, baudrate=HC_BAUDRATE,
                            connect_wait_s=HC_CONNECT_WAIT_S,
                            response_timeout_s=HC_RESPONSE_TIMEOUT_S, retries=HC_COMMAND_RETRIES,
                            start_drone=start_drone)
    if runtime is not None:
        runtime.record_event("hc_task_send", **counts.as_dict(), success=ok,
                             protocol="FleetBus", start_confirmed=ok and start_drone,
                             bridge_envelope=HC_BRIDGE_ENVELOPE)
    return ok


def go_to_first_corner(runtime):
    return track_lane_line(runtime, (LANE_ENTRY_X, LANE_ENTRY_Y),
                               (CORNER_1_X, CORNER_1_Y))


def enter_cross_lane(runtime):
    turn_to_deg(runtime, CROSS_LANE_YAW_DEG)
    if (CORNER_1_X, CORNER_1_Y) == (YELLOW_SEARCH_START_X, YELLOW_SEARCH_START_Y):
        return wait_for_fused_localization(runtime)
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
        if box is None:
            return None
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


def _observe_road_target(vision, now, color=None):
    if color is None:
        return vision.observe_drop(now)
    from target_patrol import visible_target, horizontal_target
    stamp, colored = vision.observe_colours(now)
    width, height = vision.frame_size
    boxes = sorted((box for name, box in colored if name == color),
                   key=lambda b: abs((b[0]+b[2])/2-width/2))
    return (stamp, visible_target(boxes, width, height, YELLOW_MIN_CONF),
            horizontal_target(boxes, width, YELLOW_MIN_CONF,
                              center_width_ratio=2*YELLOW_CX_TOL_PX/CAMERA_WIDTH))


def fine_center_on_road(runtime, vision, initial_box, *, color=None):
    """Three fresh +90 YOLO centres, one gain probe, then straight fused moves.

    Pixel direction is measured once in this search and never flipped from a
    noisy error. An unusable observation skips the drop; it is not centred.
    The worker owns the camera throughout; this never opens a second capture.
    """
    runtime.motion.stop()
    runtime.drive.stop()
    origin = wait_for_fused_localization(runtime).estimate.pose
    try:
        last_frame = _observe_road_target(vision, runtime.clock(), color)[0]
    except RuntimeError as exc:
        runtime.record_event("search_road_align_done", success=False, reason=str(exc))
        return None
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
                stamp, box, _ = _observe_road_target(vision, runtime.clock(), color)
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
        runtime.drive.stop()
        runtime.record_event("search_road_align_done", success=False, reason=reason)
        return None

    try:
        before = sample()
        if before is None:
            return fallback("fresh centres unavailable")
        if abs(before[0] - SEARCH_REFERENCE_CX) <= 4:
            runtime.record_event("search_road_align_done", success=True, observed_x=before[0],
                                 reference_x=SEARCH_REFERENCE_CX)
            return before[1]
        before_uncertainty = sample_uncertainty
        # Sampling takes time: search braking/coast must not consume the probe.
        probe_origin = wait_for_fused_localization(runtime).estimate.pose
        _fused_translation(motion, probe_origin, origin.yaw_rad, .02, "search_gain_probe")
        after = sample()
        probe = wait_for_fused_localization(runtime).estimate.pose
        actual_m = ((probe.x_m-probe_origin.x_m)*math.cos(origin.yaw_rad)
                    + (probe.y_m-probe_origin.y_m)*math.sin(origin.yaw_rad))
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
                runtime.record_event("search_road_align_done", success=True, observed_x=cx,
                                     reference_x=SEARCH_REFERENCE_CX)
                return box
            distance = max(-.02, min(.02, -error / gain))
            pose = wait_for_fused_localization(runtime).estimate.pose
            pose = _fused_translation(motion, pose, origin.yaw_rad, distance, "search_road_adjust")
            observation = sample()
            pose = wait_for_fused_localization(runtime).estimate.pose
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
                        color=TARGET_COLOR, safe_distance_m=None):
    """Blind fused approach: track lateral drift, settle at the fixed reach.

    Camera pixels never change the longitudinal endpoint. Pure Pursuit corrects
    cross-track/yaw while advancing; terminal adjustments are parallel to the
    saved side axis and preserve the current lateral coordinate.
    """
    safe_m = DROP_SAFE_M if safe_distance_m is None else safe_distance_m
    if safe_m is None or not math.isfinite(safe_m) or safe_m <= 0:
        raise ValueError("measured DROP_SAFE_M is required before the payload detour")
    start = (side_pose.x_m, side_pose.y_m)
    ux, uy = math.cos(side_yaw), math.sin(side_yaw)
    end = (start[0] + safe_m * ux, start[1] + safe_m * uy)
    original_drive = runtime.motion.drive
    original_nav = runtime.motion.navigation
    tolerance_m = DROP_FUSED_TOLERANCE_M
    terminal_speed_m_s = DROP_TERMINAL_SPEED_M_S

    def observe(pose):
        dx, dy = pose.x_m-start[0], pose.y_m-start[1]
        progress = dx*ux + dy*uy
        remaining = safe_m-progress
        # Slow before the endpoint, avoiding the PP patrol speed floor and
        # leaving the drive's acceleration limiting/watchdog intact.
        cap = terminal_speed_m_s if remaining <= DROP_TERMINAL_RANGE_M else DROP_FINE_SPEED_M_S
        runtime.motion.drive = replace(original_drive, max_linear_speed_m_s=min(
            original_drive.max_linear_speed_m_s, cap))
        runtime.record_event("drop_fused_tracking", yaw=pose.yaw_rad,
                             fused_x=pose.x_m, fused_y=pose.y_m,
                             road_projection=dx*math.cos(road_yaw)+dy*math.sin(road_yaw),
                             side_projection=progress, cross_track_m=ux*dy-uy*dx,
                             remaining_m=remaining, speed_cap_m_s=cap,
                             chosen_action="track_fixed_fused_line")
        return False

    def settled_pose():
        runtime.motion.stop()
        runtime.drive.stop()
        deadline = runtime.clock() + DROP_SETTLE_S
        while runtime.clock() < deadline:
            _step(runtime)  # STOP, localization safety and live fusion remain active.
            time.sleep(min(CONTROL_DT_S, max(0, deadline-runtime.clock())))
        return wait_for_fused_localization(runtime).estimate.pose

    try:
        runtime.motion.navigation = replace(original_nav,
            position_tolerance_m=min(original_nav.position_tolerance_m, .002),
            lookahead_m=min(original_nav.lookahead_m, DROP_TRACK_LOOKAHEAD_M),
            translation_speed_scale=min(original_nav.translation_speed_scale, .4))
        runtime.motion.drive = replace(original_drive, max_linear_speed_m_s=min(
            original_drive.max_linear_speed_m_s, DROP_FINE_SPEED_M_S))
        runtime.record_event("drop_align_start", color=color, mode="blind_fused",
                             safe_m=safe_m, start_xy=start, safe_drop_xy=end,
                             road_yaw_rad=road_yaw, side_yaw_rad=side_yaw,
                             tolerance_m=tolerance_m,
                             pose_reference="fused")
        motion("safe_side_approach", "track_global_line", start, end, on_tick=observe)
        for index in range(DROP_CORRECTION_TRIES + 1):
            pose = settled_pose()
            reach = (pose.x_m-start[0])*ux + (pose.y_m-start[1])*uy
            error = safe_m-reach
            runtime.record_event("drop_fused_settled", attempt=index, side_projection=reach,
                                 error_m=error, target_m=safe_m, pose=pose,
                                 pose_reference="fused")
            if abs(error) <= tolerance_m:
                runtime.record_event("drop_align_done", success=True, source="blind_fused_endpoint",
                                     side_projection=reach, error_m=error,
                                     pose=pose, pose_reference="fused")
                return pose
            if index == DROP_CORRECTION_TRIES:
                raise RuntimeError("fused drop reach did not settle within 3 mm of the fixed endpoint")
            runtime.motion.drive = replace(original_drive, max_linear_speed_m_s=min(
                original_drive.max_linear_speed_m_s, terminal_speed_m_s))
            _fused_translation(motion, pose, side_yaw, error, "drop_fused_distance_adjust")
    finally:
        runtime.motion.drive = original_drive
        runtime.motion.navigation = original_nav
        runtime.drive.stop()


def perform_payload_detour(runtime, *, servo=None, camera=None,
                           color=TARGET_COLOR, safe_distance_m=None, payload_slot=0,
                           simulate_release=False):
    """Turn at the centred stop, side line, reverse, one inward turn."""
    from payload_detour import DetourSettings, run_payload_detour
    if not simulate_release and not payload_configured(payload_slot):
        runtime.record_event('payload_release_failed', actuator='payload_servo', cargo_count=1,
                             slot=payload_slot, color=color,
                             reason='PWM unavailable or not writable')
        runtime.drive.stop()
        runtime.mission.request_safe_stop('payload PWM unavailable')
        raise RuntimeError('payload PWM unavailable or not writable')
    settings = DetourSettings(
        payload_slot=payload_slot,
        approach_m=DROP_SAFE_M if safe_distance_m is None else safe_distance_m,
        release_hold_s=PAYLOAD_RELEASE_HOLD_S,
        patrol_slow_speed_m_s=.08 * runtime.motion.navigation.translation_speed_scale)
    original_drive = runtime.motion.drive
    runtime.motion.drive = replace(original_drive, max_linear_speed_m_s=min(
        original_drive.max_linear_speed_m_s, settings.patrol_slow_speed_m_s))

    def fine_align(side_pose, road_yaw, side_yaw, motion):
        return align_drop_position(runtime, camera, side_pose, road_yaw, side_yaw, motion,
                                   color=color, safe_distance_m=settings.approach_m)

    try:
        return run_payload_detour(runtime, settings, clock=runtime.clock,
                                  sleep=time.sleep, travel_drive=original_drive,
                                  fine_align=fine_align, simulate_release=simulate_release)
    finally:
        runtime.motion.drive = original_drive


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


def _rot(theta, xy):
    c, s = math.cos(theta), math.sin(theta)
    return (c*xy[0]-s*xy[1], s*xy[0]+c*xy[1])


def field_point_to_fused_base(origin_base, local_xy, relative_yaw_deg, d500_mount):
    """Base target whose radar centre reaches local_xy at the requested heading."""
    start_offset = _rot(origin_base.yaw_rad, (d500_mount.x_m, d500_mount.y_m))
    local = _rot(origin_base.yaw_rad, local_xy)
    goal_offset = _rot(origin_base.yaw_rad + math.radians(relative_yaw_deg),
                       (d500_mount.x_m, d500_mount.y_m))
    return (origin_base.x_m+start_offset[0]+local[0]-goal_offset[0],
            origin_base.y_m+start_offset[1]+local[1]-goal_offset[1])


def field_heading_to_fused_deg(origin_base, relative_yaw_deg):
    return math.degrees(origin_base.yaw_rad) + relative_yaw_deg


def _actual_route_pose(runtime):
    pose = wait_for_fused_localization(runtime).estimate.pose
    mount = runtime.config.d500_mount
    offset = _rot(pose.yaw_rad, (mount.x_m, mount.y_m))
    return dict(actual_fused_x=pose.x_m, actual_fused_y=pose.y_m,
                actual_fused_yaw=pose.yaw_rad,
                actual_radar_fused_xy=(pose.x_m+offset[0], pose.y_m+offset[1]))


def patrol_leg_with_pending_targets(runtime, start_xy, end_xy, pending_payloads,
                                    vision, servo, latches, *, simulate_release=False):
    """Resume the same fixed fused line after every colour-specific detour."""
    from target_patrol import find_centered_target
    if vision is None:
        return track_lane_line(runtime, start_xy, end_xy)
    last_frame, visible, centered, selected_color = None, None, None, None

    def observe(now):
        nonlocal last_frame, visible, centered, selected_color
        stamp, colored = vision.observe_colours(now)
        if stamp != last_frame:
            last_frame = stamp
            width, height = vision.frame_size
            from target_patrol import visible_target, horizontal_target
            candidates, visible_boxes = [], []
            for color in pending_payloads:
                boxes = sorted((b for name, b in colored if name == color),
                               key=lambda b: abs((b[0]+b[2])/2-width/2))
                box = visible_target(boxes, width, height, YELLOW_MIN_CONF)
                middle = horizontal_target(boxes, width, YELLOW_MIN_CONF,
                    center_width_ratio=2*YELLOW_CX_TOL_PX/CAMERA_WIDTH)
                latches[color].update(box, trigger=False)
                if box is not None:
                    visible_boxes.append(box)
                if middle is not None and latches[color].armed:
                    candidates.append((abs((middle[0]+middle[2])/2-width/2), color, middle))
            visible = visible_boxes[0] if visible_boxes else None
            centered, selected_color = None, None
            if candidates:
                _, selected_color, centered = min(candidates, key=lambda item: item[:2])
                latches[selected_color].require_clear()
        return stamp, visible, centered

    while pending_payloads:
        box = find_centered_target(runtime, vision, start_xy, end_xy,
            slow_speed_m_s=.08*runtime.motion.navigation.translation_speed_scale,
            clock=runtime.clock, sleep=time.sleep, observe_drop=observe)
        if box is None:
            return wait_for_fused_localization(runtime)
        color = selected_color
        centered = None  # Never retrigger the consumed frame after failed alignment.
        aligned = fine_center_on_road(runtime, vision, box, color=color)
        if aligned is None:
            runtime.record_event("competition_target_skipped", color=color,
                                 reason="road alignment failed; await new clear frames")
            continue
        runtime.record_event("yellow_align", source="continuous_search", color=color,
                             cx_px=(aligned[0]+aligned[2])/2, cy_px=(aligned[1]+aligned[3])/2)
        slot = pending_payloads[color]
        dropped = _stage(runtime, "payload-detour", lambda: perform_payload_detour(
            runtime, servo=servo, color=color, payload_slot=slot,
            simulate_release=simulate_release))
        runtime.record_event("payload_drop", actuator="payload_servo", cargo_count=1,
                             slot=slot, color=color, success=dropped is not None and not simulate_release,
                             skipped=dropped is None or simulate_release, simulated=simulate_release,
                             detour_completed=dropped is not None, pose_reference="fused")
        if dropped is not None:
            del pending_payloads[color]
    return track_lane_line(runtime, start_xy, end_xy)


def run_full_mission(runtime, *, task_board_camera=None, yellow_camera=None, detector=None,
                     color=TARGET_COLOR, send_drone_task=None, car_only_test=False,
                     simulate_release=False):
    from config.v2_factory import build_servo
    from components.servo_positioning import park_servo

    task_camera = TASK_BOARD_CAMERA if task_board_camera is None else task_board_camera
    yellow_camera = YELLOW_CAMERA if yellow_camera is None else yellow_camera
    runtime.drive.stop()
    # The boot service homes at 0 degrees. Turn to the task-board view as soon
    # as the mission process starts, before localization wait or chassis motion.
    servo = build_servo(runtime.config, fake=runtime.mode is not RuntimeMode.HARDWARE_MISSION)
    if servo is None:
        raise RuntimeError("full competition requires the configured camera servo")
    vision = None
    try:
        park_servo(servo, 90)
        try:
            counts = _stage(runtime, "task-board", lambda: read_task_board(
                task_camera, runtime=runtime, allow_fallback=True))
        except Exception:
            runtime.drive.stop()
            runtime.mission.request_safe_stop("actual task-board counts unavailable")
            raise
        if detector is None and runtime.mode is RuntimeMode.HARDWARE_MISSION:
            try:
                detector = load_detector(YELLOW_MODEL_PATH)
            except Exception:
                LOG.exception("yellow model unavailable; skip visual drop")
        origin_base = wait_for_fused_localization(runtime).estimate.pose
        mount = runtime.config.d500_mount
        radar_offset = _rot(origin_base.yaw_rad, (mount.x_m, mount.y_m))
        runtime.record_event("competition_route_origin", fused_base_x=origin_base.x_m,
                             fused_base_y=origin_base.y_m, fused_base_yaw=origin_base.yaw_rad,
                             d500_mount_x=mount.x_m, d500_mount_y=mount.y_m,
                             derived_start_radar_xy=(origin_base.x_m+radar_offset[0],
                                                     origin_base.y_m+radar_offset[1]),
                             route_points_local=ROUTE_POINTS)
        runtime.mission.set_task_counts(counts)
        car_color, drone_counts = split_task_allocation(counts)
        runtime.record_event('payload_target_assignment', car_color=car_color, car_mandatory_yellow=1,
                             car_target_required=1, drone_allocation=drone_counts.as_dict(),
                             drone_mandatory_yellow=1)
        def handoff():
            if car_only_test:
                runtime.record_event("hc_task_send_skipped", **drone_counts.as_dict(),
                                     reason="car full-flow test; drone commands disabled")
                return True
            if send_drone_task is None:
                return send_task_to_drone_once(drone_counts, runtime=runtime)
            ok = send_drone_task(drone_counts)
            runtime.record_event("hc_task_send", **drone_counts.as_dict(), success=ok,
                                 protocol="FleetBus", start_confirmed=ok is True,
                                 bridge_envelope=HC_BRIDGE_ENVELOPE)
            return ok

        if _stage(runtime, "hc-send", handoff) is not True:
            runtime.drive.stop()
            runtime.mission.request_safe_stop("drone task/start was not confirmed")
            raise RuntimeError("drone task/start was not confirmed")
        from target_patrol import YoloVision, EntryLatch
        pending_payloads = {"yellow": 0, car_color: 1}
        latches = {name: EntryLatch() for name in pending_payloads}
        if detector is not None:
            vision = YoloVision(YELLOW_MODEL_PATH, yellow_camera, confidence=YELLOW_MIN_CONF,
                                target_region="frame", detector=detector,
                                drop_center_width_ratio=2*YELLOW_CX_TOL_PX/CAMERA_WIDTH)
            vision.start()
        for index, heading in enumerate(ROUTE_HEADINGS_DEG):
            if index:
                runtime.record_event("competition_route_turn_start", corner=index,
                                     desired_heading_local_deg=heading, **_actual_route_pose(runtime))
                turn_to_deg(runtime, field_heading_to_fused_deg(origin_base, heading))
                runtime.record_event("competition_route_turn_done", corner=index,
                                     desired_heading_local_deg=heading, **_actual_route_pose(runtime))
            a_local, b_local = ROUTE_POINTS[index:index+2]
            a = field_point_to_fused_base(origin_base, a_local, heading, mount)
            b = field_point_to_fused_base(origin_base, b_local, heading, mount)
            fields = dict(leg=index+1, start_local_xy=a_local, end_local_xy=b_local,
                          start_fused_base_xy=a, end_fused_base_xy=b,
                          desired_heading_local_deg=heading)
            runtime.record_event("competition_route_leg_start", **fields, **_actual_route_pose(runtime))
            result = patrol_leg_with_pending_targets(runtime, a, b, pending_payloads,
                vision, servo, latches, simulate_release=simulate_release)
            runtime.record_event("competition_route_leg_done", **fields, **_actual_route_pose(runtime))
        for target_color, slot in pending_payloads.items():
            runtime.record_event("payload_drop", actuator="payload_servo", cargo_count=1,
                                 slot=slot, color=target_color, success=False, skipped=True,
                                 simulated=simulate_release, detour_completed=False,
                                 reason="three-leg route exhausted without completed detour")
        runtime.drive.stop()
        runtime.mission.finish()
        pose = result.estimate.pose
        runtime.record_event("competition_finished", x_m=pose.x_m, y_m=pose.y_m,
                             yaw_rad=pose.yaw_rad, finish_local_xy=ROUTE_POINTS[-1])
        return result
    finally:
        primary_error = sys.exc_info()[0] is not None
        cleanup_error = None
        cleanups = [("drive_stop", runtime.drive.stop)]
        if vision is not None:
            cleanups.append(("vision_close", vision.close))
        if servo.is_running:
            cleanups.append(("servo_close", lambda: servo.close(hold=True)))
        for name, cleanup in cleanups:
            try:
                cleanup()
            except Exception as exc:
                cleanup_error = cleanup_error or exc
                runtime.record_event("competition_cleanup_failed", resource=name, reason=str(exc))
        if cleanup_error is not None and not primary_error:
            raise cleanup_error


def run_competition_stage(runtime, stage="full", *, task_board_camera=None,
                          yellow_camera=None, weights=None, send_drone_task=None,
                          color=TARGET_COLOR, car_only_test=False, simulate_release=False):
    """Thin CLI dispatch. No stage implicitly starts the complete mission."""
    if stage not in STAGES:
        raise ValueError(f"unknown competition stage: {stage}")
    if car_only_test and stage != "full":
        raise ValueError("car full-flow test requires the full mission")
    if stage != "full":
        runtime.record_event("competition_stage_frame", stage=stage,
                             coordinate_frame="caller_owned_fused_base", captures_field_origin=False)
    task_camera = TASK_BOARD_CAMERA if task_board_camera is None else task_board_camera
    yellow_camera = YELLOW_CAMERA if yellow_camera is None else yellow_camera
    detector = None
    if stage in {"yellow-detect", "yellow-search", "yellow-align"}:
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
                                yellow_camera=yellow_camera, detector=detector, send_drone_task=send_drone_task,
                                color=color, car_only_test=car_only_test, simulate_release=simulate_release)
    if stage == "hc-send":
        if runtime.mission.task_counts is None:
            raise RuntimeError("hc-send requires actual task counts")
        return _stage(runtime, stage, lambda: send_task_to_drone_once(
            runtime.mission.task_counts, runtime=runtime, start_drone=False))
    if stage == "task-board":
        # Read-only stage: no navigation to the observation point.
        return _stage(runtime, stage, lambda: read_task_board(task_camera, runtime=runtime))
    if stage == "drop":
        ok = _stage(runtime, stage, lambda: True if simulate_release else open_payload())
        runtime.record_event("payload_drop", actuator="dual_servo",
                             success=ok is not False and not simulate_release, simulated=simulate_release)
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
