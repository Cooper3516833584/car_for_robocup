"""2026 competition route: small, independently callable task functions."""

from __future__ import annotations

from datetime import datetime
import logging
import math
from dataclasses import dataclass, replace
from pathlib import Path
import sys
import time

from components.basic_motion_controller import MotionActionState
from components.fleet_task_sender import send_task_and_start
from components.payload_servos import open_payload, payload_configured
from components.payload_drop_log import PayloadDropLog
from components.wall_navigation import (
    FollowRightWall,
    RIGHT_BEARING_RAD,
    REAR_BEARING_RAD,
    WallActionLimits,
    WallTracker,
    can_follow_wall,
    drop_turn,
    is_front_wall,
    is_rear_wall,
    is_right_wall,
)
from mission_control import (
    LocalizationLostError,
    step_runtime as _step,
    step_wall_runtime as _wall_step,
    task_scope as _task_scope,
    usable_pose as _usable_pose,
)
from components.yellow_yolo_adapter import load_detector, select_yellow
from components.yolo_cpu import CAMERA_FPS, CAMERA_HEIGHT, CAMERA_WIDTH, IMGSZ
from config.v2_runtime import RuntimeMode
from robocup_runtime import RobocupMissionState

# ============================================================
# FIELD / COMPETITION QUICK TUNING
# Distances: m. Angles here: deg, converted at the motion boundary.
# ============================================================
#
# ---- Defaults for direct diagnostic calls --------------------------------
# Formal/test run-button settings are at the top of main_robocup.py and
# car_full_flow_test.py and are passed per run without changing these defaults.
# The four values below are user-measured lidar-centre perpendicular distances
# to the field wall, taken at the parallel patrol pose. They are ordered by the
# field diagram: left, bottom, right, top. The internal metre values are the
# lidar-centre distances; ``desired_right_base()`` converts them to
# drive-axis-centre (base_link) targets using the live D500 mount extrinsic, so
# nothing here may be pre-compensated by hand.
#
# 2026-10-16 revision: the operator re-measured every leg wall distance and
# added 20 cm to each of the four original values (140/169.5/149/213 cm), and
# supplied the final H-zone parking distance.
# 2026-10-17: the operator reduced all four patrol distances by 7 cm in total.
WALL_RIGHT_DISTANCE_M = (1.530, 1.825, 1.620, 2.260)
WALL_LEG_NAMES = ("left", "bottom", "right", "top")
# Lidar to top wall at the final upward parking pose.
FINISH_FRONT_DISTANCE_M = 1.10
DROP_SAFE_M = 0.43        # Standard blind reach: drive-axis travel into the target.
RIGHT_MOUNT_DROP_M = 0.32  # Yellow right-mount reach; never overridden by DROP_SAFE_M.
RIGHT_MOUNT_SLOT = 4
YELLOW_PAYLOAD_SLOT = RIGHT_MOUNT_SLOT
# Yellow is the car's mandatory drop and only lights up on the bottom leg.
YELLOW_SEARCH_LEGS = (1,)
PAYLOAD_RELEASE_HOLD_S = 0.50
# Wall-line algorithm starting values. These are package defaults, NOT measured
# field values; keep them here instead of spreading command-line switches.
WALL_LINE_RESIDUAL_M = 0.03
WALL_LINE_MIN_SPAN_M = 0.80
WALL_LINE_MIN_POINTS = 10
WALL_LINE_MAX_RMS_M = 0.03
WALL_MAX_AGE_S = 0.40
CORNER_TOLERANCE_M = 0.02
WALL_DROP_TOLERANCE_M = 0.01

# ---- Legacy fused-route constants ----------------------------------------
# Retained only for the standalone fused diagnostic stages that still exist
# during the migration; the competition chain no longer uses them.
# Field-local origin: D500 centre at launch. +X forward, +Y left at launch.
CONTROL_DT_S = 0.05
LOCALIZATION_WAIT_S = 30.0
TASK_BOARD_CAMERA = 0
YELLOW_CAMERA = 0
# Continuous target search holds the calibrated servo at +90 deg, facing left.
# Full mission reads the task board at the start with the camera already at +90°.

START = (0.00, 0.00)
CORNER_1 = (7.23, 0.00)
CORNER_2 = (7.23, 7.15)
FINISH = (0.00, 7.15)
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

# ---- Central 10% trigger, then fixed blind side-drop ------------------------
# Field tuning lives here; pixels never alter the blind approach endpoint.
TARGET_COLOR = "yellow"
DROP_FINE_SPEED_M_S = 0.15
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

# Legacy standalone drop-route tuning; full uses the shared payload detour.
# Only ("drive", m), ("rotate", deg), ("rotate_to", deg) are supported.
FIXED_DROP_ROUTE = [("drive", 0.00)]
# ===== END FIELD / COMPETITION QUICK TUNING =====

LOG = logging.getLogger(__name__)
STAGES = ("full", "lane", "task-board", "hc-send", "corner1", "cross-lane",
          "yellow-detect", "yellow-search", "yellow-align", "drop-route", "drop",
          "drop-align", "corner2", "finish",
          "wall-observe", "wall-leg", "wall-turn", "drop-standard", "drop-right",
          "top-return", "finish")
# Stage names of the wall chain; the fused diagnostic stages above remain
# individually callable but are not part of any competition run.
WALL_STAGES = ("wall-observe", "wall-leg", "wall-turn", "drop-standard", "drop-right",
               "top-return", "finish")
# How long the mission may hold zero waiting for the vision worker's first real
# frame before calling it a failure.
VISION_READY_TIMEOUT_S = 20.0
WALL_MOTION_STAGES = ("wall-leg", "wall-turn", "drop-standard", "drop-right", "top-return")


@dataclass(frozen=True)
class DroneAllocation:
    red: int
    blue: int
    green: int

    def as_dict(self):
        return dict(red=self.red, blue=self.blue, green=self.green)


def split_task_allocation(counts):
    """Car takes all RGB cargo plus yellow; drone carries implicit yellow only."""
    values = counts.as_dict()
    if sorted(values.values()) != [1, 1, 2]:
        raise ValueError('task board must contain one double-item and two single-item targets (2/1/1)')
    return dict(values), DroneAllocation(red=0, blue=0, green=0)


# Slot usage is a fixed hardware contract, not a per-run decision:
#   4 -> yellow (PWM6/Pin 27, right_mount, 32 cm)
#   2,3 -> the double-item colour, released in one approach
#   1 -> the first single-item colour, 0 -> the second single-item colour
# Both single-item colours use the standard 43 cm behaviour.
def build_payload_plan(counts):
    """The one and only car payload plan. Every slot 0..4 appears exactly once.

    Raises on anything that is not the 2/1/1 task-board shape instead of using
    an ``assert`` that ``python -O`` would remove.
    """
    values = counts.as_dict()
    for color, amount in values.items():
        if isinstance(amount, bool) or not isinstance(amount, int) or amount < 0:
            raise ValueError(f'task-board quota for {color} must be a non-negative int')
    if sorted(values) != ['blue', 'green', 'red'] or sorted(values.values()) != [1, 1, 2]:
        raise ValueError('task board must contain one double-item and two single-item '
                         f'targets (2/1/1), got {values}')
    double = next(color for color in ('red', 'blue', 'green') if values[color] == 2)
    singles = [color for color in ('red', 'blue', 'green') if values[color] == 1]
    plan = {
        'yellow': (YELLOW_PAYLOAD_SLOT,),
        double: (2, 3),
        singles[0]: (1,),
        singles[1]: (0,),
    }
    used = sorted(slot for slots in plan.values() for slot in slots)
    if used != list(range(5)):
        raise ValueError(f'payload plan must use each slot 0..4 once, got {used}')
    return plan


def desired_right_base(leg_index, mount_y_m, *, options=None):
    """Lidar-centre right-wall distance for one leg, as a base_link target.

    The operator measured lidar-centre perpendicular distances at the parallel
    patrol pose; with the right wall normal at ``(0, -1)`` the drive-axis-centre
    target is ``D - mount_y``. Compute this once per leg from the live extrinsic
    instead of re-deriving it from the changing measured normal every frame.
    """
    if isinstance(leg_index, bool) or not isinstance(leg_index, int) or not 0 <= leg_index < 4:
        raise ValueError('leg_index must be 0..3')
    distance = (WALL_RIGHT_DISTANCE_M if options is None else options.wall_right_distance_m)[leg_index]
    if not math.isfinite(mount_y_m):
        raise ValueError('finite mount_y required')
    target = distance - mount_y_m
    if not math.isfinite(target) or target <= 0:
        raise ValueError(f'leg {leg_index} base right target must be positive, got {target}')
    return target


def finish_front_base(mount_x_m, *, options=None):
    """Final parking front-wall base target, or ``None`` while unmeasured."""
    distance = FINISH_FRONT_DISTANCE_M if options is None else options.finish_front_m
    if distance is None:
        return None
    if not math.isfinite(mount_x_m):
        raise ValueError('finite mount_x required')
    target = distance + mount_x_m
    if not math.isfinite(target) or target <= 0:
        raise ValueError(f'finish front base target must be positive, got {target}')
    return target


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
                if sorted(counts.key) != [1, 1, 2]:
                    reason = f"invalid task-board quota shape: {counts.key}, expected 2/1/1"
                    LOG.warning("task-board attempt %d: %s", attempt + 1, reason)
                else:
                    LOG.info("task board: %s", counts.as_dict())
                    if runtime is not None:
                        runtime.record_event("task_board_counts", **counts.as_dict(), fallback=False,
                                             votes=result.votes, confidence=result.confidence)
                    return counts
            else:
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
                           simulate_release=False, payload_slots=(), on_release=None,
                           tracker=None, acquire_reference=None, limits=None,
                           behavior=None):
    """Dispatch the detour: wall feedback for a wall runtime, fused for legacy.

    The 0.43 m standard reach and the fixed 0.32 m right-mount reach are the
    same in both chains; ``safe_distance_m`` can never override the 0.32 m one.
    """
    from payload_detour import DetourSettings, run_payload_detour
    slots = tuple(payload_slots) or (payload_slot,)
    right_mount = behavior == "right_mount" or (
        behavior is None and slots == (RIGHT_MOUNT_SLOT,))
    if not simulate_release and not all(payload_configured(slot) for slot in slots):
        runtime.record_event('payload_release_failed', actuator='payload_servo',
                             cargo_count=len(slots), slot=payload_slot, slots=slots, color=color,
                             reason='PWM unavailable or not writable')
        return None
    options = getattr(runtime, "competition_options", None)
    standard_reach = DROP_SAFE_M if options is None else options.standard_drop_m
    right_reach = RIGHT_MOUNT_DROP_M if options is None else options.right_mount_drop_m
    wall_runtime = getattr(runtime, "observer", None) is not None
    if wall_runtime:
        settings = DetourSettings(
            payload_slot=payload_slot, payload_slots=slots,
            approach_m=right_reach if right_mount else standard_reach,
            behavior="right_mount" if right_mount else "standard",
            release_hold_s=PAYLOAD_RELEASE_HOLD_S,
            position_tolerance_m=WALL_DROP_TOLERANCE_M)
        return run_payload_detour(runtime, settings, clock=runtime.clock, sleep=time.sleep,
                                  tracker=tracker, acquire_reference=acquire_reference,
                                  limits=limits, simulate_release=simulate_release,
                                  on_release=on_release)
    settings = DetourSettings(
        payload_slot=payload_slot, payload_slots=slots,
        approach_m=(right_reach if right_mount else
                    standard_reach if safe_distance_m is None else safe_distance_m),
        behavior="right_mount" if right_mount else "standard",
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
                                  fine_align=fine_align, simulate_release=simulate_release,
                                  on_release=on_release)
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
                                    vision, servo, latches, *, simulate_release=False,
                                    on_release=None):
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
        centered = None  # The triggering frame has been consumed.
        frame_width, _ = vision.frame_size
        runtime.record_event("competition_target_centered", color=color,
                             cx_px=((box[0]+box[2])/2)*CAMERA_WIDTH/frame_width,
                             tolerance_px=YELLOW_CX_TOL_PX, confidence=box[4])
        slots = pending_payloads[color]
        slot = slots[0]
        dropped = _stage(runtime, "payload-detour", lambda: perform_payload_detour(
            runtime, servo=servo, color=color, payload_slot=slot,
            simulate_release=simulate_release, payload_slots=slots,
            on_release=on_release))
        runtime.record_event("payload_drop", actuator="payload_servo", cargo_count=len(slots),
                             slot=slot, slots=slots, color=color, success=dropped is not None and not simulate_release,
                             skipped=dropped is None or simulate_release, simulated=simulate_release,
                             detour_completed=dropped is not None, pose_reference="fused")
        if dropped is not None:
            del pending_payloads[color]
    return track_lane_line(runtime, start_xy, end_xy)


# ===========================================================================
# Wall-following competition chain
#
# Route topology is fixed; positions are not. Leg order is always
# left -> bottom -> right -> top, each leg follows its measured wall distance and
# stops on a reference wall, and the top edge reverses instead of turning
# around. Nothing below reads a fused pose, a map origin or a global line.
# ===========================================================================


@dataclass(frozen=True)
class WallRouteLeg:
    index: int
    name: str
    target_right_m: float
    direction: int
    terminal_role: object | None      # predicate over WallLine, None for no terminal
    terminal_target_m: float | None
    terminal_is_rear: bool
    terminal_name: str
    vision_active: bool
    # The top-edge search stops as soon as its last pending target is released,
    # rather than continuing left until the terminal wall.
    must_finish_on_pending_empty: bool = False


@dataclass(frozen=True)
class MissionResult:
    """Competition outcome. There is deliberately no fused pose in here."""

    route_done: bool
    parking_done: bool
    commanded_release_slots: tuple = ()
    pending_slots: tuple = ()
    simulated: bool = False
    reason: str = ""
    simulated_slots: tuple = ()
    attempted_slots: tuple = ()


def build_wall_route(mount_x_m, mount_y_m, *, options=None):
    """The four wall legs, with every distance converted once from the field set.

    Every leg ends on a real front wall, including the top-edge search: with the
    nose pointing left along the top edge, the map's left perimeter wall is
    *ahead* of the chassis, so it is the same front-wall terminal as the other
    legs. It must not be looked for as a body-left wall, which is a different
    piece of geometry.
    """
    desired = tuple(desired_right_base(index, mount_y_m, options=options) for index in range(4))
    legs = (
        WallRouteLeg(0, "left", desired[0], +1, is_front_wall, desired[1], False,
                     "front_wall", True),
        WallRouteLeg(1, "bottom", desired[1], +1, is_front_wall, desired[2], False,
                     "front_wall", True),
        WallRouteLeg(2, "right", desired[2], +1, is_front_wall, desired[3], False,
                     "front_wall", True),
        WallRouteLeg(3, "top_search", desired[3], +1, is_front_wall, desired[0], False,
                     "front_left_perimeter_wall", True,
                     must_finish_on_pending_empty=True),
    )
    return legs, desired


def wall_drop_reference(runtime, tracker):
    """Lock the leg's road wall as the detour reference and freeze ``d0``."""
    from payload_detour import WallDropReference

    wall = tracker.wall
    if wall is None:
        snapshot = runtime.observer.latest()
        if snapshot is None or not snapshot.is_fresh(float(runtime.clock())):
            return None
        candidates = [line for line in snapshot.lines if is_right_wall(line)]
        if not candidates:
            return None
        wall = tracker.set_reference(min(candidates, key=lambda line: line.rho_base_m))
    runtime.record_event("payload_reference_locked", reference_id="road_right_wall",
                         rho_base_m=wall.rho_base_m, bearing_rad=wall.bearing_rad,
                         span_m=wall.span_m, rms_m=wall.rms_m)
    return WallDropReference(reference_id="road_right_wall", d0_base_m=wall.rho_base_m,
                             bearing_rad=wall.bearing_rad, normal_xy=wall.normal_xy)


def _run_wall_action(runtime, action, phase):
    runtime.mission.phase = phase
    runtime.action = action
    runtime.record_event("wall_action_start", action=action.name, phase=phase)
    try:
        while True:
            _wall_step(runtime)
            outcome = action.outcome
            if outcome is not None and (outcome.done or action.finished):
                runtime.action = None
                runtime.record_event("wall_action_done", action=action.name, phase=phase,
                                     done=outcome.done, reason=outcome.reason)
                return outcome
            time.sleep(CONTROL_DT_S)
    except BaseException:
        runtime.action = None
        raise


def _reacquire_right_wall(runtime, tracker, target_right_m, *, phase):
    """Re-seed the tracked wall after a turn when it is not alongside us yet."""
    wall = tracker.wall
    if wall is not None and is_right_wall(wall):
        return wall
    snapshot = runtime.observer.latest()
    if snapshot is None:
        return None
    candidates = [line for line in snapshot.lines if is_right_wall(line)]
    if not candidates:
        return None
    seeded = min(candidates, key=lambda line: abs(line.rho_base_m - target_right_m))
    tracker.set_reference(seeded)
    runtime.record_event("wall_selection", phase=phase, role="right", selected=True,
                         reference_id="road_right_wall", rejection_reason=None,
                         rho_base_m=seeded.rho_base_m, bearing_rad=seeded.bearing_rad)
    return seeded


def _wait_for_vision_ready(runtime, vision, *, timeout_s: float = VISION_READY_TIMEOUT_S):
    """Hold zero until the vision worker has produced one real result.

    ``YoloVision.start()`` only starts a thread: the first frame is still being
    inferred (and the model deliberately drops an initial frame), so entering the
    first leg immediately would make a normal warm-up look like "inference
    unavailable or stale" and could even let one cruise command reach the motors
    before the camera was usable.
    """
    started = float(runtime.clock())
    runtime.record_event("competition_vision_wait", timeout_s=timeout_s)
    while True:
        try:
            stamp, _colors = vision.observe_colours(runtime.clock())
        except Exception as exc:
            # A real inference failure is not "not ready yet": fail loudly.
            stamp = None
        if stamp is not None:
            runtime.record_event("competition_vision_ready", waited_s=float(runtime.clock()) - started)
            return stamp
        if float(runtime.clock()) - started > timeout_s:
            runtime.record_event("competition_vision_unavailable", continuing=True)
            return False
        runtime.drive.stop()
        runtime.action = None
        _wall_step(runtime)
        time.sleep(CONTROL_DT_S)


def patrol_wall_leg(runtime, leg, tracker, *, pending, vision, latches, limits,
                    command_ledger, simulate_release=False, on_release=None):
    """One wall leg: visual triggering, terminal stop, then the corner turn.

    Returns ``(completed, exhausted)``. ``completed`` means the leg reached its
    terminal reference; ``exhausted`` means the top-edge search ran out of field
    with payload still pending. Both are normal outcomes.
    """
    from target_patrol import horizontal_target, visible_target

    runtime.mission.phase = f"leg_{leg.index + 1}_{leg.name}"
    runtime.record_event("competition_route_leg_start", leg=leg.index + 1, leg_name=leg.name,
                         target_base_right_m=leg.target_right_m, direction=leg.direction,
                         terminal=leg.terminal_name, feedback_source="wall_scan")
    follow = _make_leg_follow(leg, tracker, limits, runtime)
    runtime.action = follow
    not_before = float(runtime.clock())
    last_stamp = None
    completed = False
    exhausted = False
    triggered = None
    terminal_wall = None
    try:
        while True:
            # Requirement order matters: guard and wall/vision are read first, the
            # visible slow-down is applied, and only then is this tick's command
            # computed and sent. The command must never go out at cruise speed
            # and be corrected afterwards.
            _wall_step(runtime)
            snapshot = runtime.observer.latest()
            fresh = snapshot is not None and snapshot.is_fresh(float(runtime.clock()))
            if leg.vision_active and pending and vision is not None and fresh:
                try:
                    stamp, colored = vision.observe_colours(runtime.clock())
                except Exception as exc:
                    stamp, colored = None, ()
                    if follow.speed_cap_m_s is not None:
                        runtime.record_event("vision_unavailable", reason=str(exc), continuing=True)
                    follow.speed_cap_m_s = None
                if stamp is not None and stamp > not_before and stamp != last_stamp:
                    last_stamp = stamp
                    width, height = vision.frame_size
                    centered, visible = _select_centered(
                        colored, _eligible(pending, leg), latches, width, height,
                        visible_target, horizontal_target)
                    # A visible, still-unfinished target on this leg slows the leg
                    # down. The cap is the same cruising profile the fused chain
                    # used; it is a min() against the terminal braking cap, so it
                    # can never make the approach faster.
                    follow.speed_cap_m_s = (limits.vision_speed_m_s if visible
                                            else None)
                    if centered is not None:
                        triggered = centered
            if triggered is not None:
                color, box, slots = triggered
                triggered = None
                follow.speed_cap_m_s = None
                right_before_detour = tracker.wall
                runtime.action = None
                runtime.drive.stop()
                runtime.record_event("competition_target_centered", color=color,
                                     cx_px=(box[0] + box[2]) / 2, tolerance_px=YELLOW_CX_TOL_PX,
                                     confidence=box[4], leg=leg.index + 1)
                dropped = _stage(runtime, "payload-detour", lambda: perform_payload_detour(
                    runtime, tracker=tracker, acquire_reference=wall_drop_reference,
                    limits=limits, color=color, payload_slot=slots[0], payload_slots=slots,
                    simulate_release=simulate_release, on_release=on_release))
                runtime.record_event("payload_drop", actuator="payload_servo",
                                     cargo_count=len(slots), slot=slots[0], slots=slots,
                                     color=color,
                                     success=(not simulate_release and
                                              set(slots).issubset(dropped or ())),
                                     skipped=not dropped or simulate_release,
                                     commanded_slots=() if simulate_release else tuple(dropped or ()),
                                     simulated=simulate_release,
                                     detour_completed=dropped is not None,
                                     feedback_source="wall_scan")
                released_slots = slots if simulate_release and dropped is not None else tuple(dropped or ())
                if released_slots:
                    _mark_dropped(pending, command_ledger, color, released_slots,
                                  simulated=simulate_release)
                _reacquire_right_wall(runtime, tracker, leg.target_right_m,
                                      phase=runtime.mission.phase)
                previous_follow = follow
                follow = _make_leg_follow(leg, tracker, limits, runtime)
                follow.inherit_terminal(previous_follow, right_before_detour)
                runtime.action = follow
                not_before = float(runtime.clock())
                last_stamp = None
                latches[color].require_clear()
                if leg.must_finish_on_pending_empty and not any(pending.values()):
                    completed = True
                    exhausted = False
                    runtime.record_event("top_search_finished_early", leg=leg.index + 1,
                                         pending_colors=())
            # A terminal is only ever the wall the action actually measured.
            if follow.terminal_wall is not None:
                terminal_wall = follow.terminal_wall
            if follow.outcome is not None and follow.outcome.done:
                completed = True
                if pending:
                    exhausted = True
                    runtime.record_event("top_search_exhausted",
                                         measured_base_m=follow.outcome.fields.get(
                                             "terminal_measured_m"),
                                         target_base_m=leg.terminal_target_m,
                                         pending_colors=tuple(sorted(pending)))
            if completed:
                runtime.action = None
                runtime.drive.stop()
                break
            if follow.finished and follow.outcome is not None and not follow.outcome.done:
                raise RuntimeError(f"leg {leg.index + 1}: wall control failed "
                                   f"({follow.outcome.reason})")
            time.sleep(CONTROL_DT_S)
    finally:
        runtime.action = None
    runtime.record_event("competition_route_leg_done", leg=leg.index + 1, leg_name=leg.name,
                         completed=completed, exhausted=exhausted,
                         top_search_exhausted=exhausted,
                         terminal_wall_rho_m=(None if terminal_wall is None
                                              else terminal_wall.rho_base_m))
    return completed, exhausted, terminal_wall


def _make_leg_follow(leg, tracker, limits, runtime):
    """A fresh leg action, so no previous done/terminal state is reused."""
    return FollowRightWall(target_distance_m=leg.target_right_m, direction=leg.direction,
                           limits=limits, tracker=tracker,
                           terminal_role=leg.terminal_role,
                           terminal_target_m=leg.terminal_target_m,
                           terminal_tolerance_m=CORNER_TOLERANCE_M,
                           clock=runtime.clock)


def _eligible(pending, leg):
    """The colours this leg may act on: yellow only on its mandated leg."""
    if leg.index in YELLOW_SEARCH_LEGS:
        return dict(pending)
    return {color: slots for color, slots in pending.items() if color != "yellow"}


def _mark_dropped(pending, ledger, color, slots, *, simulated):
    """Remove the released slots from pending and record them in the ledger."""
    remaining = tuple(slot for slot in pending.get(color, ()) if slot not in slots)
    if remaining:
        pending[color] = remaining
    else:
        pending.pop(color, None)
    ledger.record(color, slots, simulated=simulated)


def _select_centered(colored, pending, latches, width, height, visible_target,
                     horizontal_target):
    """Pick the centred, latched, still-pending target, plus whether any is visible.

    Ordering is nearest to horizontal centre, then colour name. A candidate only
    consumes its latch when the caller actually accepts the drop. The second
    return value reports a visible but not-yet-centred target, which is what the
    caller uses to slow the leg down.
    """
    candidates = []
    any_visible = False
    for color in sorted(pending):
        slots = pending[color]
        boxes = sorted((box for name, box in colored if name == color),
                       key=lambda box: abs((box[0] + box[2]) / 2 - width / 2))
        visible = visible_target(boxes, width, height, YELLOW_MIN_CONF)
        middle = horizontal_target(boxes, width, YELLOW_MIN_CONF,
                                   center_width_ratio=2 * YELLOW_CX_TOL_PX / CAMERA_WIDTH)
        latches[color].update(visible, trigger=False)
        if visible is not None:
            any_visible = True
        if middle is not None and latches[color].armed:
            candidates.append((abs((middle[0] + middle[2]) / 2 - width / 2), color, middle, slots))
    if not candidates:
        return None, any_visible
    _, color, box, slots = min(candidates, key=lambda item: (item[0], item[1]))
    return (color, box, slots), any_visible


class PayloadLedger:
    """One run-wide release ledger, shared by every detour.

    ``pending`` is derived from the plan minus what has been commanded here, so
    an opened slot can never re-enter pending and a detour that failed after the
    first of two releases keeps the first one recorded.
    """

    def __init__(self):
        self.attempted: list = []
        self.commanded: list = []
        self.simulated: list = []

    def record(self, color, slots, *, simulated=False):
        for slot in slots:
            if slot not in self.attempted:
                self.attempted.append(slot)
            if simulated:
                if slot not in self.simulated:
                    self.simulated.append(slot)
            elif slot not in self.commanded:
                self.commanded.append(slot)
        LOG.info("payload ledger: color=%s slots=%s simulated=%s commanded=%s",
                 color, slots, sorted(self.simulated), sorted(self.commanded))

    @property
    def commanded_slots(self):
        return tuple(self.commanded)

    @property
    def handled_slots(self):
        return tuple(self.commanded + self.simulated)


def turn_left_at_corner(runtime, tracker, leg, limits, terminal_wall):
    """Left turn so the wall just in front becomes the right wall of the next leg.

    The reference is the terminal wall the leg actually measured -- not
    ``tracker.wall``, which at the end of a leg is still the *old* road wall.
    With the old wall the bearing would already equal the target and the turn
    would compute a zero angle.
    """
    if terminal_wall is None:
        raise RuntimeError(f"leg {leg.index + 1}: no verified terminal wall to turn on")
    runtime.mission.phase = f"corner_{leg.index + 1}"
    # Hand the next leg the wall that was in front: after the turn it is the
    # right wall, and the tracker must follow that same physical line.
    tracker.set_reference(terminal_wall)
    turn = drop_turn(terminal_wall.bearing_rad, RIGHT_BEARING_RAD, limits=limits,
                     tracker=tracker, turn_direction=+1, clock=runtime.clock,
                     handoff=lambda wall: can_follow_wall(wall, leg.terminal_target_m, limits))
    outcome = _run_wall_action(runtime, turn, runtime.mission.phase)
    if not outcome.done:
        raise RuntimeError(f"leg {leg.index + 1}: corner turn failed ({outcome.reason})")
    return outcome


def reverse_top_to_finish_corner(runtime, tracker, desired, legs, limits):
    """Top edge: keep the nose left and reverse until the rear wall is at leg 3."""
    runtime.mission.phase = "top_reverse"
    back_target = desired[2]
    follow = FollowRightWall(target_distance_m=desired[3], direction=-1, limits=limits,
                             tracker=tracker, terminal_role=is_rear_wall,
                             terminal_target_m=back_target,
                             terminal_tolerance_m=CORNER_TOLERANCE_M, clock=runtime.clock)
    outcome = _run_wall_action(runtime, follow, "top_reverse")
    if not outcome.done:
        raise RuntimeError(f"top reverse did not reach the return corner ({outcome.reason})")
    return follow


def enter_finish_zone(runtime, tracker, desired, limits, terminal_wall):
    """Right turn to face up, then close the front-wall distance into the H zone.

    As at a corner, the reference is the wall the reverse leg just measured. The
    second element of ``terminal_wall`` is the wall astern, which becomes the
    right wall once the chassis has turned to face up.
    """
    target_right = desired[2]
    rear_wall = terminal_wall
    if rear_wall is None:
        raise RuntimeError("finish approach has no verified rear wall to turn on")
    runtime.mission.phase = "finish_turn"
    tracker.set_reference(rear_wall)
    turn = drop_turn(rear_wall.bearing_rad, RIGHT_BEARING_RAD, limits=limits,
                     tracker=tracker, turn_direction=-1, clock=runtime.clock,
                     handoff=lambda wall: can_follow_wall(wall, target_right, limits))
    outcome = _run_wall_action(runtime, turn, "finish_turn")
    if not outcome.done:
        raise RuntimeError(f"final right turn failed ({outcome.reason})")
    _reacquire_right_wall(runtime, tracker, target_right, phase="finish_approach")

    front_target = finish_front_base(runtime.config.d500_mount.x_m,
                                     options=getattr(runtime, "competition_options", None))
    if front_target is None:
        runtime.drive.stop()
        runtime.record_event("finish_parameter_missing", parameter="FINISH_FRONT_DISTANCE_M",
                             at_corner=True, front_target_base_m=None)
        return False
    runtime.mission.phase = "finish_approach"
    follow = FollowRightWall(target_distance_m=target_right, direction=+1, limits=limits,
                             tracker=tracker, terminal_role=is_front_wall,
                             terminal_target_m=front_target,
                             terminal_tolerance_m=CORNER_TOLERANCE_M, clock=runtime.clock)
    outcome = _run_wall_action(runtime, follow, "finish_approach")
    if not outcome.done:
        raise RuntimeError(f"finish approach did not reach the parking distance "
                           f"({outcome.reason})")
    runtime.drive.stop()
    return True


def run_wall_mission(runtime, *, task_board_camera=None, yellow_camera=None, detector=None,
                     color=TARGET_COLOR, simulate_release=False, payload_log_dir=None, fixed_counts=None):
    """The competition chain: four wall legs, then the finish zone."""
    from config.v2_factory import build_servo
    from components.servo_positioning import park_servo

    task_camera = TASK_BOARD_CAMERA if task_board_camera is None else task_board_camera
    yellow_camera = YELLOW_CAMERA if yellow_camera is None else yellow_camera
    runtime.drive.stop()
    servo = build_servo(runtime.config, fake=runtime.mode is not RuntimeMode.HARDWARE_MISSION)
    vision = None
    commanded: list = []
    result = MissionResult(route_done=False, parking_done=False,
                           simulated=simulate_release, reason="")
    try:
        if servo is not None:
            try:
                park_servo(servo, 90)
                runtime.record_event("competition_camera_hold", angle_deg=90, color=color)
            except Exception as exc:
                runtime.record_event("camera_servo_unavailable", reason=str(exc), continuing=True)
        options = getattr(runtime, "competition_options", None)
        fixed_counts = fixed_counts if fixed_counts is not None else getattr(options, "task_counts", None)
        if fixed_counts is None:
            counts = _stage(runtime, "task-board", lambda: read_task_board(
                task_camera, runtime=runtime, allow_fallback=True))
        else:
            from components.task_board_reader import TaskCounts
            counts = TaskCounts(*fixed_counts)
            runtime.record_event("task_board_counts", **counts.as_dict(), fallback=False,
                                 source="fixed_rule", skipped=True)
        plan = build_payload_plan(counts)
        car_counts, drone_counts = split_task_allocation(counts)
        runtime.mission.set_task_counts(counts)
        runtime.record_event("payload_target_assignment", car_allocation=car_counts,
                             car_mandatory_yellow=1, car_target_required=4, car_payload_count=5,
                             drone_allocation=drone_counts.as_dict(),
                             drone_communication="disabled")
        payload_log = (PayloadDropLog(payload_log_dir, counts, car_counts, drone_counts, plan)
                       if payload_log_dir is not None else None)

        def record_release(slot):
            if payload_log is None:
                return
            try:
                payload_log.record_release(slot, datetime.now().astimezone())
            except OSError:
                LOG.exception('payload drop log update failed: %s', payload_log.path)
                runtime.record_event('payload_drop_log_failed', slot=slot)

        if detector is None and runtime.mode is RuntimeMode.HARDWARE_MISSION:
            try:
                detector = load_detector(YELLOW_MODEL_PATH)
            except Exception:
                LOG.exception("yellow model unavailable; skip visual drop")
        mount = runtime.config.d500_mount
        legs, desired = build_wall_route(mount.x_m, mount.y_m, options=getattr(runtime, "competition_options", None))
        runtime.record_event("competition_wall_route", wall_right_distance_m=WALL_RIGHT_DISTANCE_M if options is None else options.wall_right_distance_m,
                             leg_names=WALL_LEG_NAMES,
                             desired_base_right_m=desired,
                             finish_front_distance_m=FINISH_FRONT_DISTANCE_M if options is None else options.finish_front_m,
                             finish_front_base_m=finish_front_base(mount.x_m, options=options),
                             d500_mount_x=mount.x_m, d500_mount_y=mount.y_m,
                             mount_yaw_rad=mount.yaw_rad)
        limits = getattr(runtime, "limits", None) or WallActionLimits.from_config(runtime.config)

        from target_patrol import YoloVision, EntryLatch
        # The ledger lives for the whole run and is the single source of truth for
        # what has actually been released; ``pending`` is derived from it.
        ledger = PayloadLedger()
        pending = {name: tuple(slots) for name, slots in plan.items()}
        latches = {name: EntryLatch() for name in pending}
        if detector is not None:
            vision = YoloVision(YELLOW_MODEL_PATH, yellow_camera, confidence=YELLOW_MIN_CONF,
                                target_region="frame", detector=detector,
                                drop_center_width_ratio=2 * YELLOW_CX_TOL_PX / CAMERA_WIDTH)
            vision.start()
            _wait_for_vision_ready(runtime, vision)

        tracker = WallTracker()
        exhausted = False
        for index, leg in enumerate(legs):
            ok, leg_exhausted, terminal_wall = patrol_wall_leg(
                runtime, leg, tracker, pending=pending, vision=vision, latches=latches,
                limits=limits, command_ledger=ledger,
                simulate_release=simulate_release, on_release=record_release)
            if not ok:
                raise RuntimeError(f"leg {index + 1} ({leg.name}) did not complete")
            exhausted = exhausted or leg_exhausted
            if index < len(legs) - 1:
                # The turn references the wall this leg actually measured, not
                # whatever the tracker still holds from earlier in the leg.
                turn_left_at_corner(runtime, tracker, leg, limits, terminal_wall)
                runtime.record_event("vision_window_reset", not_before_s=float(runtime.clock()))
        result = MissionResult(route_done=True, parking_done=False,
                              commanded_release_slots=ledger.commanded_slots,
                              pending_slots=tuple(slot for slots in pending.values()
                                                  for slot in slots),
                              simulated=simulate_release,
                              simulated_slots=tuple(ledger.simulated),
                              attempted_slots=tuple(ledger.attempted),
                              reason="top_search_exhausted" if exhausted else "")

        # The reverse leg reports the rear wall it measured; the final right turn
        # and the H-zone approach both reference that same wall.
        reverse_follow = reverse_top_to_finish_corner(runtime, tracker, desired, legs, limits)
        parking_done = enter_finish_zone(runtime, tracker, desired, limits,
                                         reverse_follow.terminal_wall)
        result = replace(result, parking_done=parking_done,
                         reason=result.reason if parking_done else "finish_parameter_missing")
        for target_color, slots in pending.items():
            runtime.record_event("payload_drop", actuator="payload_servo",
                                 cargo_count=len(slots), slot=slots[0], slots=slots,
                                 color=target_color, success=False, skipped=True,
                                 simulated=simulate_release, detour_completed=False,
                                 reason="route exhausted without a completed detour")
        runtime.drive.stop()
        if parking_done:
            runtime.mission.finish()
        runtime.record_event("mission_result", route_done=result.route_done,
                             parking_done=result.parking_done,
                             commanded_slots=result.commanded_release_slots,
                             pending_slots=result.pending_slots, simulated=result.simulated,
                             reason=result.reason)
        return result
    finally:
        runtime.action = None
        cleanups = [("drive_stop", runtime.drive.stop)]
        if vision is not None:
            cleanups.append(("vision_close", vision.close))
        if servo is not None and servo.is_running:
            cleanups.append(("servo_close", lambda: servo.close(hold=True)))
        for name, cleanup in cleanups:
            try:
                cleanup()
            except Exception as exc:
                runtime.record_event("competition_cleanup_failed", resource=name, reason=str(exc))


def run_full_mission(runtime, *, task_board_camera=None, yellow_camera=None, detector=None,
                     color=TARGET_COLOR, send_drone_task=None, car_only_test=False,
                     simulate_release=False, payload_log_dir=None):
    """Competition entry: the wall chain for a wall runtime, else the legacy chain."""
    if getattr(runtime, "observer", None) is not None:
        if send_drone_task is not None:
            runtime.record_event("hc_task_send_deprecated",
                                 reason="the car competition chain never contacts the drone")
        return run_wall_mission(runtime, task_board_camera=task_board_camera,
                                yellow_camera=yellow_camera, detector=detector, color=color,
                                simulate_release=simulate_release,
                                payload_log_dir=payload_log_dir,
                                fixed_counts=(1, 2, 1) if car_only_test else None)
    return _run_fused_mission(runtime, task_board_camera=task_board_camera,
                              yellow_camera=yellow_camera, detector=detector, color=color,
                              send_drone_task=send_drone_task, car_only_test=car_only_test,
                              simulate_release=simulate_release,
                              payload_log_dir=payload_log_dir)


def _run_fused_mission(runtime, *, task_board_camera=None, yellow_camera=None, detector=None,
                       color=TARGET_COLOR, send_drone_task=None, car_only_test=False,
                       simulate_release=False, payload_log_dir=None):
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
            car_counts, drone_counts = split_task_allocation(counts)
            plan = build_payload_plan(counts)
        except Exception:
            runtime.drive.stop()
            runtime.mission.request_safe_stop("actual task-board counts unavailable")
            raise
        payload_log = (PayloadDropLog(payload_log_dir, counts, car_counts, drone_counts,
                                      plan)
                       if payload_log_dir is not None else None)

        def record_release(slot):
            if payload_log is None:
                return
            try:
                payload_log.record_release(slot, datetime.now().astimezone())
            except OSError:
                LOG.exception('payload drop log update failed: %s', payload_log.path)
                runtime.record_event('payload_drop_log_failed', slot=slot)

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
        runtime.record_event('payload_target_assignment', car_allocation=car_counts, car_mandatory_yellow=1,
                             car_target_required=4, car_payload_count=5,
                             drone_allocation=drone_counts.as_dict(),
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
        pending_payloads = {color: slots for color, slots in plan.items()}
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
                vision, servo, latches, simulate_release=simulate_release,
                on_release=record_release)
            runtime.record_event("competition_route_leg_done", **fields, **_actual_route_pose(runtime))
        for target_color, slots in pending_payloads.items():
            runtime.record_event("payload_drop", actuator="payload_servo", cargo_count=len(slots),
                                 slot=slots[0], slots=slots, color=target_color, success=False, skipped=True,
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


def run_wall_stage(runtime, stage, *, leg=None, camera=None, detector=None,
                   simulate_release=False, color=TARGET_COLOR):
    """Independently runnable wall stages for field bring-up.

    ``wall-observe`` is read-only hardware: it starts the D500 observer, prints
    right/front/rear candidates and a raw point summary, and constructs neither
    the drive nor a payload output. Every motion stage is explicit
    ``hardware-mission`` and never guesses a global position or runs the full
    mission by itself.
    """
    from components.wall_navigation import WallActionLimits
    from components.wall_observer import WallTracker

    if stage not in WALL_STAGES:
        raise ValueError(f"unknown wall stage: {stage}")
    if stage in {"wall-leg", "wall-turn"} and leg not in (1, 2, 3, 4):
        raise ValueError("wall-leg and wall-turn require --leg 1..4")
    limits = getattr(runtime, "limits", None) or WallActionLimits.from_config(runtime.config)
    legs, desired = build_wall_route(runtime.config.d500_mount.x_m,
                                     runtime.config.d500_mount.y_m)

    if stage == "wall-observe":
        runtime.record_event("wall_observe_start", mode=runtime.mode.value,
                             port=runtime.config.d500.port)
        while True:
            _wall_step(runtime)
            snapshot = runtime.observer.latest()
            if snapshot is None:
                runtime.record_event("wall_observe_empty")
            else:
                roles = {}
                for role, predicate in (("right", is_right_wall), ("front", is_front_wall),
                                        ("rear", is_rear_wall),
                                        ("left", lambda line: line.normal_xy[1] >= 0.5)):
                    candidates = [line for line in snapshot.lines if predicate(line)]
                    if candidates:
                        nearest = min(candidates, key=lambda line: line.rho_base_m)
                        roles[role] = dict(rho_base_m=nearest.rho_base_m,
                                           bearing_rad=nearest.bearing_rad,
                                           span_m=nearest.span_m, rms_m=nearest.rms_m,
                                           inliers=nearest.inlier_count)
                runtime.record_event("wall_observe", scan_seq=snapshot.scan_seq,
                                     point_count=snapshot.point_count,
                                     candidates=len(snapshot.lines), roles=roles,
                                     error=snapshot.error)
            time.sleep(CONTROL_DT_S)

    if stage == "wall-leg":
        target = legs[leg - 1]
        tracker = WallTracker()
        runtime.record_event("wall_stage_leg", leg=leg, leg_name=target.name,
                             target_base_right_m=target.target_right_m,
                             terminal_target_base_m=target.terminal_target_m)
        ledger = PayloadLedger()
        ok, _exhausted, _terminal = patrol_wall_leg(
            runtime, target, tracker, pending={}, vision=None, latches={}, limits=limits,
            command_ledger=ledger, simulate_release=simulate_release, on_release=None)
        return ok
    if stage == "wall-turn":
        # A corner turn needs the wall that leg just measured, so this stage runs
        # that leg first instead of turning on an empty tracker.
        target = legs[leg - 1]
        tracker = WallTracker()
        ledger = PayloadLedger()
        ok, _exhausted, terminal_wall = patrol_wall_leg(
            runtime, target, tracker, pending={}, vision=None, latches={}, limits=limits,
            command_ledger=ledger, simulate_release=simulate_release, on_release=None)
        if not ok:
            return False
        outcome = turn_left_at_corner(runtime, tracker, target, limits, terminal_wall)
        return outcome.done
    if stage == "top-return":
        tracker = WallTracker()
        return reverse_top_to_finish_corner(runtime, tracker, desired, legs, limits).done
    if stage == "finish":
        # The H-zone approach references the wall the reverse leg measured, so the
        # reverse is part of this stage rather than assumed to have happened.
        tracker = WallTracker()
        reverse = reverse_top_to_finish_corner(runtime, tracker, desired, legs, limits)
        return enter_finish_zone(runtime, tracker, desired, limits, reverse.terminal_wall)
    if stage in {"drop-standard", "drop-right"}:
        behavior = "standard" if stage == "drop-standard" else "right_mount"
        slots = (0,) if behavior == "standard" else (RIGHT_MOUNT_SLOT,)
        tracker = WallTracker()
        result = perform_payload_detour(runtime, tracker=tracker,
                                       acquire_reference=wall_drop_reference, limits=limits,
                                       color=color, payload_slot=slots[0], payload_slots=slots,
                                       simulate_release=simulate_release,
                                       behavior=behavior)
        return result is not None
    raise ValueError(f"unhandled wall stage: {stage}")


def run_competition_stage(runtime, stage="full", *, task_board_camera=None,
                          yellow_camera=None, weights=None, send_drone_task=None,
                          color=TARGET_COLOR, car_only_test=False, simulate_release=False,
                          payload_log_dir=None, leg=None):
    """Thin CLI dispatch. No stage implicitly starts the complete mission."""
    if stage not in STAGES:
        raise ValueError(f"unknown competition stage: {stage}")
    if car_only_test and stage != "full":
        raise ValueError("car full-flow test requires the full mission")
    task_camera = TASK_BOARD_CAMERA if task_board_camera is None else task_board_camera
    yellow_camera = YELLOW_CAMERA if yellow_camera is None else yellow_camera
    if stage in WALL_STAGES:
        if not runtime.is_running:
            runtime.start()
        return run_wall_stage(runtime, stage, leg=leg, camera=yellow_camera,
                              simulate_release=simulate_release, color=color)
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
                                color=color, car_only_test=car_only_test, simulate_release=simulate_release,
                                payload_log_dir=payload_log_dir)
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
