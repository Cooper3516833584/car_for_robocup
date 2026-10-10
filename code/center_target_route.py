"""Compatibility imports for the validated route tool; implementation is shared."""

from target_patrol import (
    BEEP_S, MODEL_PATH, PERIOD_S, VISION_MAX_AGE_S, EntryLatch, RouteAction, YoloVision,
    central_target, filter_target_boxes, horizontal_target, pin_vision_worker,
    route_from_pose, run_route, select_fastest_cpus, visible_target,
)
