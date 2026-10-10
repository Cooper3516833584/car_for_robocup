"""Traditional-vision ring-arc feature for the CH3 close-range drop alignment.

With the camera servo at 0 deg the chassis looks at the yellow outer ring from
roughly 0.4 m. YOLO is deliberately not used here: the packaged 0 deg reference
photograph and every live frame are compared only by the horizontal position and
top height of the ring's outer arc. The gold CH3 baffle in the lower frame and
any filled colour patch are rejected by the arc shape test, so the reference is
never the image centre ``(320, 240)`` nor a template of the whole frame.
"""

from __future__ import annotations

# Every frame is resized to this (width, height) before thresholding, so one set
# of pixel numbers describes the reference photo and all live frames.
IMAGE_SIZE = (640, 480)
# The 0 deg view shows the outer arc in the lower half; the gold baffle is below
# it. These are image segmentation limits, not drop-failure tolerances.
ARC_TOP_PX = 240
ARC_BOTTOM_PX = 441
MIN_ARC_WIDTH_RATIO = 0.27
MIN_ARC_HEIGHT_PX = 20
# A thin ring arc fills very little of its bounding box; a solid patch does not.
MAX_ARC_FILL_RATIO = 0.16
APEX_BAND_PX = 8

# Reserved for the other competition colours; this round only yellow+CH3 has a
# real 0 deg reference photograph, so other colours keep the fused fallback.
HSV_BANDS = {
    "yellow": [((15, 70, 60), (39, 255, 255))],
    "red": [((0, 90, 65), (10, 255, 255)),
            ((170, 90, 65), (179, 255, 255))],
    "blue": [((95, 70, 60), (135, 255, 255))],
    "green": [((40, 70, 50), (90, 255, 255))],
}


def extract_target_arc(frame, color="yellow"):
    """Return ``(apex_x, apex_y, arc_width)`` of the widest thin arc, else None.

    ``apex_x``/``apex_y`` are the pixel position of the highest point of the
    selected outer arc and ``arc_width`` its bounding-box width at the 640x480
    reference size.
    """
    if frame is None or color not in HSV_BANDS:
        return None
    import cv2
    import numpy as np

    image = cv2.resize(frame, IMAGE_SIZE)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros((IMAGE_SIZE[1], IMAGE_SIZE[0]), dtype=np.uint8)
    for low, high in HSV_BANDS[color]:
        mask |= cv2.inRange(hsv, low, high)
    mask[:ARC_TOP_PX, :] = 0
    mask[ARC_BOTTOM_PX:, :] = 0
    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    best = None
    for contour in contours:
        _, _, width, height = cv2.boundingRect(contour)
        if width < MIN_ARC_WIDTH_RATIO * IMAGE_SIZE[0] or height < MIN_ARC_HEIGHT_PX:
            continue
        if cv2.contourArea(contour) / max(1, width * height) > MAX_ARC_FILL_RATIO:
            continue
        if best is None or width > best[0]:
            best = (width, contour)
    if best is None:
        return None
    width, contour = best
    points = contour.reshape(-1, 2)
    top_y = int(points[:, 1].min())
    apex_x = float(np.median(points[points[:, 1] <= top_y + APEX_BAND_PX, 0]))
    return (apex_x, float(top_y), float(width))


def read_reference(path, color="yellow"):
    """Read the packaged 0 deg photograph once and extract its arc feature."""
    import cv2

    image = cv2.imread(str(path))
    return extract_target_arc(image, color)


def reference_error(current, reference):
    """Live minus reference in pixels: the only signal the alignment corrects."""
    return current[0] - reference[0], current[1] - reference[1]
