"""HSV and whole-arc quadratic fit for stationary/side-axis CH3 observations.

At servo 0 deg, reference and live frames use identical HSV segmentation and
whole-arc fitting. The lower baffle and solid patches are rejected. Feature X
is diagnostic; only repeated, stable feature Y can stop a side-line approach.
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
# A thin ring arc fills very little of its bounding box; a solid patch does not.
MAX_ARC_FILL_RATIO = 0.16

# Reserved for the other competition colours; this round only yellow+CH3 has a
# real 0 deg reference photograph, so other colours keep the fused fallback.
HSV_BANDS = {
    "yellow": [((15, 70, 60), (39, 255, 255))],
    "red": [((0, 90, 65), (10, 255, 255)),
            ((170, 90, 65), (179, 255, 255))],
    "blue": [((95, 70, 60), (135, 255, 255))],
    "green": [((40, 70, 50), (90, 255, 255))],
}


def target_mask(frame, color="yellow"):
    """Fixed-size HSV mask with the lower metal baffle excluded."""
    import cv2
    import numpy as np
    image = cv2.resize(frame, IMAGE_SIZE)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros((IMAGE_SIZE[1], IMAGE_SIZE[0]), dtype=np.uint8)
    for low, high in HSV_BANDS.get(color, []):
        mask |= cv2.inRange(hsv, low, high)
    mask[:ARC_TOP_PX] = 0
    mask[ARC_BOTTOM_PX:] = 0
    return mask


def _fit_arc(frame, color):
    import cv2
    import numpy as np
    if frame is None or color not in HSV_BANDS:
        return None
    mask = target_mask(frame, color)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    thin = np.zeros_like(mask)
    for contour in contours:
        _, _, width, height = cv2.boundingRect(contour)
        # Keep fragments too, so a local occlusion need not change the model.
        if width < 15 or cv2.contourArea(contour) / max(1, width*height) > MAX_ARC_FILL_RATIO:
            continue
        cv2.drawContours(thin, [contour], -1, 255, cv2.FILLED)
    ys, xs = np.nonzero(thin)
    if len(xs) < 80:
        return None
    columns = np.unique(xs)
    tops = np.array([ys[xs == x].min() for x in columns], dtype=float)
    # Fit the sufficiently long upper portion, away from lower limbs/metal.
    upper = tops <= tops.min() + 90
    x, y = columns[upper].astype(float), tops[upper]
    if len(x) < 80 or np.ptp(x) < MIN_ARC_WIDTH_RATIO*IMAGE_SIZE[0]:
        return None
    keep = np.ones(len(x), dtype=bool)
    for _ in range(3):
        model = np.polyfit(x[keep]-320, y[keep], 2)
        residual = np.abs(np.polyval(model, x-320)-y)
        keep = residual <= 5
        if keep.sum() < .7*len(x):
            return None
    a, b, c = np.polyfit(x[keep]-320, y[keep], 2)
    if not .0002 <= a <= .02:
        return None
    apex_x = 320-b/(2*a)
    apex_y = c-b*b/(4*a)
    span = float(np.ptp(x[keep]))
    if (span < MIN_ARC_WIDTH_RATIO*IMAGE_SIZE[0]
            or not x[keep].min()+35 <= apex_x <= x[keep].max()-35
            or not ARC_TOP_PX <= apex_y < ARC_BOTTOM_PX
            or np.sqrt(np.mean((np.polyval((a,b,c), x[keep]-320)-y[keep])**2)) > 3.5):
        return None
    return (float(apex_x), float(apex_y), span), (a,b,c), x[keep]


def extract_target_arc(frame, color="yellow"):
    """Return whole-model (feature_x, feature_y, visible_span), or None."""
    fit = _fit_arc(frame, color)
    return None if fit is None else fit[0]


def stable_feature(observations, *, count=3):
    """Median of distinct successive frames only when the same arc repeats."""
    import numpy as np
    if len(observations) < count or any(v is None for v in observations[-count:]):
        return None
    values = np.asarray(observations[-count:], dtype=float)
    if values.shape != (count, 3) or not np.isfinite(values).all():
        return None
    centre = np.median(values, axis=0)
    # Quality rejection, never a task failure or a movement instruction.
    if (np.ptp(values[:,2]) > max(20, centre[2]*.12)
            or np.ptp(values[:,0]) > 15 or np.ptp(values[:,1]) > 15):
        return None
    return tuple(float(v) for v in centre)


def save_observation(frame, prefix, color="yellow"):
    """Save raw, HSV and full fit overlay plus numeric feature for field QA."""
    import cv2
    import json
    import numpy as np
    from pathlib import Path
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    image = cv2.resize(frame, IMAGE_SIZE)
    overlay = image.copy()
    fit = _fit_arc(image, color)
    feature = None
    if fit is not None:
        feature, model, columns = fit
        xs = np.arange(int(columns.min()), int(columns.max())+1)
        ys = np.polyval(model, xs-320)
        points = np.stack((xs, ys), axis=1).astype(np.int32)
        cv2.polylines(overlay, [points], False, (255,0,255), 2)
        cv2.circle(overlay, (round(feature[0]), round(feature[1])), 5, (0,0,255), -1)
    for suffix, pixels in (("raw.jpg", frame), ("mask.png", target_mask(image, color)),
                           ("fit.jpg", overlay)):
        if not cv2.imwrite(str(prefix)+"_"+suffix, pixels):
            raise OSError(f"cannot save {prefix}_{suffix}")
    Path(str(prefix)+"_feature.json").write_text(json.dumps(
        {"color": color, "feature_x_y_span": feature}, indent=2), encoding="utf-8")
    return feature


def read_reference(path, color="yellow"):
    """Run exactly the live-frame extractor on a field/reference photograph."""
    import cv2

    image = cv2.imread(str(path))
    return extract_target_arc(image, color)


def reference_error(current, reference):
    """Live minus reference in pixels; X is logging only during side approach."""
    return current[0] - reference[0], current[1] - reference[1]
