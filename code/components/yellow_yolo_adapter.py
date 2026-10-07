"""Thin adapter for target_yolo/verify_model.py's Ultralytics result API."""

from dataclasses import dataclass
import math
from pathlib import Path

from components.yolo_cpu import CpuDetector


@dataclass(frozen=True)
class YellowDetection:
    cx_px: float
    cy_px: float
    width_px: float
    height_px: float
    confidence: float


def load_detector(weights):
    path = Path(weights)
    if not path.is_file():
        raise FileNotFoundError(path)  # Never download substitute weights.
    from ultralytics import YOLO
    return CpuDetector(YOLO(str(path)))


def select_yellow(results, *, class_name="yellow", min_conf=0.55):
    best = None
    for result in results:
        if result.boxes is None:
            continue
        for box in result.boxes:
            if result.names[int(box.cls.item())] != class_name:
                continue
            confidence = float(box.conf.item())
            x1, y1, x2, y2 = map(float, box.xyxy[0].tolist())
            if (not all(math.isfinite(v) for v in (confidence, x1, y1, x2, y2))
                    or confidence < min_conf or x2 <= x1 or y2 <= y1):
                continue
            if best is None or confidence > best.confidence:
                best = YellowDetection((x1 + x2) / 2, (y1 + y2) / 2,
                                       x2 - x1, y2 - y1, confidence)
    return best
