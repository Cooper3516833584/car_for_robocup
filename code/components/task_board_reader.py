#!/usr/bin/env python3
"""RoboCup task-board reader: OpenCV rectification + lightweight Chinese OCR.

Designed for Cooper3516833584/car_for_robocup.  The module deliberately owns
only camera/image perception.  It does not command the drive base and does not
know mission strategy.

Production intent:
- The car turns to face the board after the referee starts the round.
- A short burst of frames is passed to TaskBoardReader.
- Only a validated {red, blue, green} result whose sum is 4 is accepted.
- A failed read is an explicit failure; never silently default to 2/1/1.

The OCR dependency is imported lazily so the rest of the repository and unit
tests remain importable on machines without RapidOCR/ONNX Runtime.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from itertools import islice
from pathlib import Path
import json
import math
import re
import sys
import unicodedata
from typing import Iterable, Protocol, Sequence

import numpy as np

try:
    import cv2  # type: ignore
except ImportError:  # pragma: no cover - exercised on minimal dev machines
    cv2 = None  # type: ignore


ColorName = str
Quad = tuple[
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
]


@dataclass(frozen=True, slots=True)
class OCRToken:
    text: str
    score: float = 1.0
    box: Quad | None = None

    @property
    def y_center(self) -> float:
        if self.box is None:
            return 0.0
        return sum(point[1] for point in self.box) / 4.0

    @property
    def x_center(self) -> float:
        if self.box is None:
            return 0.0
        return sum(point[0] for point in self.box) / 4.0

    @property
    def height(self) -> float:
        if self.box is None:
            return 0.0
        ys = [point[1] for point in self.box]
        return max(ys) - min(ys)


@dataclass(frozen=True, slots=True)
class TaskCounts:
    red: int
    blue: int
    green: int

    def __post_init__(self) -> None:
        for value in (self.red, self.blue, self.green):
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 4:
                raise ValueError("task count must be in [0, 4]")
        if self.total != 4:
            raise ValueError("task counts must add up to 4")

    @property
    def total(self) -> int:
        return self.red + self.blue + self.green

    @property
    def key(self) -> tuple[int, int, int]:
        return self.red, self.blue, self.green

    def as_dict(self) -> dict[str, int]:
        return {"red": self.red, "blue": self.blue, "green": self.green}


@dataclass(frozen=True, slots=True)
class ParsedTask:
    counts: TaskCounts | None
    confidence: float
    inferred_colors: tuple[ColorName, ...] = ()
    lines: tuple[str, ...] = ()
    reason: str | None = None

    @property
    def valid(self) -> bool:
        return self.counts is not None


@dataclass(frozen=True, slots=True)
class TaskBoardResult:
    counts: TaskCounts | None
    confidence: float
    raw_lines: tuple[str, ...] = ()
    inferred_colors: tuple[ColorName, ...] = ()
    board_quad: Quad | None = None
    source: str = "none"
    reason: str | None = None
    votes: int = 1

    @property
    def valid(self) -> bool:
        return self.counts is not None

    @property
    def task(self) -> dict[str, int] | None:
        return None if self.counts is None else self.counts.as_dict()

    def to_json_dict(self) -> dict[str, object]:
        return {
            "valid": self.valid,
            "task": self.task,
            "confidence": round(float(self.confidence), 6),
            "raw_lines": list(self.raw_lines),
            "inferred_colors": list(self.inferred_colors),
            "board_quad": None if self.board_quad is None else self.board_quad,
            "source": self.source,
            "reason": self.reason,
            "votes": self.votes,
        }


@dataclass(frozen=True, slots=True)
class TaskBoardConfig:
    canonical_width_px: int = 1200
    canonical_height_px: int = 520
    minimum_quad_area_ratio: float = 0.035
    maximum_quad_area_ratio: float = 0.92
    minimum_aspect_ratio: float = 1.30
    maximum_aspect_ratio: float = 3.80
    minimum_rectangularity: float = 0.68
    minimum_board_brightness: float = 120.0
    minimum_ocr_score: float = 0.45
    allow_missing_color_inference: bool = True
    full_frame_fallback: bool = True
    enhanced_ocr_retry: bool = True
    required_consensus_votes: int = 3
    maximum_frames: int = 10
    debug_directory: Path | None = None

    def __post_init__(self) -> None:
        if self.canonical_width_px < 200 or self.canonical_height_px < 120:
            raise ValueError("canonical task-board image is too small")
        if not 0.0 < self.minimum_quad_area_ratio < self.maximum_quad_area_ratio <= 1.0:
            raise ValueError("invalid task-board area ratios")
        if not 1.0 < self.minimum_aspect_ratio < self.maximum_aspect_ratio:
            raise ValueError("invalid board aspect-ratio range")
        if not 0.0 <= self.minimum_rectangularity <= 1.0:
            raise ValueError("minimum_rectangularity must be in [0, 1]")
        if not 0.0 <= self.minimum_ocr_score <= 1.0:
            raise ValueError("minimum_ocr_score must be in [0, 1]")
        if self.required_consensus_votes <= 0 or self.maximum_frames <= 0:
            raise ValueError("consensus frame limits must be positive")
        if self.required_consensus_votes > self.maximum_frames:
            raise ValueError("required votes cannot exceed maximum frames")


class OCRBackend(Protocol):
    def recognize(self, image: np.ndarray) -> Sequence[OCRToken]: ...


class RapidOCRBackend:
    """Small adapter around RapidOCR's current RapidOCROutput API.

    Current RapidOCR exposes result.txts/result.scores/result.boxes.  The
    defensive legacy branch makes the component tolerant of older tuple-style
    returns encountered on pre-existing robot images.
    """

    def __init__(self) -> None:
        self._engine = None

    def _load(self):
        if self._engine is None:
            try:
                from rapidocr import RapidOCR  # type: ignore
            except ImportError as exc:  # pragma: no cover - hardware setup path
                raise RuntimeError(
                    "RapidOCR is not installed. Install requirements-task-board.txt"
                ) from exc
            self._engine = RapidOCR()
        return self._engine

    def recognize(self, image: np.ndarray) -> Sequence[OCRToken]:
        engine = self._load()
        result = engine(image, use_cls=False)
        if result is None:
            return ()

        if hasattr(result, "txts"):
            texts = getattr(result, "txts", None)
            texts = () if texts is None else texts
            scores = getattr(result, "scores", None)
            scores = () if scores is None else scores
            boxes = getattr(result, "boxes", None)
            tokens: list[OCRToken] = []
            for index, text in enumerate(texts):
                score = float(scores[index]) if index < len(scores) else 1.0
                box = _coerce_quad(boxes[index]) if boxes is not None and index < len(boxes) else None
                tokens.append(OCRToken(str(text), score, box))
            return tokens

        # Older RapidOCR releases returned (result, elapsed).  Each result item
        # commonly looked like [box, text, score].
        payload = result[0] if isinstance(result, tuple) and result else result
        tokens = []
        if isinstance(payload, (list, tuple)):
            for item in payload:
                if not isinstance(item, (list, tuple)) or len(item) < 2:
                    continue
                box = _coerce_quad(item[0])
                text = str(item[1])
                score = float(item[2]) if len(item) >= 3 else 1.0
                tokens.append(OCRToken(text, score, box))
        return tokens


def _require_opencv() -> None:
    if cv2 is None:
        raise RuntimeError("OpenCV (cv2) is required for task-board image processing")


def _coerce_quad(value) -> Quad | None:
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=np.float32).reshape(4, 2)
    except (TypeError, ValueError):
        return None
    return tuple((float(x), float(y)) for x, y in array)  # type: ignore[return-value]


def order_quad(points: np.ndarray | Sequence[Sequence[float]]) -> np.ndarray:
    """Return four points in TL, TR, BR, BL order without scipy/imutils."""
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    cyclic = pts[np.argsort(angles)]
    # With image y pointing down, atan2 sorting usually starts on the upper-left
    # or lower-left.  Rotate so the smallest x+y point is the first (TL).
    start = int(np.argmin(cyclic[:, 0] + cyclic[:, 1]))
    cyclic = np.roll(cyclic, -start, axis=0)
    # Ensure TL -> TR -> BR -> BL, not TL -> BL -> BR -> TR.
    if cyclic[1, 0] < cyclic[-1, 0]:
        cyclic = cyclic[[0, 3, 2, 1]]
    return cyclic.astype(np.float32)


def warp_task_board(image: np.ndarray, quad: np.ndarray | Sequence[Sequence[float]], *, width: int, height: int) -> np.ndarray:
    _require_opencv()
    src = order_quad(quad)
    dst = np.array(
        [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(image, matrix, (width, height), flags=cv2.INTER_CUBIC)


def _quad_metrics(quad: np.ndarray, image_shape: tuple[int, ...], gray: np.ndarray) -> tuple[float, float, float, float]:
    _require_opencv()
    ordered = order_quad(quad)
    area = abs(float(cv2.contourArea(ordered)))
    image_area = float(image_shape[0] * image_shape[1])
    area_ratio = area / max(1.0, image_area)
    widths = [np.linalg.norm(ordered[1] - ordered[0]), np.linalg.norm(ordered[2] - ordered[3])]
    heights = [np.linalg.norm(ordered[3] - ordered[0]), np.linalg.norm(ordered[2] - ordered[1])]
    aspect = max(widths) / max(1.0, max(heights))
    rect = cv2.minAreaRect(ordered)
    rect_area = float(rect[1][0] * rect[1][1])
    rectangularity = area / rect_area if rect_area > 1.0 else 0.0
    mask = np.zeros(gray.shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, ordered.astype(np.int32), 255)
    brightness = float(cv2.mean(gray, mask=mask)[0])
    return area_ratio, aspect, rectangularity, brightness


def find_task_board_quad(image: np.ndarray, config: TaskBoardConfig = TaskBoardConfig()) -> np.ndarray | None:
    """Locate a likely white landscape KT-board quadrilateral.

    This intentionally does not assume the exact sample-board layout.  It scores
    large bright quadrilaterals and leaves whole-frame OCR as a fallback.
    """
    _require_opencv()
    if image is None or image.ndim not in (2, 3):
        raise ValueError("image must be a grayscale or BGR numpy array")
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 45, 145)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

    # A second contour source helps when the border is weak but the board face
    # is much brighter than its surroundings.
    _, bright = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    sources = (edges, bright)
    best: tuple[float, np.ndarray] | None = None
    image_area = image.shape[0] * image.shape[1]

    for source_index, source in enumerate(sources):
        contours, _ = cv2.findContours(source, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = abs(float(cv2.contourArea(contour)))
            if area < config.minimum_quad_area_ratio * image_area:
                continue
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                continue
            approx = cv2.approxPolyDP(contour, 0.018 * perimeter, True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue
            quad = approx.reshape(4, 2).astype(np.float32)
            h, w = image.shape[:2]
            boundary_hits = sum(
                1
                for x, y in quad
                if x <= 3.0 or y <= 3.0 or x >= w - 4.0 or y >= h - 4.0
            )
            if boundary_hits >= 2:
                continue
            area_ratio, aspect, rectangularity, brightness = _quad_metrics(quad, image.shape, gray)
            if source_index == 1 and area_ratio > 0.45:
                # The bright-mask source can otherwise mistake a large wall or
                # floor region touching the horizon for the white task board.
                continue
            if not config.minimum_quad_area_ratio <= area_ratio <= config.maximum_quad_area_ratio:
                continue
            if not config.minimum_aspect_ratio <= aspect <= config.maximum_aspect_ratio:
                continue
            if rectangularity < config.minimum_rectangularity:
                continue
            if brightness < config.minimum_board_brightness:
                continue
            aspect_preference = math.exp(-abs(math.log(max(aspect, 1e-6) / 2.30)))
            brightness_score = min(1.0, max(0.0, (brightness - 100.0) / 155.0))
            border_bonus = 0.05 if source_index == 0 else 0.0
            score = (
                1.75 * area_ratio
                + 0.55 * rectangularity
                + 0.30 * brightness_score
                + 0.20 * aspect_preference
                + border_bonus
            )
            if best is None or score > best[0]:
                best = (score, order_quad(quad))
    return None if best is None else best[1]


def enhance_for_ocr(image: np.ndarray) -> np.ndarray:
    _require_opencv()
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=1.8, tileGridSize=(8, 8))
    enhanced = clahe.apply(gray)
    blurred = cv2.GaussianBlur(enhanced, (0, 0), 1.0)
    sharpened = cv2.addWeighted(enhanced, 1.55, blurred, -0.55, 0)
    return cv2.cvtColor(sharpened, cv2.COLOR_GRAY2BGR)


_CHINESE_DIGITS = {
    "零": 0,
    "〇": 0,
    "一": 1,
    "二": 2,
    "两": 2,
    "兩": 2,
    "三": 3,
    "四": 4,
}
_COLOR_ALIASES: dict[ColorName, tuple[str, ...]] = {
    "red": ("红色", "紅色", "红", "紅"),
    "blue": ("蓝色", "藍色", "蓝", "藍"),
    "green": ("绿色", "綠色", "緑色", "绿", "綠", "緑"),
}
_SAFE_REPLACEMENTS = {
    "紅色": "红色",
    "藍色": "蓝色",
    "綠色": "绿色",
    "緑色": "绿色",
    "地快": "地块",
    "物資": "物资",
}
_COLOR_PATTERN = re.compile("[红紅蓝藍绿綠緑]色?")
_COLOR_NAMES = dict(zip("红紅蓝藍绿綠緑", ("red", "red", "blue", "blue", "green", "green", "green")))
_NUMBER_PATTERN = re.compile(r"[-−]?[0-9零〇一二两兩三四五六七八九十百]+(?:[.点][0-9]+)?")
_COUNT_GAP = r"[\s:：,，;；、|()（）\[\]【】=。．]{0,8}"
# Capture whole numeric tokens (including invalid ones), never the first digit
# of e.g. 12 or -1. Region identifiers are ignored only when quantity semantics
# supply explicit evidence inside the same color span.
_STRONG_COUNT_PATTERNS = (
    re.compile(r"(?:需要|需)?(?:投入|投放|投)" + _COUNT_GAP + "(" + _NUMBER_PATTERN.pattern + ")"),
    re.compile(r"(?:物资)?数量" + _COUNT_GAP + "(" + _NUMBER_PATTERN.pattern + ")"),
    re.compile("(" + _NUMBER_PATTERN.pattern + ")" + _COUNT_GAP
               + r"(?:个|件)" + _COUNT_GAP + r"(?:救援)?物资"),
)


def normalize_ocr_text(text: str) -> str:
    normalized = unicodedata.normalize("NFKC", str(text))
    for old, new in _SAFE_REPLACEMENTS.items():
        normalized = normalized.replace(old, new)
    normalized = re.sub(r"\s+", "", normalized)
    return normalized


def _digit_value(token: str) -> int | None:
    if token.isdigit():
        value = int(token)
        return value if 0 <= value <= 4 else None
    return _CHINESE_DIGITS.get(token)


def _token_line_groups(tokens: Sequence[OCRToken]) -> tuple[tuple[OCRToken, ...], ...]:
    if not tokens:
        return ()
    with_box = [token for token in tokens if token.box is not None]
    without_box = [token for token in tokens if token.box is None]
    groups: list[list[OCRToken]] = []
    if with_box:
        ordered = sorted(with_box, key=lambda token: (token.y_center, token.x_center))
        positive_heights = [token.height for token in ordered if token.height > 0.0]
        median_height = float(np.median(positive_heights)) if positive_heights else 30.0
        tolerance = max(10.0, 0.62 * median_height)
        for token in ordered:
            target = None
            for group in groups:
                center = sum(item.y_center for item in group) / len(group)
                if abs(token.y_center - center) <= tolerance:
                    target = group
                    break
            if target is None:
                groups.append([token])
            else:
                target.append(token)
        groups.sort(key=lambda group: sum(item.y_center for item in group) / len(group))
        for group in groups:
            group.sort(key=lambda token: token.x_center)
    if without_box:
        groups.extend([[token] for token in without_box])
    return tuple(tuple(group) for group in groups)


def ocr_lines(tokens: Sequence[OCRToken], *, minimum_score: float = 0.0) -> tuple[tuple[str, float], ...]:
    kept = [token for token in tokens if token.text.strip() and token.score >= minimum_score]
    lines: list[tuple[str, float]] = []
    for group in _token_line_groups(kept):
        text = "".join(token.text for token in group)
        if not text:
            continue
        score = sum(float(token.score) for token in group) / len(group)
        lines.append((normalize_ocr_text(text), score))
    return tuple(lines)


def _extract_count_from_color_span(span: str) -> int | None:
    """Prefer explicit quantities; otherwise require one unambiguous value.

    Ambiguity and malformed numeric evidence raise instead of returning None,
    so the parser cannot silently fill an uncertain color from the total of 4.
    """
    strong = [match.group(1) for pattern in _STRONG_COUNT_PATTERNS for match in pattern.finditer(span)]
    candidates = strong or [match.group() for match in _NUMBER_PATTERN.finditer(span)]
    values = set()
    for raw in candidates:
        value = _digit_value(raw) if len(raw) == 1 else None
        if value is None:
            raise ValueError(f"invalid count: {raw!r}")
        values.add(value)
    if len(values) > 1:
        raise ValueError(f"ambiguous count: {sorted(values)}")
    return next(iter(values)) if values else None


def _extract_counts(text: str) -> list[tuple[ColorName, int]]:
    """Keep each quantity inside its own color span; reject malformed counts."""
    anchors = list(_COLOR_PATTERN.finditer(text))
    observations = []
    for index, anchor in enumerate(anchors):
        color = _COLOR_NAMES[anchor.group()[0]]
        end = anchors[index + 1].start() if index + 1 < len(anchors) else len(text)
        span = text[anchor.end():end]
        try:
            value = _extract_count_from_color_span(span)
        except ValueError as exc:
            raise ValueError(f"{color}: {exc}") from exc
        if value is None:
            continue
        observations.append((color, value))
    return observations


def parse_task_tokens(
    tokens: Sequence[OCRToken],
    *,
    minimum_score: float = 0.45,
    allow_missing_color_inference: bool = True,
) -> ParsedTask:
    lines_with_score = ocr_lines(tokens, minimum_score=minimum_score)
    lines = tuple(text for text, _ in lines_with_score)
    if not lines:
        return ParsedTask(None, 0.0, lines=(), reason="no usable OCR text")

    # Resolve complete color spans first: a separate OCR line containing a
    # region identifier must not override quantity semantics on the next line.
    # Color anchors still bound every span, including across OCR line breaks.
    joined = "|".join(lines)
    joined_score = sum(score for _, score in lines_with_score) / len(lines_with_score)
    try:
        joined_observations = _extract_counts(joined)
    except ValueError as exc:
        return ParsedTask(None, 0.0, lines=lines, reason=str(exc))
    line_evidence: dict[ColorName, list[tuple[int, float]]] = defaultdict(list)
    for text, score in lines_with_score:
        try:
            for color, value in _extract_counts(text):
                line_evidence[color].append((value, score))
        except ValueError:
            # An incomplete OCR line can hold a non-quantity region number.
            # The full spans above have already validated the actual counts.
            continue
    observations: dict[ColorName, list[tuple[int, float]]] = defaultdict(list)
    for color, value in dict.fromkeys(joined_observations):
        scores = [score for seen_value, score in line_evidence[color] if seen_value == value]
        observations[color].extend((value, score) for score in (scores or [joined_score * 0.92]))

    resolved: dict[ColorName, int] = {}
    evidence_scores: list[float] = []
    for color, seen in observations.items():
        values = {value for value, _ in seen}
        if len(values) > 1:
            return ParsedTask(
                None,
                0.0,
                lines=lines,
                reason=f"conflicting OCR values for {color}: {sorted(values)}",
            )
        if values:
            value = next(iter(values))
            resolved[color] = value
            evidence_scores.extend(score for seen_value, score in seen if seen_value == value)

    inferred: list[ColorName] = []
    missing = [color for color in ("red", "blue", "green") if color not in resolved]
    if len(missing) == 1 and allow_missing_color_inference:
        inferred_value = 4 - sum(resolved.values())
        if 0 <= inferred_value <= 4:
            resolved[missing[0]] = inferred_value
            inferred.append(missing[0])
            evidence_scores.append(max(0.0, joined_score - 0.20))
            missing = []

    if missing:
        return ParsedTask(
            None,
            max(evidence_scores, default=0.0) * 0.5,
            lines=lines,
            reason="missing color counts: " + ", ".join(missing),
        )

    try:
        counts = TaskCounts(resolved["red"], resolved["blue"], resolved["green"])
    except ValueError as exc:
        return ParsedTask(None, 0.0, lines=lines, reason=str(exc))

    confidence = sum(evidence_scores) / max(1, len(evidence_scores))
    confidence -= 0.12 * len(inferred)
    confidence = max(0.0, min(1.0, confidence))
    return ParsedTask(counts, confidence, tuple(inferred), lines, None)


class TaskBoardReader:
    def __init__(
        self,
        config: TaskBoardConfig = TaskBoardConfig(),
        ocr_backend: OCRBackend | None = None,
    ) -> None:
        self.config = config
        self.ocr = ocr_backend or RapidOCRBackend()
        self._debug_sequence = 0

    def recognize_frame(self, frame: np.ndarray) -> TaskBoardResult:
        _require_opencv()
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            return TaskBoardResult(None, 0.0, reason="empty frame")

        quad = find_task_board_quad(frame, self.config)
        candidates: list[tuple[str, np.ndarray, Quad | None]] = []
        if quad is not None:
            warped = warp_task_board(
                frame,
                quad,
                width=self.config.canonical_width_px,
                height=self.config.canonical_height_px,
            )
            candidates.append(("rectified", warped, _coerce_quad(quad)))
        if self.config.full_frame_fallback:
            candidates.append(("full_frame", frame, None))

        if not candidates:
            self._save_debug(frame, None, None, (), "no_board_candidate")
            return TaskBoardResult(None, 0.0, board_quad=None, reason="task board not found")

        failures: list[str] = []
        best_invalid: TaskBoardResult | None = None
        for source, image, source_quad in candidates:
            attempts = [(source, image)]
            if self.config.enhanced_ocr_retry:
                attempts.append((source + "_enhanced", enhance_for_ocr(image)))
            for attempt_source, attempt_image in attempts:
                try:
                    tokens = tuple(self.ocr.recognize(attempt_image))
                except Exception as exc:
                    return TaskBoardResult(None, 0.0, source=attempt_source,
                                           reason=f"OCR backend failed: {type(exc).__name__}: {exc}", votes=0)
                parsed = parse_task_tokens(
                    tokens,
                    minimum_score=self.config.minimum_ocr_score,
                    allow_missing_color_inference=self.config.allow_missing_color_inference,
                )
                result = TaskBoardResult(
                    parsed.counts,
                    parsed.confidence,
                    parsed.lines,
                    parsed.inferred_colors,
                    source_quad,
                    attempt_source,
                    parsed.reason,
                )
                self._save_debug(frame, source_quad, attempt_image, tokens, attempt_source, result)
                if result.valid:
                    return result
                failures.append(f"{attempt_source}: {parsed.reason}")
                if best_invalid is None or result.confidence > best_invalid.confidence:
                    best_invalid = result

        if best_invalid is not None:
            return TaskBoardResult(
                None,
                best_invalid.confidence,
                best_invalid.raw_lines,
                best_invalid.inferred_colors,
                best_invalid.board_quad,
                best_invalid.source,
                "; ".join(failures),
            )
        return TaskBoardResult(None, 0.0, reason="; ".join(failures) or "OCR failed")

    def recognize_frames(self, frames: Iterable[np.ndarray]) -> TaskBoardResult:
        ballots: dict[tuple[int, int, int], list[TaskBoardResult]] = defaultdict(list)
        invalid_reasons: list[str] = []
        best_invalid: TaskBoardResult | None = None
        seen_frames = 0

        for frame in islice(frames, self.config.maximum_frames):
            seen_frames += 1
            result = self.recognize_frame(frame)
            if not result.valid:
                invalid_reasons.append(result.reason or "invalid")
                if best_invalid is None or result.confidence > best_invalid.confidence:
                    best_invalid = result
                continue
            assert result.counts is not None
            group = ballots[result.counts.key]
            group.append(result)
            if len(group) >= self.config.required_consensus_votes:
                return self._merge_votes(group)

        if ballots:
            winner_key, winner_group = max(
                ballots.items(),
                key=lambda item: (len(item[1]), sum(value.confidence for value in item[1])),
            )
            return TaskBoardResult(
                None,
                sum(item.confidence for item in winner_group) / len(winner_group),
                winner_group[-1].raw_lines,
                winner_group[-1].inferred_colors,
                winner_group[-1].board_quad,
                winner_group[-1].source,
                f"consensus not reached: best={winner_key}, votes={len(winner_group)}/{self.config.required_consensus_votes}",
                votes=len(winner_group),
            )

        if best_invalid is not None:
            return TaskBoardResult(
                None,
                best_invalid.confidence,
                best_invalid.raw_lines,
                best_invalid.inferred_colors,
                best_invalid.board_quad,
                best_invalid.source,
                "no valid frame; " + " | ".join(invalid_reasons[-3:]),
                votes=0,
            )
        return TaskBoardResult(None, 0.0, reason="no frames supplied", votes=0)

    def recognize_camera(
        self,
        camera_index: int | str = 0,
        *,
        width: int = 1280,
        height: int = 720,
        warmup_frames: int = 6,
    ) -> TaskBoardResult:
        _require_opencv()
        backend = cv2.CAP_V4L2 if sys.platform.startswith("linux") else cv2.CAP_ANY
        capture = cv2.VideoCapture(camera_index, backend)
        if not capture.isOpened():
            capture.release()
            return TaskBoardResult(None, 0.0, reason=f"cannot open camera {camera_index!r}")
        try:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            for _ in range(max(0, warmup_frames)):
                capture.read()

            def frames():
                for _ in range(self.config.maximum_frames):
                    ok, frame = capture.read()
                    if ok and frame is not None:
                        yield frame

            return self.recognize_frames(frames())
        finally:
            capture.release()

    @staticmethod
    def _merge_votes(results: Sequence[TaskBoardResult]) -> TaskBoardResult:
        last = results[-1]
        assert last.counts is not None
        confidence = sum(result.confidence for result in results) / len(results)
        inferred = tuple(sorted({color for result in results for color in result.inferred_colors}))
        return TaskBoardResult(
            last.counts,
            confidence,
            last.raw_lines,
            inferred,
            last.board_quad,
            last.source,
            None,
            votes=len(results),
        )

    def _save_debug(
        self,
        original: np.ndarray,
        quad: Quad | None,
        ocr_image: np.ndarray | None,
        tokens: Sequence[OCRToken],
        label: str,
        result: TaskBoardResult | None = None,
    ) -> None:
        directory = self.config.debug_directory
        if directory is None or cv2 is None:
            return
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        index = self._debug_sequence
        self._debug_sequence += 1
        stem = f"{index:04d}_{label}"
        overlay = original.copy()
        if quad is not None:
            cv2.polylines(
                overlay,
                [np.asarray(quad, dtype=np.int32)],
                isClosed=True,
                color=(0, 255, 0),
                thickness=3,
            )
        cv2.imwrite(str(directory / f"{stem}_scene.jpg"), overlay)
        if ocr_image is not None:
            cv2.imwrite(str(directory / f"{stem}_ocr.jpg"), ocr_image)
        payload = {
            "tokens": [
                {"text": token.text, "score": token.score, "box": token.box}
                for token in tokens
            ],
            "result": None if result is None else result.to_json_dict(),
        }
        (directory / f"{stem}.json").write_text(
            json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
        )


__all__ = [
    "OCRBackend",
    "OCRToken",
    "ParsedTask",
    "RapidOCRBackend",
    "TaskBoardConfig",
    "TaskBoardReader",
    "TaskBoardResult",
    "TaskCounts",
    "enhance_for_ocr",
    "find_task_board_quad",
    "normalize_ocr_text",
    "ocr_lines",
    "order_quad",
    "parse_task_tokens",
    "warp_task_board",
]
