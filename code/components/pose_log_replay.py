"""Canonical sensor-event JSONL reader and deterministic fusion replay."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable

from components.pose_fusion import FusedPoseEstimate, PoseFusion
from core.types import Pose2D, PoseQuality


@dataclass(frozen=True, slots=True)
class PoseLogEvent:
    t_s: float
    source: str
    pose: Pose2D
    quality: PoseQuality


def read_pose_events(path: str | Path) -> list[PoseLogEvent]:
    events: list[PoseLogEvent] = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSONL at line {line_number}: {exc}") from exc
            if not isinstance(row, dict):
                raise ValueError(f"event at line {line_number} must be a JSON object")
            source = {"t265_pose": "t265", "d500_pose": "d500"}.get(row.get("type"))
            if source is None:
                continue
            try:
                timestamp = float(row["t"])
                x_m, y_m, yaw_rad = float(row["x_m"]), float(row["y_m"]), float(row["yaw_rad"])
                valid = bool(row.get("valid", True))
                confidence = float(row.get("confidence", 1.0))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid pose event at line {line_number}: {exc}") from exc
            if not all(math.isfinite(value) for value in (timestamp, x_m, y_m, yaw_rad, confidence)):
                raise ValueError(f"non-finite pose event at line {line_number}")
            if timestamp < 0.0 or not 0.0 <= confidence <= 1.0:
                raise ValueError(f"invalid time or confidence at line {line_number}")
            if not valid:
                continue
            pose = Pose2D(x_m, y_m, yaw_rad, timestamp)
            quality = PoseQuality(source, True, False, confidence, confidence, 0.0)
            events.append(PoseLogEvent(timestamp, source, pose, quality))
    events.sort(key=lambda event: (event.t_s, 0 if event.source == "t265" else 1))
    return events


def replay_fusion(events: Iterable[PoseLogEvent], fusion: PoseFusion) -> list[tuple[float, FusedPoseEstimate]]:
    """Feed time-sorted events without sleeping and return estimates per event."""
    output = []
    for event in events:
        if event.source == "t265":
            fusion.update_t265(event.pose, event.quality)
        else:
            fusion.update_d500(event.pose, event.quality)
        output.append((event.t_s, fusion.estimate(event.t_s)))
    return output
