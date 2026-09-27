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
    map_alignment_valid: bool = False


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
            event_type = row.get("type")
            d500_mode = str(row.get("d500_mode", "")).upper()
            if event_type == "t265_pose":
                source = "t265"
            elif event_type in {"d500_pose", "d500_global_fallback"}:
                source = "d500_global_fallback" if (
                    event_type == "d500_global_fallback" or d500_mode == "GLOBAL_PREDICTED"
                ) else "d500_absolute"
            else:
                source = None
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
            quality = PoseQuality("d500" if source.startswith("d500") else source, True, False, confidence, confidence, 0.0)
            map_alignment_valid = bool(
                row.get("d500_absolute_observation_accepted", row.get("map_alignment_valid", False))
                if source == "d500_absolute"
                else row.get("d500_map_alignment_established", row.get("map_alignment_valid", False))
            )
            events.append(PoseLogEvent(timestamp, source, pose, quality, map_alignment_valid))
    priority = {"t265": 0, "d500_absolute": 1, "d500_global_fallback": 2}
    events.sort(key=lambda event: (event.t_s, priority[event.source]))
    return events


def replay_fusion(events: Iterable[PoseLogEvent], fusion: PoseFusion) -> list[tuple[float, FusedPoseEstimate]]:
    """Feed time-sorted events without sleeping and return estimates per event."""
    output = []
    alignment_trusted = False
    for event in events:
        if event.source == "t265":
            fusion.update_t265(event.pose, event.quality)
        elif event.source == "d500_absolute":
            fusion.update_d500_absolute(event.pose, event.quality)
            alignment_trusted = alignment_trusted or event.map_alignment_valid
        else:
            fusion.update_d500_global_fallback(
                event.pose, event.quality,
                map_alignment_valid=alignment_trusted and event.map_alignment_valid,
            )
        output.append((event.t_s, fusion.estimate(event.t_s)))
    return output
