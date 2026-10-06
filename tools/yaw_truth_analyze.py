#!/usr/bin/env python3
"""Align external chassis-yaw CSV samples with a closed-loop motion event log."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

TWO_PI = 2.0 * math.pi
FOLLOW_UP_S = (0.2, 0.5, 1.0, 2.0, 3.0)


def _unwrap(values: list[float]) -> list[float]:
    if not values:
        return []
    result = [values[0]]
    for value in values[1:]:
        delta = (value - result[-1]) % TWO_PI
        if delta > math.pi:
            delta -= TWO_PI
        result.append(result[-1] + delta)
    return result


def _yaw_change_at(events: list[dict], matched: dict | None,
                   baseline: dict | None = None) -> float | None:
    if not events or matched is None or baseline is None:
        return None
    values = [float(event.get("yaw_rad", 0.0)) for event in events]
    unwrapped = _unwrap(values)
    index = min(range(len(events)), key=lambda i: abs(float(events[i]["t"]) - float(matched["t"])))
    baseline_index = min(range(len(events)),
                         key=lambda i: abs(float(events[i]["t"]) - float(baseline["t"])))
    return math.degrees(unwrapped[index] - unwrapped[baseline_index])


def _raw_quaternion_yaw(quaternion) -> float | None:
    if not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4:
        return None
    x, y, z, w = (float(item) for item in quaternion)
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not norm or not math.isfinite(norm):
        return None
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))


def _read_events(path: Path) -> list[dict]:
    events = []
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(event, dict) and isinstance(event.get("t"), (int, float)):
                events.append(event)
    return sorted(events, key=lambda event: float(event["t"]))


def _read_truth(path: Path, offset_s: float) -> tuple[list[dict], str, float]:
    rows = []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or ())
        time_column = "event_t" if "event_t" in fields else "time_s"
        yaw_column = "yaw_rad" if "yaw_rad" in fields else "yaw_deg"
        if time_column not in fields or yaw_column not in fields:
            raise ValueError("CSV must contain event_t/time_s and yaw_rad/yaw_deg columns")
        for row in reader:
            try:
                moment = float(row[time_column])
                yaw = float(row[yaw_column])
            except (TypeError, ValueError):
                continue
            if yaw_column == "yaw_deg":
                yaw = math.radians(yaw)
            if not math.isfinite(moment) or not math.isfinite(yaw):
                continue
            stop_value = str(row.get("physical_stop", "")).strip().lower()
            rows.append({"t": moment if time_column == "event_t" else moment + offset_s,
                         "yaw_rad": yaw,
                         "physical_stop": stop_value in {"1", "true", "yes", "y"}})
    rows.sort(key=lambda row: row["t"])
    for row, yaw in zip(rows, _unwrap([row["yaw_rad"] for row in rows])):
        row["yaw_unwrapped_rad"] = yaw
    effective_offset = offset_s if time_column == "time_s" else 0.0
    return rows, time_column, effective_offset


def _nearest(events: list[dict], moment: float, *, tolerance_s: float) -> dict | None:
    if not events:
        return None
    found = min(events, key=lambda event: abs(float(event["t"]) - moment))
    return found if abs(float(found["t"]) - moment) <= tolerance_s else None


def analyze(events_path: Path, truth_path: Path, *, time_offset_s: float,
            sync_uncertainty_ms: float) -> dict:
    if not math.isfinite(time_offset_s) or not math.isfinite(sync_uncertainty_ms) or sync_uncertainty_ms < 0:
        raise ValueError("time offset and synchronization uncertainty must be finite; uncertainty cannot be negative")
    events = _read_events(events_path)
    truth, truth_time_column, effective_offset = _read_truth(truth_path, time_offset_s)
    if not truth:
        raise ValueError("external truth CSV has no usable yaw rows")
    fused = [event for event in events if event.get("type") == "fused_pose"]
    t265 = [event for event in events if event.get("type") == "t265_pose"]
    raw_native = []
    for event in t265:
        yaw = _raw_quaternion_yaw(event.get("raw_quaternion_xyzw"))
        if yaw is not None:
            raw_native.append({"t": event["t"], "yaw_rad": yaw})
    commands = [event for event in events if event.get("type") == "drive_command"]
    anchors = [event for event in events if event.get("type") == "slam_anchor"]
    tolerance = max(0.10, sync_uncertainty_ms / 1000.0 * 2.0)
    truth_start = truth[0]["yaw_unwrapped_rad"]
    baseline_fused = _nearest(fused, truth[0]["t"], tolerance_s=tolerance)
    baseline_t265 = _nearest(t265, truth[0]["t"], tolerance_s=tolerance)
    baseline_raw = _nearest(raw_native, truth[0]["t"], tolerance_s=tolerance)
    aligned = []
    for row in truth:
        moment = row["t"]
        fused_event = _nearest(fused, moment, tolerance_s=tolerance)
        t265_event = _nearest(t265, moment, tolerance_s=tolerance)
        command = _nearest(commands, moment, tolerance_s=tolerance)
        anchor = _nearest(anchors, moment, tolerance_s=tolerance)
        raw_event = _nearest(raw_native, moment, tolerance_s=tolerance)
        aligned.append({
            "event_t": moment,
            "external_yaw_deg": math.degrees(row["yaw_unwrapped_rad"] - truth_start),
            "physical_stop": row["physical_stop"],
            "fused_yaw_change_deg": _yaw_change_at(fused, fused_event, baseline_fused),
            "t265_adapter_yaw_change_deg": _yaw_change_at(t265, t265_event, baseline_t265),
            # Quaternion yaw is left in native T265 axes; it is not a base_link truth value.
            "raw_quaternion_yaw_native_change_deg": _yaw_change_at(raw_native, raw_event, baseline_raw),
            "raw_angular_velocity_xyz": None if t265_event is None else
                                        t265_event.get("raw_angular_velocity_xyz"),
            "device_timestamp_ms": None if t265_event is None else
                                   t265_event.get("t265_device_timestamp_ms"),
            "mapped_measurement_time_before_causal_clamp": None if t265_event is None else
                t265_event.get("t265_mapped_measurement_time_before_causal_clamp"),
            "measurement_time_used": None if t265_event is None else
                                     t265_event.get("t265_measurement_time_used"),
            "received_time": None if t265_event is None else
                             t265_event.get("t265_received_time"),
            "measurement_time_clamped_to_receive": None if t265_event is None else
                t265_event.get("measurement_time_clamped_to_receive"),
            "command_omega_rad_s": None if command is None else command.get("limited_omega_rad_s"),
            "slam_anchor": None if anchor is None else {
                "accepted": anchor.get("accepted"),
                "rejection_reason": anchor.get("rejection_reason"),
                "innovation_m": anchor.get("innovation_m"),
                "innovation_yaw_rad": anchor.get("innovation_yaw_rad"),
                "candidate_count": anchor.get("candidate_count"),
                "candidate_age_s": anchor.get("candidate_age_s"),
                "migration_active": anchor.get("migration_active"),
            },
        })
    last_fused_yaw = next((row["fused_yaw_change_deg"] for row in reversed(aligned)
                           if row["fused_yaw_change_deg"] is not None), None)
    last_adapter_yaw = next((row["t265_adapter_yaw_change_deg"] for row in reversed(aligned)
                             if row["t265_adapter_yaw_change_deg"] is not None), None)
    summary = {
        "truth_samples": len(truth),
        "aligned_samples": len(aligned),
        "truth_time_column": truth_time_column,
        "time_offset_s_applied": effective_offset,
        "time_sync_uncertainty_ms": sync_uncertainty_ms,
        "nearest_sample_tolerance_s": tolerance,
        "external_yaw_change_deg": round(aligned[-1]["external_yaw_deg"], 3),
        "fused_yaw_change_deg": last_fused_yaw,
        "t265_adapter_yaw_change_deg": last_adapter_yaw,
        "yaw_error_external_minus_fused_deg": (
            None if last_fused_yaw is None else
            round(aligned[-1]["external_yaw_deg"] - last_fused_yaw, 3)
        ),
    }
    stop_rows = [row for row in truth if row["physical_stop"]]
    if stop_rows:
        stop_t = stop_rows[0]["t"]
        stop_yaw = stop_rows[0]["yaw_unwrapped_rad"]
        follow_up = {}
        for delay in FOLLOW_UP_S:
            match = min(truth, key=lambda row: abs(row["t"] - (stop_t + delay)))
            follow_up[str(delay)] = (
                round(math.degrees(match["yaw_unwrapped_rad"] - stop_yaw), 3)
                if abs(match["t"] - (stop_t + delay)) <= tolerance else None
            )
        summary["external_yaw_after_physical_stop_deg"] = follow_up
    return {"summary": summary, "aligned_samples": aligned}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--events", required=True, type=Path)
    parser.add_argument("--truth-csv", required=True, type=Path)
    parser.add_argument("--time-offset-s", type=float, default=0.0,
                        help="add to CSV time_s to map it into events.jsonl t")
    parser.add_argument("--sync-uncertainty-ms", required=True, type=float,
                        help="estimated external-to-event clock alignment uncertainty")
    parser.add_argument("--output", type=Path, help="optional JSON output path")
    args = parser.parse_args(argv)
    try:
        result = analyze(args.events, args.truth_csv, time_offset_s=args.time_offset_s,
                         sync_uncertainty_ms=args.sync_uncertainty_ms)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
