#!/usr/bin/env python3
"""Compare fixed-local post-turn error and synchronized C10B startup telemetry."""

import argparse
import json
import math
from pathlib import Path
import statistics


def summarize(directory):
    directory = Path(directory)
    rows = [json.loads(line) for line in (directory / "action/events.jsonl").read_text(
        encoding="utf-8").splitlines()]
    summary = json.loads((directory / "action/summary.json").read_text(encoding="utf-8"))
    start = next((r for r in rows if r["type"] == "turn_forward_diag_stage_start"
                  and r["stage"] == "forward_20cm"), None)
    result = {"directory": str(directory), "valid": summary["valid"],
              "error": summary["error"], "parameters": summary.get("parameters")}
    if start is None:
        return result
    motion = [r for r in rows if r["type"] == "motion_action" and r["t"] >= start["t"]]
    if not motion:
        return result
    reference = {r.get("pose_reference") for r in motion}
    result.update({
        "pose_reference": sorted(str(v) for v in reference),
        "max_abs_heading_deg": max(abs(math.degrees(r["diagnostics"].get(
            "heading_drift_rad", 0.0))) for r in motion),
        "max_abs_cross_cm": max(abs(r["diagnostics"].get(
            "cross_track_error_m", 0.0)) * 100 for r in motion),
        "last_motion": motion[-1],
    })
    path = directory / "c10b.jsonl"
    bases = [r["t265_received_time_monotonic"] - r["t"] for r in rows
             if r["type"] == "t265_pose" and "t265_received_time_monotonic" in r]
    if not path.exists() or not bases:
        return result
    base = statistics.median(bases)
    telemetry = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    commands = [r for r in rows if r["type"] == "drive_command" and r["t"] >= start["t"]]
    result["startup"] = []
    for offset in (0.2, 0.45, 0.67, 0.9, 1.13, 1.5, 2.0):
        sample = min(motion, key=lambda r: abs(r["t"] - start["t"] - offset))
        command = min(commands, key=lambda r: abs(r["t"] - sample["t"]))
        window = [r for r in telemetry if abs(r["t_mono"] - base - sample["t"]) <= 0.075]
        if not window:
            continue
        vx = statistics.mean(r["int16_be_fields"][0] for r in window)
        vz = statistics.mean(r["int16_be_fields"][2] for r in window)
        # Stock firmware conversion track is 0.164 m, distinct from physical track.
        result["startup"].append({
            "elapsed_s": sample["t"] - start["t"],
            "heading_deg": math.degrees(sample["diagnostics"].get("heading_drift_rad", 0.0)),
            "target_left_mm_s": command["left_target_m_s"] * 1000,
            "target_right_mm_s": command["right_target_m_s"] * 1000,
            "feedback_left_mm_s": vx - 0.082 * vz,
            "feedback_right_mm_s": vx + 0.082 * vz,
            "telemetry_frames": len(window),
        })
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", nargs="+", type=Path)
    args = parser.parse_args()
    print(json.dumps([summarize(path) for path in args.directory], ensure_ascii=False, indent=2))
