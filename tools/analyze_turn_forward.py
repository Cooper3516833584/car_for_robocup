#!/usr/bin/env python3
"""Compare fixed-local post-turn error and synchronized C10B startup telemetry."""

import argparse
import json
import math
from pathlib import Path
import statistics


def yaw_change_deg(a, b):
    return math.degrees(math.atan2(math.sin(b - a), math.cos(b - a)))


def summarize(directory):
    directory = Path(directory)
    rows = [json.loads(line) for line in (directory / "action/events.jsonl").read_text(
        encoding="utf-8").splitlines()]
    summary = json.loads((directory / "action/summary.json").read_text(encoding="utf-8"))
    start = next((r for r in rows if r["type"] == "turn_forward_diag_stage_start"
                  and r["stage"].startswith("forward_")), None)
    result = {"directory": str(directory), "valid": summary["valid"],
              "error": summary["error"], "parameters": summary.get("parameters")}
    if start is None:
        return result
    turn_start = next((r for r in rows if r["type"] == "turn_forward_diag_stage_start"
                       and r["stage"].endswith("_90deg")), None)
    turn_done = next((r for r in rows if r["type"] == "turn_forward_diag_stage_done"
                      and r["stage"].endswith("_90deg")), None)
    poses = [r for r in rows if r["type"] == "t265_pose"]
    if turn_start and turn_done and poses:
        # Compare raw adapted T265 yaw as well as fusion: a fresh forward
        # reference can otherwise hide turn overshoot during the stopped hold.
        t0, t1, t2 = [min(poses, key=lambda r: abs(r["t"] - stage["t"]))
                      for stage in (turn_start, turn_done, start)]
        expected = 90.0 if turn_start["stage"].startswith("left") else -90.0
        commands_hold = [r for r in rows if r["type"] == "drive_command"
                         and turn_done["t"] <= r["t"] < start["t"]]
        result["turn_transition"] = {
            "hold_s": start["t"] - turn_done["t"],
            "t265_sample_offsets_s": [pose["t"] - stage["t"] for pose, stage
                                      in zip((t0, t1, t2), (turn_start, turn_done, start))],
            "fused_hold_yaw_change_deg": yaw_change_deg(
                turn_done["pose"]["yaw_rad"], start["pose"]["yaw_rad"]),
            "t265_hold_yaw_change_deg": yaw_change_deg(t1["yaw_rad"], t2["yaw_rad"]),
            "t265_forward_heading_error_from_turn_target_deg":
                yaw_change_deg(t0["yaw_rad"] + math.radians(expected), t2["yaw_rad"]),
            "hold_drive_command_rows": len(commands_hold),
            "hold_max_requested_omega_rad_s": max(
                (abs(r["requested_omega_rad_s"]) for r in commands_hold), default=None),
        }
    motion = [r for r in rows if r["type"] == "motion_action" and r["t"] >= start["t"]]
    if not motion:
        return result
    reference = {r.get("pose_reference") for r in motion}
    headings = [math.degrees(r["diagnostics"].get("heading_drift_rad", 0.0)) for r in motion]
    peak = max(motion, key=lambda r: abs(r["diagnostics"].get("heading_drift_rad", 0.0)))
    result.update({
        "pose_reference": sorted(str(v) for v in reference),
        "max_abs_heading_deg": max(abs(h) for h in headings),
        "min_heading_deg": min(headings), "max_heading_deg": max(headings),
        "peak_elapsed_s": peak["t"] - start["t"],
        "max_abs_cross_cm": max(abs(r["diagnostics"].get(
            "cross_track_error_m", 0.0)) * 100 for r in motion),
        "last_motion": motion[-1],
    })
    path = directory / "c10b.jsonl"
    if not path.exists():
        path = directory / "c10b-trace.jsonl"
    bases = [r["t265_received_time_monotonic"] - r["t"] for r in rows
             if r["type"] == "t265_pose" and "t265_received_time_monotonic" in r]
    if not path.exists() or not bases:
        return result
    base = statistics.median(bases)
    telemetry = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    if "turn_transition" in result:
        result["turn_transition"]["hold_telemetry"] = []
        for offset in (0.0, 0.2, 0.5, 0.9):
            window = [r for r in telemetry
                      if abs(r["t_mono"] - base - turn_done["t"] - offset) <= 0.075]
            if window:
                vx = statistics.mean(r["int16_be_fields"][0] for r in window)
                vz = statistics.mean(r["int16_be_fields"][2] for r in window)
                result["turn_transition"]["hold_telemetry"].append({
                    "elapsed_after_turn_s": offset, "feedback_vx_mm_s": vx,
                    "feedback_vz_mrad_s": vz, "feedback_left_mm_s": vx - 0.082 * vz,
                    "feedback_right_mm_s": vx + 0.082 * vz, "frames": len(window),
                })
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
