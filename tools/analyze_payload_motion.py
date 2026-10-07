"""Summarize payload-stage motion commands from an existing JSONL log."""

import argparse
import json
import math
from pathlib import Path


def summarize(path):
    stages = []
    active = None
    rows = [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines()]
    # Priority events can overtake ordinary motion events in the asynchronous log.
    for row in sorted(rows, key=lambda item: item.get("t", 0)):
        event = row.get("type")
        if event == "payload_detour_stage_start":
            active = {"stage": row["stage"], "start_s": row["t"],
                      "args": row.get("args"), "samples": [], "phase_changes": []}
            stages.append(active)
        elif active is not None and event == "motion_action":
            samples = active["samples"]
            if not samples or row.get("phase") != samples[-1].get("phase"):
                active["phase_changes"].append({
                    "t": row["t"], "phase": row.get("phase"),
                    "diagnostics": row.get("diagnostics"), "command": row.get("command")})
            samples.append(row)
        elif active is not None and event == "payload_detour_stage_done":
            active["end_s"] = row["t"]
            active["pose"] = row["pose"]
            active = None
    for stage in stages:
        samples = stage.pop("samples")
        stage["sample_count"] = len(samples)
        if samples:
            stage["last_motion"] = samples[-1]
            stage["max_heading_drift_deg"] = max(
                abs(math.degrees(s.get("diagnostics", {}).get("heading_drift_rad", 0)))
                for s in samples)
    return stages


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events", type=Path)
    args = parser.parse_args()
    print(json.dumps(summarize(args.events), ensure_ascii=False, indent=2))
