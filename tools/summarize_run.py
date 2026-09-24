"""Print compact health and motion statistics from a RoboCup event JSONL."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics


def summarize(path: str | Path) -> dict[str, float | int | None]:
    events = [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines() if line.strip()]
    times = [float(row["t"]) for row in events if isinstance(row.get("t"), (int, float))]
    fused = [row for row in events if row.get("type") == "fused_pose"]
    d500 = [row for row in fused if row.get("d500_innovation_m") is not None]
    drive = [row for row in events if row.get("type") == "drive_command"]
    speeds = [abs(float(row.get("requested_v_m_s", 0.0))) for row in drive]
    d500_rejects = sum(row.get("d500_accepted") is False for row in fused)
    d500_observations = sum(row.get("type") == "d500_pose" for row in events) + sum(
        row.get("type") == "d500_rejected" for row in events
    )
    nav = [row for row in events if row.get("type") == "navigation"]
    nav_start = next((float(row["t"]) for row in nav if row.get("goal") is not None), None)
    nav_end = next((float(row["t"]) for row in nav if row.get("state") == "goal_reached"), None)
    lost_times = [float(row["t"]) for row in fused if row.get("state") == "lost"]
    return {
        "duration_s": max(times) - min(times) if times else 0.0,
        "pose_lost_count": sum(row.get("state") == "lost" for row in fused),
        "pose_lost_duration_s": max(0.0, lost_times[-1] - lost_times[0]) if len(lost_times) > 1 else 0.0,
        "d500_reject_count": d500_rejects,
        "d500_reject_ratio": d500_rejects / d500_observations if d500_observations else None,
        "d500_innovation_mean_m": statistics.fmean(float(row["d500_innovation_m"]) for row in d500) if d500 else None,
        "d500_innovation_max_m": max((float(row["d500_innovation_m"]) for row in d500), default=None),
        "t265_low_confidence_count": sum(
            row.get("type") == "t265_rejected" and row.get("reason") == "tracker_confidence_too_low"
            for row in events
        ),
        "max_commanded_speed_m_s": max(speeds, default=0.0),
        "c10b_reject_count": sum(row.get("type") == "c10b_reject" for row in events),
        "goal_settle_time_s": nav_end - nav_start if nav_start is not None and nav_end is not None else None,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="RoboCup events JSONL")
    args = parser.parse_args(argv)
    print(json.dumps(summarize(args.input), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
