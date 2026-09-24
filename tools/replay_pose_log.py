"""Replay canonical T265/D500 JSONL pose events through the current fusion config."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))

from components.pose_fusion import PoseFusion
from components.pose_log_replay import read_pose_events, replay_fusion
from config.v2_loader import load_v2_config


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="input canonical event JSONL")
    parser.add_argument("--config", help="schema-v2 configuration")
    parser.add_argument("--output", help="output fused-pose JSONL; defaults to stdout")
    args = parser.parse_args(argv)
    events = read_pose_events(args.input)
    config = load_v2_config(args.config)
    estimates = replay_fusion(events, PoseFusion(config.fusion))
    output = None if args.output is None else Path(args.output).open("w", encoding="utf-8")
    try:
        for timestamp, estimate in estimates:
            row = {
                "t": timestamp,
                "type": "fused_pose",
                "x_m": None if estimate.pose is None else estimate.pose.x_m,
                "y_m": None if estimate.pose is None else estimate.pose.y_m,
                "yaw_rad": None if estimate.pose is None else estimate.pose.yaw_rad,
                "state": estimate.state.value,
                "d500_accepted": estimate.d500_accepted,
                "d500_innovation_m": estimate.last_d500_innovation_m,
                "d500_innovation_yaw_rad": estimate.last_d500_innovation_yaw_rad,
                "rejection_reason": estimate.rejection_reason,
            }
            line = json.dumps(row, separators=(",", ":"))
            print(line) if output is None else output.write(line + "\n")
    finally:
        if output is not None:
            output.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
