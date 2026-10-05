#!/usr/bin/env python3
"""Report the D500/slam_toolbox localization-chain counters for a run.

Reads ``events.jsonl`` (plus ``manifest.json`` / ``summary.json`` when present)
from one run directory and prints the counters the TF-starvation fix is judged
on, then a verdict per acceptance criterion that can be decided from a log.

Usage:
    python3 tools/localization_chain_report.py RUN_DIR [RUN_DIR ...]
    python3 tools/localization_chain_report.py --compare BEFORE AFTER

A run directory is anything containing ``events.jsonl``:
``/home/radxa/car_test_logs/<run>/`` or a nested action directory such as
``<run>/fwd20/``.  Pass ``--json`` for a machine-readable summary.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

# Counters copied out of every slam_status event.
COUNTER_FIELDS = (
    "scan_input_count", "scan_publish_count", "scan_drop_count",
    "scan_drop_no_tf_window_count", "tf_starved_force_count",
    "tf_lookup_fail_count", "pose_received_count", "loop_closure_count",
    "bridge_queue_overwrite_count",
)
STAMP_FIELDS = ("first_tf_stamp_s", "last_tf_stamp_s", "no_window_measurement_s")
MIN_SCAN_PUBLISH_HZ = 8.0
STATIONARY_OK_WINDOW_S = 10.0


def load_events(path: Path) -> list[dict]:
    events: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(event, dict):
                events.append(event)
    return events


def _num(value, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _slam_key(event: dict, field: str):
    return event.get("slam." + field, event.get(field))


def summarize(run: Path) -> dict:
    events_path = run / "events.jsonl" if run.is_dir() else run
    run_dir = events_path.parent
    events = load_events(events_path)
    slam = [event for event in events if event.get("type") == "slam_status"]
    fused = [event for event in events if event.get("type") == "fused_pose"]
    t265 = [event for event in events if event.get("type") == "t265_pose"]

    result: dict = {"run": run_dir.name, "path": str(run_dir), "events": len(events),
                    "slam_status_samples": len(slam)}
    if not slam:
        result["note"] = "no slam_status samples; is the SLAM sidecar running?"
        return result

    first = slam[0]
    last = slam[-1]
    elapsed = max(0.0, _num(last.get("t")) - _num(first.get("t")))

    stamps: dict = {}
    for field in STAMP_FIELDS:
        values = [_slam_key(event, field) for event in slam]
        present = [value for value in values if value is not None]
        stamps[field] = {
            "first_non_null": present[0] if present else None,
            "last_non_null": present[-1] if present else None,
            "null_samples": sum(1 for value in values if value is None),
            "null_after_first_non_null": (
                sum(1 for value in values[values.index(present[0]):] if value is None)
                if present else len(values)
            ),
        }
    result["tf_window"] = stamps

    counters: dict = {}
    for field in COUNTER_FIELDS:
        start = _num(_slam_key(first, field))
        end = _num(_slam_key(last, field))
        counters[field] = {"start": start, "end": end, "delta": end - start}
    result["counters"] = counters
    result["duration_s"] = round(elapsed, 3)
    result["scan_publish_hz"] = round(
        counters["scan_publish_count"]["delta"] / elapsed if elapsed > 0 else 0.0, 3)
    result["scan_input_hz"] = round(
        counters["scan_input_count"]["delta"] / elapsed if elapsed > 0 else 0.0, 3)

    states_seen: dict = {}
    for event in slam:
        states_seen[str(event.get("state"))] = states_seen.get(str(event.get("state")), 0) + 1
    result["slam"] = {
        "states": states_seen,
        "failed_samples": states_seen.get("SLAM_FAILED", 0),
        "last_state": str(last.get("state")),
    }

    states: dict = {}
    for event in fused:
        states[str(event.get("state"))] = states.get(str(event.get("state")), 0) + 1
    result["fused"] = {
        "samples": len(fused),
        "states": states,
        "ok_samples": states.get("ok", 0),
        "longest_ok_s": _longest_ok_run(fused),
        "source_flags": sorted({tuple(event.get("source_flags") or ()) for event in fused
                                if event.get("state") == "ok"}),
    }
    confidences = [_num(event.get("confidence"), float("nan")) for event in t265]
    confidences = [value for value in confidences if math.isfinite(value)]
    result["t265"] = {
        "samples": len(t265),
        "min_confidence": min(confidences) if confidences else None,
    }
    result["verdict"] = _verdict(result)
    return result


def _longest_ok_run(fused: list[dict]) -> float:
    best = 0.0
    start_s: float | None = None
    previous_s: float | None = None
    for event in fused:
        when = _num(event.get("t"))
        if event.get("state") == "ok":
            if start_s is None:
                start_s = when
            previous_s = when
            best = max(best, previous_s - start_s)
        else:
            start_s = None
    return round(best, 3)


def _verdict(result: dict) -> dict:
    verdict: dict = {}
    window = result["tf_window"]["first_tf_stamp_s"]
    # A dead bridge freezes its counters, so a frozen non-null stamp must not be
    # able to pass the "never starved" criterion on its own.
    bridge_alive = result["slam"]["failed_samples"] == 0
    verdict["bridge_alive"] = bridge_alive
    verdict["tf_never_starved"] = bool(
        bridge_alive and window["first_non_null"] is not None
        and window["null_after_first_non_null"] == 0)
    verdict["scan_publish_hz_ok"] = result["scan_publish_hz"] >= MIN_SCAN_PUBLISH_HZ
    fused = result["fused"]
    verdict["stationary_preflight_ok"] = (
        fused["longest_ok_s"] >= STATIONARY_OK_WINDOW_S
        and any({"t265", "slam", "fused"} <= set(flags) for flags in fused["source_flags"]))
    verdict["watchdog_fired"] = result["counters"]["tf_starved_force_count"]["delta"] > 0
    verdict["no_window_drops"] = result["counters"]["scan_drop_no_tf_window_count"]["delta"]
    return verdict


def _print(result: dict) -> None:
    print("== %s ==" % result["path"])
    if "note" in result:
        print("  " + result["note"])
        return
    print("  slam_status samples=%d duration=%.1fs scan_input=%.2f/s scan_publish=%.2f/s"
          % (result["slam_status_samples"], result["duration_s"],
             result["scan_input_hz"], result["scan_publish_hz"]))
    print("  bridge states=%s last=%s" % (result["slam"]["states"], result["slam"]["last_state"]))
    for field in STAMP_FIELDS:
        entry = result["tf_window"][field]
        print("  %-28s first=%s last=%s null=%d null_after_first=%d"
              % (field, _fmt(entry["first_non_null"]), _fmt(entry["last_non_null"]),
                 entry["null_samples"], entry["null_after_first_non_null"]))
    for field in COUNTER_FIELDS:
        entry = result["counters"][field]
        print("  %-28s %s -> %s (+%s)"
              % (field, _fmt(entry["start"]), _fmt(entry["end"]), _fmt(entry["delta"])))
    fused = result["fused"]
    print("  fused samples=%d states=%s longest_ok=%.1fs flags=%s"
          % (fused["samples"], fused["states"], fused["longest_ok_s"],
             [list(flags) for flags in fused["source_flags"]]))
    print("  t265 samples=%s" % result["t265"]["samples"])
    for name, value in result["verdict"].items():
        print("  VERDICT %-24s %s" % (name, value))


def _fmt(value) -> str:
    if value is None:
        return "null"
    if isinstance(value, float):
        return ("%.3f" % value).rstrip("0").rstrip(".")
    return str(value)


def _compare(before: dict, after: dict) -> None:
    print("== before/after ==")
    print("  %-28s %-14s %-14s" % ("counter", "before", "after"))
    for field in COUNTER_FIELDS:
        left = before.get("counters", {}).get(field, {}).get("delta")
        right = after.get("counters", {}).get(field, {}).get("delta")
        print("  %-28s %-14s %-14s" % (field, _fmt(left), _fmt(right)))
    print("  %-28s %-14s %-14s" % (
        "first_tf_stamp_s",
        _fmt(before.get("tf_window", {}).get("first_tf_stamp_s", {}).get("first_non_null")),
        _fmt(after.get("tf_window", {}).get("first_tf_stamp_s", {}).get("first_non_null")),
    ))
    print("  %-28s %-14s %-14s" % (
        "scan_publish_hz", _fmt(before.get("scan_publish_hz")), _fmt(after.get("scan_publish_hz"))))
    print("  %-28s %-14s %-14s" % (
        "longest_ok_s",
        _fmt(before.get("fused", {}).get("longest_ok_s")),
        _fmt(after.get("fused", {}).get("longest_ok_s")),
    ))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="*", help="run directories containing events.jsonl")
    parser.add_argument("--compare", nargs=2, metavar=("BEFORE", "AFTER"),
                        help="print a before/after counter table")
    parser.add_argument("--json", action="store_true", help="emit JSON instead of text")
    args = parser.parse_args(argv)
    if args.compare:
        before, after = (summarize(Path(item)) for item in args.compare)
        if args.json:
            print(json.dumps({"before": before, "after": after}, ensure_ascii=False, indent=2))
        else:
            _print(before)
            _print(after)
            _compare(before, after)
        return 0
    if not args.runs:
        parser.error("give at least one run directory, or --compare BEFORE AFTER")
    results = [summarize(Path(item)) for item in args.runs]
    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        for result in results:
            _print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
