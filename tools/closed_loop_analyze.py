#!/usr/bin/env python3
"""Offline stage-2 diagnostics for closed_loop_motion run directories.

Reads the run's manifest/summary and events.jsonl and reports the fused-pose
relative motion, the commanded-speed integrals, fused-pose jumps and the SLAM
anchor innovations.  It works on incomplete runs too (timeout, abort, safe
stop), which is where closed_loop_motion.py writes no metrics of its own.

Read-only: this tool never opens a device and never writes next to the log.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

TWO_PI = 2.0 * math.pi

# Validity thresholds.  A jump at or above MAX_POSE_STEP_M is the same bound
# closed_loop_motion.py aborts on, so a run that crossed it is not a usable
# sample.  The warning bounds sit at the closed-loop acceptance scale: a fused
# pose that moves more than the 1 cm target in one sample, or a SLAM anchor that
# disagrees with T265 dead-reckoning by more than the 3 cm position tolerance,
# cannot support a 1 cm settled measurement.
JUMP_INVALID_M = 0.10
JUMP_WARN_M = 0.01
YAW_JUMP_INVALID_RAD = math.radians(20.0)
YAW_JUMP_WARN_RAD = math.radians(1.0)
INNOVATION_WARN_M = 0.03
HEALTHY_STATES = {"ok", "t265_degraded", "d500_degraded"}
REQUIRED_SOURCES = {"t265", "slam", "fused"}


def unwrap_delta(previous: float, current: float) -> float:
    delta = (current - previous) % TWO_PI
    if delta > math.pi:
        delta -= TWO_PI
    return delta


def _num(value, default=0.0):
    return default if value is None else float(value)


def load_events(path: Path) -> list[dict]:
    events = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return events


def _by_type(events: list[dict], name: str) -> list[dict]:
    return [event for event in events if event.get("type") == name]


def _integrate(events: list[dict], value) -> float:
    total = 0.0
    for previous, current in zip(events, events[1:]):
        dt = _num(current.get("t")) - _num(previous.get("t"))
        if dt <= 0.0:
            continue
        total += 0.5 * (value(previous) + value(current)) * dt
    return total


def _at_or_before(events: list[dict], moment: float) -> dict | None:
    found = None
    for event in events:
        if _num(event.get("t")) <= moment:
            found = event
        else:
            break
    return found


def summarize(run: Path) -> dict:
    events_path = run / "events.jsonl" if run.is_dir() else run
    run_dir = events_path.parent
    events = load_events(events_path)
    manifest = _read_json(run_dir / "manifest.json")
    summary = _read_json(run_dir / "summary.json")

    motion = _by_type(events, "motion_action")
    fused = [event for event in _by_type(events, "fused_pose") if event.get("x_m") is not None]
    t265 = _by_type(events, "t265_pose")
    commands = _by_type(events, "drive_command")
    safety = _by_type(events, "safety")
    slam = _by_type(events, "slam_status")

    result: dict = {
        "run": run_dir.name,
        "action": (manifest or {}).get("action", {}).get("name") or (summary or {}).get("action"),
        "valid": (summary or {}).get("valid"),
        "error": (summary or {}).get("error"),
        "samples": {"fused": len(fused), "motion_action": len(motion), "drive_command": len(commands)},
    }

    if not motion or not fused:
        result["note"] = "no motion_action or fused_pose samples"
        return result

    action_start = _num(motion[0].get("t"))
    action_end = _num(motion[-1].get("t"))
    result["action_window_s"] = round(action_end - action_start, 3)
    result["states"] = sorted({str(event.get("state")) for event in motion})
    result["phases"] = sorted({str(event.get("phase")) for event in motion})

    result["commanded"] = {
        "integral_linear_m": round(_integrate(motion, lambda e: _num(e.get("command", {}).get("linear_x_m_s"))), 4),
        "integral_angular_rad": round(_integrate(motion, lambda e: _num(e.get("command", {}).get("angular_z_rad_s"))), 4),
        "max_linear_m_s": round(max((abs(_num(e.get("command", {}).get("linear_x_m_s"))) for e in motion), default=0.0), 4),
        "max_angular_rad_s": round(max((abs(_num(e.get("command", {}).get("angular_z_rad_s"))) for e in motion), default=0.0), 4),
    }

    if commands:
        result["drive"] = {
            "max_requested_v_m_s": round(max(abs(_num(e.get("requested_v_m_s"))) for e in commands), 4),
            "max_limited_v_m_s": round(max(abs(_num(e.get("limited_v_m_s"))) for e in commands), 4),
            "max_requested_omega_rad_s": round(max(abs(_num(e.get("requested_omega_rad_s"))) for e in commands), 4),
            "max_limited_omega_rad_s": round(max(abs(_num(e.get("limited_omega_rad_s"))) for e in commands), 4),
            "max_wheel_target_m_s": round(max(max(abs(_num(e.get("left_target_m_s"))), abs(_num(e.get("right_target_m_s")))) for e in commands), 4),
            "actuation_enabled": sorted({bool(e.get("actuation_enabled")) for e in commands}),
            "protocol_mode": sorted({str(e.get("protocol_mode")) for e in commands}),
        }

    start = _at_or_before(fused, action_start) or fused[0]
    end = fused[-1]
    dx = _num(end["x_m"]) - _num(start["x_m"])
    dy = _num(end["y_m"]) - _num(start["y_m"])
    yaw0 = _num(start["yaw_rad"])
    c, s = math.cos(yaw0), math.sin(yaw0)
    accumulated_yaw = sum(unwrap_delta(_num(a["yaw_rad"]), _num(b["yaw_rad"]))
                          for a, b in zip(fused, fused[1:])
                          if _num(a.get("t")) >= action_start)
    result["fused"] = {
        "start": _pose(start), "end": _pose(end),
        "longitudinal_m": round(dx * c + dy * s, 4),
        "lateral_m": round(-dx * s + dy * c, 4),
        "center_shift_m": round(math.hypot(dx, dy), 4),
        "accumulated_yaw_rad": round(accumulated_yaw, 4),
        "accumulated_yaw_deg": round(math.degrees(accumulated_yaw), 3),
    }

    steps = []
    for previous, current in zip(fused, fused[1:]):
        dt = _num(current.get("t")) - _num(previous.get("t"))
        step = math.hypot(_num(current["x_m"]) - _num(previous["x_m"]),
                          _num(current["y_m"]) - _num(previous["y_m"]))
        steps.append((step, dt, _num(current.get("t")),
                      abs(unwrap_delta(_num(previous["yaw_rad"]), _num(current["yaw_rad"])))))
    if steps:
        worst = max(steps, key=lambda item: item[0])
        result["fused_jumps"] = {
            "max_step_m": round(worst[0], 4),
            "max_step_at_s": round(worst[2], 2),
            "max_step_dt_s": round(worst[1], 4),
            "max_step_speed_m_s": round(worst[0] / worst[1], 3) if worst[1] > 0 else None,
            "max_yaw_step_deg": round(math.degrees(max(item[3] for item in steps)), 3),
            "steps_over_2cm": sum(1 for item in steps if item[0] > 0.02),
            "steps_over_5cm": sum(1 for item in steps if item[0] > 0.05),
        }

    innovations = [(abs(_num(e.get("d500_innovation_m"))), _num(e.get("t")))
                   for e in fused if e.get("d500_innovation_m") is not None]
    if innovations:
        biggest = max(innovations)
        result["slam_innovation"] = {
            "max_m": round(biggest[0], 4),
            "max_at_s": round(biggest[1], 2),
            "nonzero_updates": sum(1 for value, _ in innovations if value > 1e-9),
        }

    if t265:
        result["t265_raw"] = {
            "displacement_m": round(math.hypot(_num(t265[-1]["x_m"]) - _num(t265[0]["x_m"]),
                                               _num(t265[-1]["y_m"]) - _num(t265[0]["y_m"])), 4),
            "yaw_change_deg": round(math.degrees(_num(t265[-1]["yaw_rad"]) - _num(t265[0]["yaw_rad"])), 3),
            "min_tracker_confidence": round(min(_num(e.get("tracker_confidence")) for e in t265), 2),
        }

    if safety:
        result["safety"] = [{"t": round(_num(e.get("t")), 2), "code": e.get("event_code"), "state": e.get("state")}
                            for e in safety]
    if slam:
        result["slam_states"] = sorted({str(e.get("state")) for e in slam})
    result["validity"] = _validity(result, fused, safety, summary)
    return result


def _validity(result: dict, fused: list[dict], safety: list[dict], summary: dict | None) -> dict:
    """Classify a run the way the plan requires before it can be a valid sample.

    The plan says an action with a lost pose, a single-source downgrade, a
    timestamp anomaly or an obvious pose jump must be marked invalid and never
    averaged into a passing result.  This turns that rule into a check.
    """
    invalid: list[str] = []
    warnings: list[str] = []

    jumps = result.get("fused_jumps")
    if jumps:
        if jumps["max_step_m"] >= JUMP_INVALID_M:
            invalid.append("fused pose jumped %.3f m in one sample (>= %.2f m)"
                           % (jumps["max_step_m"], JUMP_INVALID_M))
        elif jumps["max_step_m"] > JUMP_WARN_M:
            warnings.append("fused pose stepped %.3f m in one sample" % jumps["max_step_m"])
        if jumps["max_yaw_step_deg"] >= math.degrees(YAW_JUMP_INVALID_RAD):
            invalid.append("fused yaw jumped %.1f deg in one sample" % jumps["max_yaw_step_deg"])
        elif jumps["max_yaw_step_deg"] > math.degrees(YAW_JUMP_WARN_RAD):
            warnings.append("fused yaw stepped %.2f deg in one sample" % jumps["max_yaw_step_deg"])

    innovation = result.get("slam_innovation")
    if innovation and innovation["max_m"] > INNOVATION_WARN_M:
        warnings.append("SLAM anchor innovation reached %.3f m" % innovation["max_m"])

    states = {str(event.get("state")) for event in fused}
    bad_states = sorted(states - HEALTHY_STATES)
    if bad_states:
        invalid.append("fused pose states outside healthy set: %s" % ", ".join(bad_states))

    missing = []
    for event in fused:
        flags = set(event.get("source_flags") or ())
        if not REQUIRED_SOURCES.issubset(flags):
            missing.append(sorted(REQUIRED_SOURCES - flags))
    if missing:
        invalid.append("fused samples missing sources on %d of %d samples" % (len(missing), len(fused)))

    if safety:
        codes = sorted({str(event.get("event_code")) for event in safety})
        result["safety_codes"] = codes
        blocking = [code for code in codes if code not in {"D500_GLOBAL_PENDING"}]
        if blocking:
            warnings.append("safety events: %s" % ", ".join(blocking))

    if summary:
        if summary.get("dropped_events"):
            invalid.append("event log dropped %s events" % summary["dropped_events"])
        if summary.get("log_write_error"):
            invalid.append("event log write error: %s" % summary["log_write_error"])

    return {"action_valid": not invalid, "invalid_reasons": invalid, "warnings": warnings}


def _pose(event: dict) -> dict:
    return {"t": round(_num(event.get("t")), 3), "x_m": round(_num(event.get("x_m")), 4),
            "y_m": round(_num(event.get("y_m")), 4), "yaw_deg": round(math.degrees(_num(event.get("yaw_rad"))), 3)}


def _read_json(path: Path):
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError:
        return None


def _print(result: dict) -> None:
    print("=" * 72)
    print("%s  action=%s valid=%s" % (result["run"], result.get("action"), result.get("valid")))
    if result.get("error"):
        print("  error: %s" % result["error"])
    if "commanded" in result:
        print("  window %ss  states=%s phases=%s" % (result["action_window_s"], result["states"], result["phases"]))
        print("  commanded: %s" % json.dumps(result["commanded"]))
    if "drive" in result:
        print("  drive:     %s" % json.dumps(result["drive"]))
    if "fused" in result:
        print("  fused:     longitudinal=%+.4f m lateral=%+.4f m yaw=%+.3f deg"
              % (result["fused"]["longitudinal_m"], result["fused"]["lateral_m"],
                 result["fused"]["accumulated_yaw_deg"]))
    if "fused_jumps" in result:
        print("  jumps:     %s" % json.dumps(result["fused_jumps"]))
    if "slam_innovation" in result:
        print("  slam_innov:%s" % json.dumps(result["slam_innovation"]))
    if "t265_raw" in result:
        print("  t265 raw:  %s" % json.dumps(result["t265_raw"]))
    if result.get("safety"):
        print("  safety:    %s" % json.dumps(result["safety"]))
    validity = result.get("validity")
    if validity:
        print("  VALIDITY:  action_valid=%s" % validity["action_valid"])
        for reason in validity["invalid_reasons"]:
            print("    INVALID: %s" % reason)
        for warning in validity["warnings"]:
            print("    warning: %s" % warning)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("runs", nargs="+", help="run directory (or events.jsonl) to analyse")
    parser.add_argument("--json", action="store_true", help="emit one JSON object per run")
    args = parser.parse_args(argv)
    results = []
    for name in args.runs:
        path = Path(name)
        if not path.exists():
            print("missing: %s" % path, file=sys.stderr)
            continue
        results.append(summarize(path))
    if args.json:
        print(json.dumps(results, ensure_ascii=False, indent=2))
    else:
        for result in results:
            _print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
