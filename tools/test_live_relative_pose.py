#!/usr/bin/env python3
"""Print sensor-only fused base_link pose relative to the stable startup pose at 2 Hz.

Start the repository's SLAM ROS launch separately, then run this tool with the
board TOML. Hold the vehicle still at P0 until the ten-second preflight passes.
No drive or relay port is opened; all output files stay outside the checkout.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "code"))

from components.diagnostics_log import JsonlEventLogger  # noqa: E402
from config.v2_loader import load_v2_config  # noqa: E402
from config.v2_runtime import RuntimeMode  # noqa: E402
from robocup_runtime import build_runtime  # noqa: E402
from slam_manual_accuracy import (  # noqa: E402
    AccuracySession, CausalT265Source, ScanOnlyD500Source, percentile,
    relative_delta, test_config_from_board,
)

PRINT_PERIOD_S = 0.5


def live_pose(session: AccuracySession, origin, now_s: float) -> str:
    """Return a fresh relative pose or an explicit invalid-state line."""
    recent = session._recent(2.0)
    reasons = session._health_reasons(recent)
    if recent and now_s - recent[-1].t_s > 0.3:
        reasons.append("fused pose stale")
    tf_ages = [s.tf_age_ms for s in recent if s.tf_age_ms is not None]
    if percentile(tf_ages, 0.95) >= 30.0:
        reasons.append("T265/scan TF alignment p95 >= 30 ms")
    scan_hz = 0.0
    if len(recent) >= 2:
        scan_hz = (recent[-1].scan_count - recent[0].scan_count) / max(
            0.001, recent[-1].t_s - recent[0].t_s
        )
    if scan_hz < 3.5:
        reasons.append("scan publication below 3.5 Hz")
    if session.logger.write_error or session.logger.dropped_events:
        reasons.append("event log write failed or dropped events")
    if reasons:
        return "INVALID " + "; ".join(reasons)
    current = recent[-1]
    dx_m, dy_m, dyaw_rad = relative_delta(
        origin.x_m, origin.y_m, origin.yaw_unwrapped_rad,
        current.x_m, current.y_m, current.yaw_unwrapped_rad,
    )
    return ("x_cm=%+.2f y_cm=%+.2f yaw_deg=%+.2f "
            "T265_conf=%.2f scan_hz=%.2f" % (
                dx_m * 100.0, dy_m * 100.0, math.degrees(dyaw_rad),
                current.t265_confidence, scan_hz,
            ))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="existing board TOML; read only")
    parser.add_argument("--output", help="new log directory outside the Git checkout")
    parser.add_argument("--seconds", type=float, help="optional positive run duration")
    args = parser.parse_args(argv)
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    output = (Path(args.output) if args.output else
              Path.home() / "car_test_logs" / time.strftime("live_relative_pose_%Y%m%d_%H%M%S"))
    output = output.resolve()
    if output.exists():
        parser.error("output directory already exists; choose a new run name")
    if output == REPO_ROOT or REPO_ROOT in output.parents:
        parser.error("output must be outside the Git checkout")
    config = test_config_from_board(load_v2_config(args.config))
    output.mkdir(parents=True)
    logger = JsonlEventLogger(output / "events.jsonl")
    runtime = None
    session = None
    try:
        runtime = build_runtime(config, RuntimeMode.HARDWARE_PROBE,
                                event_logger=logger, sensor_only=True)
        runtime.t265_source = CausalT265Source(runtime.t265_source)
        runtime.d500_source = ScanOnlyD500Source(
            runtime, config.d500.port, config.d500.baudrate, logger,
        )
        session = AccuracySession(runtime, logger, output)
        session.start()
        print("SENSOR-ONLY 2 Hz: hold at P0 until ORIGIN is set; Ctrl+C stops.", flush=True)
        origin = None
        next_tick = time.monotonic()
        deadline = None if args.seconds is None else next_tick + args.seconds
        while deadline is None or time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
                now = time.monotonic()
            next_tick = max(next_tick + PRINT_PERIOD_S, now + PRINT_PERIOD_S)
            if session.error is not None:
                raise RuntimeError("sampler failed: " + session.error)
            if origin is None:
                ready, reasons = session.preflight()
                if not ready:
                    print("WAIT_ORIGIN " + "; ".join(reasons[:3]), flush=True)
                    continue
                origin = session._stationary_snapshot(session._recent(2.0))
                print("ORIGIN set at stable base_link pose; +X forward, +Y left, +yaw CCW",
                      flush=True)
            print(live_pose(session, origin, now), flush=True)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            if session is not None:
                session.close()
            elif runtime is not None:
                runtime.close()
        finally:
            logger.close()
            print("Logs: %s" % output, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
