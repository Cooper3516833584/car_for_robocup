#!/usr/bin/env python3
"""Print sensor-only fused base_link pose relative to the stable startup pose at 2 Hz.

Start the repository's SLAM ROS launch separately, then run this tool with the
board TOML. Hold the vehicle still at P0 until the ten-second preflight passes.
No drive or relay port is opened; all output files stay outside the checkout.

``--seconds`` is a hard wall-clock budget for the whole process, including the
stationary preflight and the shutdown path.  A broken localization chain blocks
``pipeline.stop()`` on the T265 and can hold the accuracy sampler's sample lock,
so neither the preflight poll nor the close runs on the main thread's critical
path: a wedged call must cost a warning, never the operator's time budget.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import threading
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(REPO_ROOT / "code"))
sys.path.insert(0, str(TOOLS_DIR))

from components.diagnostics_log import JsonlEventLogger  # noqa: E402
from config.v2_loader import load_v2_config  # noqa: E402
from config.v2_runtime import RuntimeMode  # noqa: E402
from robocup_runtime import build_runtime  # noqa: E402
from slam_manual_accuracy import (  # noqa: E402
    AccuracySession, CausalT265Source, ScanOnlyD500Source, percentile,
    relative_delta, test_config_from_board,
)

PRINT_PERIOD_S = 0.5
PREFLIGHT_POLL_S = 0.25
PREFLIGHT_STALE_S = 2.0
DEFAULT_SHUTDOWN_TIMEOUT_S = 15.0


class BoundedPreflight:
    """Poll ``session.preflight()`` off the main loop.

    ``preflight()`` only reads the sample buffer, but the accuracy sampler holds
    the sample lock while it works and a stalled sensor read used to stall the
    whole diagnostic.  The probe thread keeps the last result; a stale result
    means the probe itself is wedged and the caller is told so instead of
    blocking.
    """

    def __init__(self, session, *, period_s: float = PREFLIGHT_POLL_S,
                 stale_after_s: float = PREFLIGHT_STALE_S) -> None:
        self._session = session
        self._period_s = period_s
        self._stale_after_s = stale_after_s
        self._lock = threading.Lock()
        self._result: tuple[bool, list[str]] = (False, ["preflight pending"])
        self._updated_s: float | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="bounded-preflight", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                result = self._session.preflight()
            except Exception as exc:  # noqa: BLE001 - reported, never fatal
                result = (False, ["preflight failed: %s: %s" % (type(exc).__name__, exc)])
            with self._lock:
                self._result = result
                self._updated_s = time.monotonic()
            self._stop.wait(self._period_s)

    def latest(self) -> tuple[bool, list[str]]:
        with self._lock:
            result, updated = self._result, self._updated_s
        if updated is None or time.monotonic() - updated > self._stale_after_s:
            return False, ["preflight did not return within %.1f s" % self._stale_after_s]
        return result


def run_bounded(call, timeout_s: float) -> tuple[bool, object]:
    """Run ``call`` on a daemon thread; return (finished, value).

    A call that outlives the budget is abandoned (the thread is a daemon, so it
    cannot keep the interpreter alive) and the caller learns it did not finish.
    """
    box: dict = {}

    def worker() -> None:
        try:
            box["value"] = call()
        except BaseException as exc:  # noqa: BLE001 - re-raised to the caller
            box["error"] = exc

    thread = threading.Thread(target=worker, name="bounded-call", daemon=True)
    thread.start()
    thread.join(timeout=max(0.0, float(timeout_s)))
    if thread.is_alive():
        return False, None
    if "error" in box:
        raise box["error"]
    return True, box.get("value")


def close_session(session, runtime, logger) -> None:
    try:
        if session is not None:
            session.close()
        elif runtime is not None:
            runtime.close()
    finally:
        logger.close()


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
    parser.add_argument("--seconds", type=float,
                        help="hard total run duration, including preflight and shutdown")
    parser.add_argument("--shutdown-timeout", type=float, default=DEFAULT_SHUTDOWN_TIMEOUT_S,
                        help="maximum seconds spent closing sensors and logs")
    args = parser.parse_args(argv)
    if args.seconds is not None and (not math.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    if not math.isfinite(args.shutdown_timeout) or args.shutdown_timeout <= 0:
        parser.error("--shutdown-timeout must be finite and positive")
    started_s = time.monotonic()
    hard_deadline = None if args.seconds is None else started_s + args.seconds
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
    preflight = None
    try:
        runtime = build_runtime(config, RuntimeMode.HARDWARE_PROBE,
                                event_logger=logger, sensor_only=True)
        runtime.t265_source = CausalT265Source(runtime.t265_source)
        runtime.d500_source = ScanOnlyD500Source(
            runtime, config.d500.port, config.d500.baudrate, logger,
        )
        session = AccuracySession(runtime, logger, output)
        session.start()
        preflight = BoundedPreflight(session)
        preflight.start()
        print("SENSOR-ONLY 2 Hz: hold at P0 until ORIGIN is set; Ctrl+C stops.", flush=True)
        origin = None
        next_tick = time.monotonic()
        while hard_deadline is None or time.monotonic() < hard_deadline:
            now = time.monotonic()
            if now < next_tick:
                time.sleep(next_tick - now)
                now = time.monotonic()
            next_tick = max(next_tick + PRINT_PERIOD_S, now + PRINT_PERIOD_S)
            if session.error is not None:
                raise RuntimeError("sampler failed: " + session.error)
            if origin is None:
                ready, reasons = preflight.latest()
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
        if preflight is not None:
            preflight.stop()
        remaining = args.shutdown_timeout
        if hard_deadline is not None:
            remaining = min(remaining, max(1.0, hard_deadline - time.monotonic()))
        try:
            finished, _ = run_bounded(lambda: close_session(session, runtime, logger), remaining)
        except Exception as exc:  # noqa: BLE001 - shutdown must not mask the result
            finished = True
            print("WARNING: shutdown reported %s: %s" % (type(exc).__name__, exc), flush=True)
        if not finished:
            print("WARNING: shutdown did not finish within %.1f s; exiting anyway" % remaining,
                  flush=True)
        print("Logs: %s" % output, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
