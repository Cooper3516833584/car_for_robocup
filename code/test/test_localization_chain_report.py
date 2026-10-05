"""Hardware-free checks for the localization-chain counter report tool."""

from __future__ import annotations

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from tools import localization_chain_report as report  # noqa: E402


def _slam(t_s: float, *, first_tf, last_tf, publish: int, no_window: int = 0,
          starved: int = 0, inputs: int = 0, lookup_fail: int = 0,
          state: str = "SLAM_OK") -> dict:
    return {
        "t": t_s, "type": "slam_status", "state": state,
        "slam.first_tf_stamp_s": first_tf,
        "slam.last_tf_stamp_s": last_tf,
        "slam.no_window_measurement_s": None,
        "slam.scan_input_count": inputs,
        "slam.scan_publish_count": publish,
        "slam.scan_drop_count": 0,
        "slam.scan_drop_no_tf_window_count": no_window,
        "slam.tf_starved_force_count": starved,
        "slam.tf_lookup_fail_count": lookup_fail,
        "slam.pose_received_count": publish,
        "slam.loop_closure_count": 0,
        "slam.bridge_queue_overwrite_count": 0,
    }


def _fused(t_s: float, state: str, flags) -> dict:
    return {"t": t_s, "type": "fused_pose", "state": state, "source_flags": list(flags),
            "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}


def _write_run(directory: Path, events: list[dict]) -> Path:
    run = directory / "run"
    run.mkdir(parents=True, exist_ok=True)
    with (run / "events.jsonl").open("w", encoding="utf-8") as handle:
        for event in events:
            handle.write(json.dumps(event) + "\n")
    return run


class LocalizationChainReportTests(unittest.TestCase):
    def test_healthy_run_passes_every_log_decidable_criterion(self) -> None:
        events = [_slam(0.0, first_tf=100.0, last_tf=100.0, publish=0, inputs=0)]
        events += [_slam(float(t), first_tf=100.0, last_tf=100.0 + t, publish=t * 10, inputs=t * 12)
                   for t in range(1, 21)]
        events += [_fused(float(t), "ok", ("t265", "slam", "fused")) for t in range(0, 21)]
        with tempfile.TemporaryDirectory() as tmp:
            result = report.summarize(_write_run(Path(tmp), events))
        self.assertEqual(result["verdict"]["tf_never_starved"], True)
        self.assertEqual(result["verdict"]["scan_publish_hz_ok"], True)
        self.assertEqual(result["verdict"]["stationary_preflight_ok"], True)
        self.assertEqual(result["verdict"]["watchdog_fired"], False)

    def test_latched_lost_run_is_flagged(self) -> None:
        # Reproduces the verify01 signature: first_tf_stamp_s null forever and
        # scan_publish_count flat at zero.
        events = [_slam(float(t), first_tf=None, last_tf=None, publish=0,
                        no_window=t * 14, inputs=t * 14) for t in range(0, 21)]
        events += [_fused(float(t), "lost", ()) for t in range(0, 21)]
        with tempfile.TemporaryDirectory() as tmp:
            result = report.summarize(_write_run(Path(tmp), events))
        self.assertEqual(result["verdict"]["tf_never_starved"], False)
        self.assertEqual(result["verdict"]["scan_publish_hz_ok"], False)
        self.assertEqual(result["verdict"]["no_window_drops"], 14 * 20)

    def test_a_late_null_tf_stamp_breaks_the_first_criterion(self) -> None:
        events = [_slam(0.0, first_tf=None, last_tf=None, publish=0),
                  _slam(1.0, first_tf=10.0, last_tf=11.0, publish=5),
                  _slam(2.0, first_tf=None, last_tf=None, publish=5)]
        with tempfile.TemporaryDirectory() as tmp:
            result = report.summarize(_write_run(Path(tmp), events))
        self.assertEqual(result["tf_window"]["first_tf_stamp_s"]["first_non_null"], 10.0)
        self.assertEqual(result["tf_window"]["first_tf_stamp_s"]["null_after_first_non_null"], 1)
        self.assertEqual(result["verdict"]["tf_never_starved"], False)

    def test_watchdog_counters_are_reported(self) -> None:
        events = [_slam(0.0, first_tf=None, last_tf=None, publish=0),
                  _slam(5.0, first_tf=10.0, last_tf=15.0, publish=30, starved=2)]
        with tempfile.TemporaryDirectory() as tmp:
            result = report.summarize(_write_run(Path(tmp), events))
        self.assertEqual(result["counters"]["tf_starved_force_count"]["delta"], 2)
        self.assertEqual(result["verdict"]["watchdog_fired"], True)

    def test_stationary_preflight_needs_ten_continuous_ok_seconds(self) -> None:
        events = [_slam(0.0, first_tf=1.0, last_tf=1.0, publish=0)]
        events += [_slam(float(t), first_tf=1.0, last_tf=1.0 + t, publish=t * 4)
                   for t in range(1, 21)]
        events += [_fused(float(t), "ok", ("t265", "slam", "fused")) for t in range(0, 6)]
        events += [_fused(6.5, "lost", ())]
        events += [_fused(float(t), "ok", ("t265", "slam", "fused")) for t in range(7, 10)]
        with tempfile.TemporaryDirectory() as tmp:
            result = report.summarize(_write_run(Path(tmp), events))
        self.assertEqual(result["fused"]["longest_ok_s"], 5.0)
        self.assertEqual(result["verdict"]["stationary_preflight_ok"], False)

    def test_a_dead_bridge_cannot_pass_on_frozen_counters(self) -> None:
        # A crashed SlamBridge thread freezes its metrics at the last values, so
        # a frozen non-null first_tf_stamp_s must not look like a healthy run.
        events = [_slam(0.0, first_tf=100.0, last_tf=100.0, publish=0),
                  _slam(1.0, first_tf=100.0, last_tf=100.0, publish=2),
                  _slam(2.0, first_tf=100.0, last_tf=100.0, publish=2, state="SLAM_FAILED")]
        with tempfile.TemporaryDirectory() as tmp:
            result = report.summarize(_write_run(Path(tmp), events))
        self.assertEqual(result["slam"]["failed_samples"], 1)
        self.assertEqual(result["slam"]["last_state"], "SLAM_FAILED")
        self.assertEqual(result["verdict"]["bridge_alive"], False)
        self.assertEqual(result["verdict"]["tf_never_starved"], False)

    def test_missing_slam_status_is_reported_not_crashed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            result = report.summarize(_write_run(Path(tmp), [_fused(0.0, "lost", ())]))
        self.assertIn("no slam_status samples", result["note"])

    def test_main_prints_a_before_after_comparison(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            before = _write_run(root / "before", [
                _slam(0.0, first_tf=None, last_tf=None, publish=0, no_window=291)])
            after = _write_run(root / "after", [
                _slam(0.0, first_tf=100.0, last_tf=100.0, publish=0),
                _slam(20.0, first_tf=100.0, last_tf=120.0, publish=180),
            ])
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                code = report.main(["--compare", str(before), str(after)])
        self.assertEqual(code, 0)
        text = stream.getvalue()
        self.assertIn("before/after", text)
        self.assertIn("scan_publish_hz", text)
        self.assertIn("291", text)

    def test_json_mode_is_machine_readable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run = _write_run(Path(tmp), [_slam(0.0, first_tf=1.0, last_tf=2.0, publish=3)])
            stream = io.StringIO()
            with contextlib.redirect_stdout(stream):
                code = report.main([str(run), "--json"])
        self.assertEqual(code, 0)
        payload = json.loads(stream.getvalue())
        self.assertEqual(payload[0]["counters"]["scan_publish_count"]["end"], 3)


if __name__ == "__main__":
    unittest.main()
