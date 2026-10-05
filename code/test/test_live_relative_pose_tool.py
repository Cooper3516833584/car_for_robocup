"""Hardware-free checks for the bounded live-relative-pose diagnostic tool.

The 2026-10-04 field failure: ``tools/test_live_relative_pose.py --seconds 22``
ran for 3 min 51 s without exiting because the main loop could block on the
stationary preflight (and the shutdown could block on the T265 pipeline stop)
while the localization chain was broken.  These tests pin the hard budget.
"""

from __future__ import annotations

import contextlib
import io
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from tools import test_live_relative_pose as tool  # noqa: E402


class _FakeLogger:
    def __init__(self, _path=None) -> None:
        self.write_error = False
        self.dropped_events = 0
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _FakeRuntime:
    def __init__(self) -> None:
        self.t265_source = object()
        self.d500_source = object()
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _FakeConfig:
    class _D500:
        port = "/dev/null"
        baudrate = 115200

    d500 = _D500()


def _session_type(preflight, *, close_delay_s: float = 0.0):
    class _FakeSession:
        def __init__(self, runtime, logger, output) -> None:
            self.runtime = runtime
            self.logger = logger
            self.output = output
            self.error = None
            self.closed = False

        def start(self) -> None:
            pass

        def close(self) -> None:
            if close_delay_s:
                time.sleep(close_delay_s)
            self.closed = True

        def preflight(self):
            return preflight()

        def _recent(self, _seconds):
            return []

    return _FakeSession


def _run_main(preflight, *, seconds: float, shutdown_timeout: float = 1.0,
              close_delay_s: float = 0.0):
    output = Path(tempfile.mkdtemp()) / "run"
    session_type = _session_type(preflight, close_delay_s=close_delay_s)
    stream = io.StringIO()
    with patch.object(tool, "load_v2_config", return_value=object()), \
            patch.object(tool, "test_config_from_board", return_value=_FakeConfig()), \
            patch.object(tool, "JsonlEventLogger", _FakeLogger), \
            patch.object(tool, "build_runtime", return_value=_FakeRuntime()), \
            patch.object(tool, "CausalT265Source", lambda source: source), \
            patch.object(tool, "ScanOnlyD500Source", lambda *a, **k: object()), \
            patch.object(tool, "AccuracySession", session_type), \
            contextlib.redirect_stdout(stream):
        started = time.monotonic()
        code = tool.main(["--config", "board.toml", "--output", str(output),
                          "--seconds", str(seconds),
                          "--shutdown-timeout", str(shutdown_timeout)])
        elapsed = time.monotonic() - started
    return code, elapsed, stream.getvalue()


class BoundedPreflightTests(unittest.TestCase):
    def test_wedged_preflight_never_blocks_the_caller(self) -> None:
        session = _session_type(lambda: time.sleep(30.0))(None, None, None)
        probe = tool.BoundedPreflight(session, period_s=0.01, stale_after_s=0.05)
        probe.start()
        try:
            started = time.monotonic()
            ready, reasons = probe.latest()
            elapsed = time.monotonic() - started
            self.assertFalse(ready)
            self.assertIn("preflight did not return", reasons[0])
            self.assertLess(elapsed, 0.5)
        finally:
            probe.stop()

    def test_ready_result_is_reported_once_the_probe_returns(self) -> None:
        session = _session_type(lambda: (True, []))(None, None, None)
        probe = tool.BoundedPreflight(session, period_s=0.01, stale_after_s=1.0)
        probe.start()
        try:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                ready, reasons = probe.latest()
                if ready:
                    break
                time.sleep(0.01)
            self.assertTrue(ready)
            self.assertEqual(reasons, [])
        finally:
            probe.stop()


class RunBoundedTests(unittest.TestCase):
    def test_returns_the_value_of_a_fast_call(self) -> None:
        finished, value = tool.run_bounded(lambda: 7, 1.0)
        self.assertTrue(finished)
        self.assertEqual(value, 7)

    def test_abandons_a_call_that_outlives_the_budget(self) -> None:
        started = time.monotonic()
        finished, value = tool.run_bounded(lambda: time.sleep(30.0), 0.05)
        self.assertFalse(finished)
        self.assertIsNone(value)
        self.assertLess(time.monotonic() - started, 1.0)

    def test_propagates_an_exception(self) -> None:
        def boom():
            raise ValueError("nope")

        with self.assertRaises(ValueError):
            tool.run_bounded(boom, 1.0)


class MainBudgetTests(unittest.TestCase):
    def test_seconds_is_a_hard_budget_when_preflight_never_passes(self) -> None:
        code, elapsed, output = _run_main(
            lambda: (False, ["need ten seconds of stationary samples"]),
            seconds=0.7, shutdown_timeout=1.0,
        )
        self.assertEqual(code, 0)
        self.assertLess(elapsed, 5.0)
        self.assertIn("WAIT_ORIGIN", output)
        self.assertIn("Logs:", output)

    def test_a_wedged_preflight_cannot_extend_the_budget(self) -> None:
        # This is the reported 3 min 51 s hang: without the bounded probe the
        # main loop never reaches its deadline check.
        code, elapsed, output = _run_main(lambda: time.sleep(30.0), seconds=0.7,
                                          shutdown_timeout=1.0)
        self.assertEqual(code, 0)
        self.assertLess(elapsed, 6.0)
        self.assertIn("preflight did not return", output)

    def test_a_blocked_shutdown_cannot_extend_the_budget(self) -> None:
        code, elapsed, output = _run_main(
            lambda: (False, ["never ready"]), seconds=0.5,
            shutdown_timeout=1.0, close_delay_s=30.0,
        )
        self.assertEqual(code, 0)
        self.assertLess(elapsed, 6.0)
        self.assertIn("shutdown did not finish", output)
        self.assertIn("Logs:", output)


if __name__ == "__main__":
    unittest.main()
