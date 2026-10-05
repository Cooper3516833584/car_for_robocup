"""Relay build/runtime wiring tests: dry-run stays in memory, shutdown fails safe."""

from __future__ import annotations

from dataclasses import replace
import logging
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

logging.getLogger("robocup-runtime").setLevel(logging.CRITICAL)
logging.getLogger("components.relay_lcus").setLevel(logging.CRITICAL)

from components.navigation_common import NavigationGrid
from components.relay_lcus import FakeLCUSRelay, LCUSRelay
from config.v2_factory import build_relay
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode, validate_runtime_readiness
from robocup_runtime import build_runtime


class RecordingRelay:
    """Relay double that records shutdown calls and can be told to fail."""

    def __init__(self, *, all_off_result: bool = True, close_error: Exception | None = None) -> None:
        self.port = "recording"
        self.channel_count = 4
        self.connected = False
        self.events: list[str] = []
        self.open_calls = 0
        self.all_off_calls = 0
        self.close_calls = 0
        self._all_off_result = all_off_result
        self._close_error = close_error

    def open(self) -> None:
        self.open_calls += 1
        self.connected = True

    def all_off(self, verify: bool = False, retries: int = 1) -> bool:
        self.all_off_calls += 1
        self.events.append("relay_all_off")
        return self._all_off_result

    def close(self) -> None:
        self.close_calls += 1
        self.events.append("relay_close")
        if self._close_error is not None:
            raise self._close_error
        self.connected = False

    def query_status(self, timeout=None):
        return {1: False}

    def get_channel_state(self, channel: int, timeout=None):
        return False


class FailingOpenRelay(RecordingRelay):
    """Relay whose port cannot be opened; nothing was actuated yet."""

    def open(self) -> None:
        self.open_calls += 1
        raise OSError("relay port busy")


class RelayFactoryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_v2_config()
        self.enabled = replace(self.config, relay=replace(self.config.relay, enabled=True, port="COM9"))

    def test_disabled_relay_builds_no_device(self) -> None:
        self.assertIsNone(build_relay(self.config))
        self.assertIsNone(build_relay(self.config, fake=True))

    def test_fake_relay_never_opens_a_port(self) -> None:
        relay = build_relay(self.enabled, fake=True)
        self.assertIsInstance(relay, FakeLCUSRelay)
        self.assertFalse(relay.connected)

    def test_real_relay_is_constructed_from_config_without_opening(self) -> None:
        relay = build_relay(self.enabled)
        self.assertIsInstance(relay, LCUSRelay)
        self.assertEqual(relay.port, "COM9")
        self.assertEqual(relay.baudrate, self.enabled.relay.baudrate)
        self.assertEqual(relay.channel_count, self.enabled.relay.channel_count)
        self.assertFalse(relay.connected)

    def test_enabling_the_relay_does_not_change_hardware_mission_readiness(self) -> None:
        self.assertEqual(
            validate_runtime_readiness(self.config, RuntimeMode.HARDWARE_MISSION),
            validate_runtime_readiness(self.enabled, RuntimeMode.HARDWARE_MISSION),
        )


class RelayRuntimeTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = load_v2_config()
        self.enabled = replace(self.config, relay=replace(self.config.relay, enabled=True, port="COM9"))
        self.now = [5.0]

    def clock(self) -> float:
        return self.now[0]

    def make_runtime(self, config=None):
        return build_runtime(
            self.enabled if config is None else config,
            RuntimeMode.DRY_RUN,
            clock=self.clock,
            fake_sample_count=2,
            world=NavigationGrid(100, 100, 0.1, origin_x_m=-5.0, origin_y_m=-5.0),
        )

    def test_dry_run_uses_the_in_memory_relay_and_releases_it_on_close(self) -> None:
        runtime = self.make_runtime()
        self.assertIsInstance(runtime.relay, FakeLCUSRelay)
        runtime.run_steps(2, period_s=0.001)
        self.assertEqual(runtime.relay.open_count, 1)
        self.assertEqual(runtime.relay.off_requests, 1)
        self.assertEqual(runtime.relay.close_count, 1)
        self.assertFalse(runtime.relay.connected)

    def test_disabled_relay_leaves_the_runtime_without_a_payload_device(self) -> None:
        runtime = self.make_runtime(self.config)
        self.assertIsNone(runtime.relay)
        runtime.run_steps(1, period_s=0.001)

    def test_opening_a_relay_only_happens_at_start(self) -> None:
        runtime = self.make_runtime()
        self.assertEqual(runtime.relay.open_count, 0)
        runtime.start()
        self.assertEqual(runtime.relay.open_count, 1)
        runtime.close()

    def test_start_failure_releases_the_relay_and_stops_everything_else(self) -> None:
        runtime = self.make_runtime()
        runtime.relay = FailingOpenRelay()
        with self.assertRaisesRegex(OSError, "busy"):
            runtime.start()
        self.assertFalse(runtime.is_running)
        self.assertTrue(runtime.t265_source.stopped)
        self.assertTrue(runtime.d500_source.stopped)
        self.assertEqual(runtime.drive.backend.close_count, 0)
        self.assertEqual(runtime.relay.close_calls, 1)

    def test_unconfirmed_disconnect_still_closes_cleanly(self) -> None:
        runtime = self.make_runtime()
        runtime.relay = RecordingRelay(all_off_result=False, close_error=OSError("port gone"))
        runtime.start()
        runtime.close()
        self.assertEqual(runtime.relay.all_off_calls, 1)
        self.assertEqual(runtime.relay.close_calls, 1)
        self.assertEqual(runtime.drive.backend.close_count, 1)

    def test_payload_release_happens_after_the_base_stops_and_before_the_drive_closes(self) -> None:
        runtime = self.make_runtime()
        order: list[str] = []
        backend = runtime.drive.backend
        original_stop, original_close = backend.stop, backend.close

        def recording_stop() -> None:
            order.append("drive_stop")
            original_stop()

        def recording_close() -> None:
            order.append("drive_close")
            original_close()

        backend.stop = recording_stop
        backend.close = recording_close
        relay = RecordingRelay()
        relay.events = order
        runtime.relay = relay

        runtime.start()
        runtime.close()
        self.assertEqual(
            order,
            ["drive_stop", "relay_all_off", "relay_close", "drive_stop", "drive_close"],
        )

    def test_relay_events_are_recorded_in_the_diagnostics_log(self) -> None:
        class CapturingLogger:
            def __init__(self) -> None:
                self.events: list[dict] = []

            def emit(self, event, priority: bool = False) -> None:
                self.events.append(event)

            def close(self) -> None:
                return None

        runtime = self.make_runtime()
        capture = CapturingLogger()
        runtime.event_logger = capture
        runtime.run_steps(1, period_s=0.001)
        types = [event["type"] for event in capture.events]
        self.assertIn("relay_ready", types)
        self.assertIn("relay_shutdown", types)


if __name__ == "__main__":
    unittest.main()
