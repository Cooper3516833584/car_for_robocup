"""Hardware-free checks for the offline stage-2 log analysis and C10B frame parsing."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from tools import c10b_telemetry_probe as probe  # noqa: E402
from tools import closed_loop_analyze as analyze  # noqa: E402


def _write_run(directory: Path, *, jump_at: int | None = None, innovation_m: float = 0.0,
               mutate=None) -> Path:
    """Write one synthetic closed_loop_motion-style run and return its directory."""
    run = directory / "run"
    run.mkdir()
    events = [{"t": 0.0, "type": "runtime_started"}]
    events.append({"t": 10.0, "type": "motion_action", "action": "drive_distance",
                   "state": "running", "phase": "tracking",
                   "command": {"linear_x_m_s": 0.1, "angular_z_rad_s": 0.0},
                   "diagnostics": {"remaining_m": 0.5}})
    x = 0.0
    for index in range(1, 101):
        moment = 10.0 + index * 0.05
        x += 0.1 * 0.05
        events.append({"t": moment, "type": "motion_action", "action": "drive_distance",
                       "state": "running", "phase": "tracking",
                       "command": {"linear_x_m_s": 0.1, "angular_z_rad_s": 0.0},
                       "diagnostics": {"remaining_m": 0.5 - x}})
        events.append({"t": moment, "type": "fused_pose", "x_m": x + (0.03 if index == jump_at else 0.0),
                       "y_m": 0.0, "yaw_rad": 0.0, "state": "ok", "age_s": 0.002,
                       "source_flags": ["t265", "slam", "fused"], "fusion_source": "T265_SLAM",
                       "d500_innovation_m": innovation_m if index == jump_at else 0.0,
                       "d500_innovation_yaw_rad": 0.0, "d500_accepted": True,
                       "t265_confidence": 0.667})
        events.append({"t": moment, "type": "drive_command", "requested_v_m_s": 0.1,
                       "limited_v_m_s": 0.1, "requested_omega_rad_s": 0.0, "limited_omega_rad_s": 0.0,
                       "left_target_m_s": 0.1, "right_target_m_s": 0.1,
                       "protocol_mode": "differential_vx_vz", "actuation_enabled": True})
    events.append({"t": 15.0, "type": "motion_action", "action": "drive_distance",
                   "state": "succeeded", "phase": "done",
                   "command": {"linear_x_m_s": 0.0, "angular_z_rad_s": 0.0}, "diagnostics": {}})
    events.append({"t": 15.0, "type": "fused_pose", "x_m": x, "y_m": 0.0, "yaw_rad": 0.0,
                   "state": "ok", "age_s": 0.002, "source_flags": ["t265", "slam", "fused"],
                   "fusion_source": "T265_SLAM", "d500_innovation_m": 0.0,
                   "d500_innovation_yaw_rad": 0.0, "d500_accepted": True, "t265_confidence": 0.667})
    if mutate is not None:
        mutate(events)
    with (run / "events.jsonl").open("w", encoding="utf-8") as handle:
        for event in events:
            handle.write(json.dumps(event) + "\n")
    (run / "manifest.json").write_text(json.dumps({"action": {"name": "drive-distance"}}), encoding="utf-8")
    (run / "summary.json").write_text(json.dumps({"valid": True, "action": "drive-distance"}),
                                     encoding="utf-8")
    return run


def _find(events: list[dict], kind: str, *, at: float | None = None) -> dict:
    for event in events:
        if event.get("type") != kind:
            continue
        if at is None or abs(event["t"] - at) < 1e-9:
            return event
    raise AssertionError("no %s event at %s" % (kind, at))


class ClosedLoopAnalyzeTests(unittest.TestCase):
    def test_clean_run_reports_commanded_and_measured_motion(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            result = analyze.summarize(_write_run(Path(tmp)))
        self.assertEqual(result["action"], "drive-distance")
        self.assertAlmostEqual(result["commanded"]["integral_linear_m"], 0.5, places=3)
        self.assertAlmostEqual(result["fused"]["longitudinal_m"], 0.495, places=3)
        self.assertAlmostEqual(result["fused"]["lateral_m"], 0.0, places=3)
        self.assertEqual(result["fused_jumps"]["steps_over_5cm"], 0)
        self.assertEqual(result["slam_innovation"]["fused_samples_with_nonzero_last_innovation"], 0)
        self.assertEqual(result["slam_anchor_events"]["total"], 0)
        self.assertTrue(result["validity"]["action_valid"])
        self.assertEqual(result["validity"]["invalid_reasons"], [])

    def test_injected_pose_jump_and_innovation_are_reported(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            result = analyze.summarize(_write_run(Path(tmp), jump_at=40, innovation_m=0.21))
        self.assertGreater(result["fused_jumps"]["max_step_m"], 0.02)
        self.assertAlmostEqual(result["fused_jumps"]["max_step_at_s"], 12.0, places=1)
        self.assertAlmostEqual(result["slam_innovation"]["max_m"], 0.21, places=3)
        self.assertEqual(result["slam_innovation"]["fused_samples_with_nonzero_last_innovation"], 1)
        # A 3.5 cm step is a warning at the 1 cm acceptance scale, not yet the
        # tool's own 10 cm abort bound.
        self.assertTrue(result["validity"]["action_valid"])
        self.assertTrue(any("innovation" in item for item in result["validity"]["warnings"]))

    def test_counts_unique_slam_anchor_events_and_relabels_t265_adapter(self) -> None:
        def mutate(events: list[dict]) -> None:
            events.extend([
                {"t": 12.0, "type": "slam_anchor", "accepted": True,
                 "innovation_m": 0.01, "innovation_yaw_rad": 0.02},
                {"t": 12.1, "type": "slam_anchor", "accepted": False,
                 "consensus_pending": True, "rejection_reason": "slam_innovation_gate",
                 "innovation_m": 0.21, "innovation_yaw_rad": -0.03},
                {"t": 12.0, "type": "t265_pose", "x_m": 0.0, "y_m": 0.0,
                 "yaw_rad": 3.13, "tracker_confidence": 3},
                {"t": 12.1, "type": "t265_pose", "x_m": 0.0, "y_m": 0.0,
                 "yaw_rad": -3.13, "tracker_confidence": 2},
            ])

        with tempfile.TemporaryDirectory() as tmp:
            result = analyze.summarize(_write_run(Path(tmp), mutate=mutate))
        self.assertEqual(result["slam_anchor_events"]["total"], 2)
        self.assertEqual(result["slam_anchor_events"]["accepted"], 1)
        self.assertEqual(result["slam_anchor_events"]["rejected"], 1)
        self.assertEqual(result["slam_anchor_events"]["consensus_pending"], 1)
        self.assertEqual(result["slam_anchor_events"]["rejection_reasons"]["slam_innovation_gate"], 1)
        self.assertNotIn("t265_raw", result)
        self.assertAlmostEqual(result["t265_adapter"]["yaw_change_deg"], 1.328, places=2)

    def test_jump_beyond_the_tool_abort_bound_is_invalid(self) -> None:
        def mutate(events: list[dict]) -> None:
            _find(events, "fused_pose", at=12.0)["x_m"] += 0.20

        with tempfile.TemporaryDirectory() as tmp:
            result = analyze.summarize(_write_run(Path(tmp), mutate=mutate))
        self.assertFalse(result["validity"]["action_valid"])
        self.assertTrue(any("jumped" in reason for reason in result["validity"]["invalid_reasons"]))

    def test_degraded_sources_mark_the_action_invalid(self) -> None:
        def mutate(events: list[dict]) -> None:
            _find(events, "fused_pose", at=12.0)["source_flags"] = ["t265", "fused"]

        with tempfile.TemporaryDirectory() as tmp:
            result = analyze.summarize(_write_run(Path(tmp), mutate=mutate))
        self.assertFalse(result["validity"]["action_valid"])
        self.assertTrue(any("missing sources" in reason
                            for reason in result["validity"]["invalid_reasons"]))

    def test_lost_pose_state_marks_the_action_invalid(self) -> None:
        def mutate(events: list[dict]) -> None:
            _find(events, "fused_pose", at=12.0)["state"] = "lost"

        with tempfile.TemporaryDirectory() as tmp:
            result = analyze.summarize(_write_run(Path(tmp), mutate=mutate))
        self.assertFalse(result["validity"]["action_valid"])
        self.assertTrue(any("healthy set" in reason for reason in result["validity"]["invalid_reasons"]))

    def test_incomplete_run_without_motion_events_does_not_raise(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp) / "empty"
            run.mkdir()
            (run / "events.jsonl").write_text('{"t": 0.0, "type": "runtime_started"}\n', encoding="utf-8")
            result = analyze.summarize(run)
        self.assertIn("note", result)

    def test_truncated_json_lines_are_skipped(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run = _write_run(Path(tmp))
            with (run / "events.jsonl").open("a", encoding="utf-8") as handle:
                handle.write('{"t": 99.0, "type": "fused_pose"\n')
            result = analyze.summarize(run)
        self.assertEqual(result["samples"]["fused"], 101)


class C10BFrameAlignmentTests(unittest.TestCase):
    @staticmethod
    def _frame(motor_flag: int, voltage: int) -> bytes:
        frame = bytearray(24)
        frame[0] = 0x7B
        frame[1] = motor_flag
        frame[20] = (voltage >> 8) & 0xFF
        frame[21] = voltage & 0xFF
        checksum = 0
        for value in frame[:22]:
            checksum ^= value
        frame[22] = checksum
        frame[23] = 0x7D
        return bytes(frame)

    def test_valid_frame_is_decoded(self) -> None:
        buffer = bytearray(self._frame(0, 0xE050))
        frames = probe._aligned_frames(buffer)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0][1], 0)
        self.assertEqual(buffer, bytearray())

    def test_leading_garbage_is_resynchronised(self) -> None:
        buffer = bytearray(b"\x11\x22\x33" + self._frame(0, 0xE050))
        self.assertEqual(len(probe._aligned_frames(buffer)), 1)

    def test_truncated_frame_is_retained_for_the_next_read(self) -> None:
        buffer = bytearray(self._frame(0, 0xE050)[:10])
        self.assertEqual(probe._aligned_frames(buffer), [])
        self.assertEqual(len(buffer), 10)

    def test_bad_checksum_is_dropped(self) -> None:
        corrupted = bytearray(self._frame(0, 0xE050))
        corrupted[5] ^= 0xFF
        self.assertEqual(probe._aligned_frames(corrupted), [])

    def test_motor_disabled_flag_is_visible(self) -> None:
        frames = probe._aligned_frames(bytearray(self._frame(1, 0xE050)))
        self.assertEqual(len(frames), 1)
        self.assertNotEqual(frames[0][1], 0)

    def test_no_header_clears_the_buffer(self) -> None:
        buffer = bytearray(b"\x01\x02\x03\x04")
        self.assertEqual(probe._aligned_frames(buffer), [])
        self.assertEqual(buffer, bytearray())


if __name__ == "__main__":
    unittest.main()
