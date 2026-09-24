from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.diagnostics_log import JsonlEventLogger
from components.pose_fusion import PoseFusion
from components.pose_log_replay import read_pose_events, replay_fusion
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from robocup_runtime import build_runtime


def synthetic_trajectory(path: Path) -> None:
    """Write straight-turn-straight sensors with drift, rejects and low confidence."""
    with path.open("w", encoding="utf-8") as stream:
        index = 0
        for tick in range(51):
            t = tick * 0.2
            if t <= 5.0:
                x, y, yaw = t / 5.0, 0.0, 0.0
            elif t <= 7.0:
                x, y, yaw = 1.0, 0.0, (t - 5.0) * (3.141592653589793 / 4.0)
            else:
                x, y, yaw = 1.0, (t - 7.0) / 3.0, 3.141592653589793 / 2.0
            if abs(t - 8.0) < 1e-8:
                stream.write(json.dumps({"t": t, "type": "t265_rejected", "reason": "tracker_confidence_too_low"}) + "\n")
            else:
                stream.write(json.dumps({
                    "t": t, "type": "t265_pose", "x_m": x + 0.01 * t,
                    "y_m": y, "yaw_rad": yaw, "confidence": 0.95,
                }) + "\n")
            d500_x = x + (5.0 if abs(t - 6.0) < 1e-8 else 0.0)
            stream.write(json.dumps({
                "t": t, "type": "d500_pose", "x_m": d500_x,
                "y_m": y, "yaw_rad": yaw,
            }) + "\n")
            index += 1


class PoseLogReplayTests(unittest.TestCase):
    def test_golden_synthetic_log_replays_fusion_and_rejects_outlier(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "golden.jsonl"
            synthetic_trajectory(path)
            events = read_pose_events(path)
            estimates = replay_fusion(events, PoseFusion(load_v2_config().fusion))
        self.assertEqual(len(events), 101)
        outlier_result = next(estimate for t, estimate in estimates if abs(t - 6.0) < 1e-8 and estimate.d500_accepted is False)
        self.assertEqual(outlier_result.rejection_reason, "position_innovation_gate")
        final = estimates[-1][1]
        self.assertEqual(final.state.value, "ok")
        self.assertAlmostEqual(final.pose.x_m, 1.0, delta=0.10)
        self.assertAlmostEqual(final.pose.y_m, 1.0, delta=0.10)

    def test_runtime_replay_consumes_events_without_sensor_devices(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "replay.jsonl"
            path.write_text(
                '{"t":0.0,"type":"t265_pose","x_m":0,"y_m":0,"yaw_rad":0}\n'
                '{"t":0.0,"type":"d500_pose","x_m":0,"y_m":0,"yaw_rad":0}\n'
                '{"t":0.1,"type":"t265_pose","x_m":0.1,"y_m":0,"yaw_rad":0}\n',
                encoding="utf-8",
            )
            runtime = build_runtime(
                load_v2_config(), RuntimeMode.REPLAY, replay_file=str(path), clock=lambda: 10.0
            )
            results = runtime.run_replay()
        self.assertEqual(len(results), 2)
        self.assertIsNone(runtime.t265_source)
        self.assertIsNone(runtime.d500_source)
        self.assertGreaterEqual(results[-1].estimate.pose.x_m, 0.09)

    def test_async_jsonl_logger_writes_events(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "events.jsonl"
            logger = JsonlEventLogger(path, capacity=16)
            logger.emit({"t": 0.0, "type": "safety", "state": "safe_stop"}, priority=True)
            logger.emit({"t": 0.1, "type": "fused_pose", "state": "ok"})
            time.sleep(0.07)
            logger.close()
            lines = path.read_text(encoding="utf-8").splitlines()
            rows = [json.loads(line) for line in lines]
        self.assertTrue(all(isinstance(row, dict) for row in rows))
        self.assertEqual({row["type"] for row in rows}, {"safety", "fused_pose"})


if __name__ == "__main__":
    unittest.main()
