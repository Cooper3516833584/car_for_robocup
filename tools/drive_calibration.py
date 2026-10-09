#!/usr/bin/env python3
"""Retired motor entry; use closed_loop_motion.py. Read-only helpers remain available."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "code"))

from components.pose_fusion import (  # noqa: E402
    PoseFusionState, SLAM_CONSENSUS_GRACE_S, SLAM_CANDIDATE_MAX_AGE_S,
)
from config.relative_slam_profile import accepted_relative_slam_profile  # noqa: E402
from config.v2_factory import build_differential_drive  # noqa: E402
from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config  # noqa: E402
from config.v2_runtime import RuntimeMode, runtime_constraints  # noqa: E402
from core.types import Twist2D  # noqa: E402
from robocup_runtime import RobocupMissionState, build_runtime  # noqa: E402
from tools.motion_test_common import unwrap_delta  # noqa: E402


SEND_PERIOD_S = 0.02
MAX_PULSE_S = 45.0
SETTLE_TIMEOUT_S = 3.0
NO_PROGRESS_TIMEOUT_S = 3.0
MAX_POSE_STEP_M = 0.10
MAX_YAW_STEP_RAD = math.radians(20.0)
MAX_ROTATION_CENTER_DRIFT_M = 0.10
ZERO = Twist2D(0.0, 0.0)


@dataclass(frozen=True)
class PoseSample:
    received_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    t265_age_s: float
    slam_age_s: float
    t265_confidence: float
    source_flags: tuple[str, ...]


@dataclass(frozen=True)
class PulseResult:
    target: float
    predicted: float
    measured: float
    lateral_m: float
    center_shift_m: float
    elapsed_s: float


def accepted_fused_sample(estimate, config, now_s: float, *,
                          allow_pending: bool = False) -> PoseSample | None:
    """Require the same fresh T265+SLAM chain used by relative task actions."""

    pending_age = estimate.slam_observation_age_s
    slam_fresh = (estimate.d500_age_s is not None and estimate.d500_age_s <= 0.5)
    if (allow_pending and estimate.slam_consensus_pending
            and estimate.d500_age_s is not None
            and 0.0 <= estimate.d500_age_s <= SLAM_CONSENSUS_GRACE_S
            and pending_age is not None and math.isfinite(pending_age)
            and 0.0 <= pending_age <= SLAM_CANDIDATE_MAX_AGE_S):
        slam_fresh = True
    if (estimate.pose is None or estimate.state is not PoseFusionState.OK
            or not {"t265", "slam", "fused"}.issubset(estimate.source_flags)
            or not estimate.anchor_initialized
            or estimate.t265_age_s is None or estimate.d500_age_s is None
            or estimate.t265_confidence is None
            or not all(math.isfinite(value) for value in (
                now_s, estimate.pose.x_m, estimate.pose.y_m, estimate.pose.yaw_rad,
                estimate.t265_age_s, estimate.d500_age_s, estimate.t265_confidence,
            ))
            or estimate.t265_age_s < 0.0 or estimate.d500_age_s < 0.0
            or estimate.t265_age_s > config.fusion.t265_max_age_s
            or not slam_fresh
            or estimate.t265_confidence < config.fusion.t265_min_tracker_confidence / 3.0):
        return None
    pose = estimate.pose
    return PoseSample(
        now_s, pose.x_m, pose.y_m, pose.yaw_rad,
        estimate.t265_age_s, estimate.d500_age_s,
        estimate.t265_confidence, estimate.source_flags,
    )


class FusedPoseReader(threading.Thread):
    """Run the existing sensor-only Runtime and keep its latest accepted pose."""

    def __init__(self, config, *, clock=time.monotonic, sleep=time.sleep) -> None:
        super().__init__(daemon=True, name="drive-calibration-fused-pose")
        self.config = accepted_relative_slam_profile(config)
        self.clock = clock
        self.sleep = sleep
        self.runtime = build_runtime(self.config, RuntimeMode.HARDWARE_PROBE, sensor_only=True, clock=clock)
        self.error: str | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._sample: PoseSample | None = None

    def run(self) -> None:
        try:
            self.runtime.start()
            while not self._stop_event.is_set():
                result = self.runtime.step()
                if result.error or result.mission_state in {RobocupMissionState.ERROR, RobocupMissionState.SAFE_STOP}:
                    raise RuntimeError(result.error or result.mission_state.value)
                sample = accepted_fused_sample(result.estimate, self.config, self.clock())
                with self._lock:
                    self._sample = sample
                self.sleep(SEND_PERIOD_S)
        except Exception as exc:
            self.error = "%s: %s" % (type(exc).__name__, exc)
            with self._lock:
                self._sample = None
        finally:
            self.runtime.close()

    def latest(self) -> PoseSample | None:
        with self._lock:
            sample = self._sample
        if sample is None or self.error:
            return None
        age = self.clock() - sample.received_s
        return sample if 0.0 <= age <= self.config.fusion.t265_max_age_s else None

    def wait_ready(self, timeout_s: float = 30.0) -> bool:
        deadline = self.clock() + timeout_s
        while self.clock() < deadline and not self.error:
            if self.latest() is not None:
                return True
            self.sleep(0.05)
        return False

    def stop(self) -> None:
        self._stop_event.set()


def _progress(start: PoseSample, current: PoseSample, kind: str, yaw_total: float) -> tuple[float, float, float]:
    dx = current.x_m - start.x_m
    dy = current.y_m - start.y_m
    along = dx * math.cos(start.yaw_rad) + dy * math.sin(start.yaw_rad)
    lateral = -dx * math.sin(start.yaw_rad) + dy * math.cos(start.yaw_rad)
    center = math.hypot(dx, dy)
    return (along if kind == "distance" else yaw_total), lateral, center


class PulseRunner:
    def __init__(self, drive, reader, *, clock=time.monotonic, sleep=time.sleep) -> None:
        self.drive = drive
        self.reader = reader
        self.clock = clock
        self.sleep = sleep
        self.aborted = False
        self.trace: list[dict] = []

    def _sample(self) -> PoseSample:
        sample = self.reader.latest()
        if sample is None:
            raise RuntimeError("fresh fused T265+SLAM pose unavailable")
        return sample

    def run(self, kind: str, target: float, speed: float) -> PulseResult:
        self.drive.stop()
        raise RuntimeError("Retired motion executor; use tools/closed_loop_motion.py with localization")

    def _wait_for_settle(self, previous: PoseSample, yaw_total: float) -> tuple[PoseSample, float]:
        self.drive.stop()
        raise RuntimeError("Retired motion executor; use tools/closed_loop_motion.py with localization")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="this car's local schema-v2 profile")
    parser.add_argument("--trace", required=True, help="CSV file outside the repository for this one action")
    parser.add_argument("--confirm-motor-test", action="store_true")
    parser.add_argument("--confirm-mount-checked", action="store_true")
    actions = parser.add_subparsers(dest="action", required=True)
    distance = actions.add_parser("distance")
    distance.add_argument("--cm", type=float, required=True, help="signed 10..50 cm")
    distance.add_argument("--speed-m-s", type=float, default=0.05)
    rotate = actions.add_parser("rotate")
    rotate.add_argument("--deg", type=float, required=True, help="signed 10..90 degrees, CCW positive")
    rotate.add_argument("--speed-rad-s", type=float, default=0.20)
    return parser


def _write_trace(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None) -> int:
    print("Retired motion entry; use tools/closed_loop_motion.py or run_center_target_route.py", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
