#!/usr/bin/env python3
"""Interactive, sensor-only T265/D500/SLAM displacement measurement.

The operator moves the vehicle. This program never starts a drive backend or
opens a payload relay. Ground truth is entered in the floor frame after each
stop; planned travel distances are never used as measurements.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import select
import statistics
import sys
import threading
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "code"))

from components.diagnostics_log import JsonlEventLogger  # noqa: E402
from components.radar_driver import D500SerialDriver, RadarScanAssembler  # noqa: E402
from config.v2_loader import load_v2_config  # noqa: E402
from config.v2_models import LocalizationConfig, SlamLocalizationConfig  # noqa: E402
from config.v2_runtime import RuntimeMode  # noqa: E402
from robocup_runtime import build_runtime  # noqa: E402

SAMPLE_PERIOD_S = 0.02
WINDOW_S = 2.0
MIN_WINDOW_SAMPLES = 20
MAX_SEGMENT_S = 180.0
POSITION_LIMIT_M = 0.03
YAW_LIMIT_DEG = 3.0
JUMP_LIMIT_M = 0.30
MIN_TRACKER_CONFIDENCE = 2


@dataclass(frozen=True)
class FloorPose:
    x_m: float
    y_m: float
    yaw_deg: float  # continuous: +360 is different from zero


@dataclass(frozen=True)
class PoseSample:
    t_s: float
    x_m: float | None
    y_m: float | None
    yaw_rad: float | None
    yaw_unwrapped_rad: float | None
    t265_x_m: float | None
    t265_y_m: float | None
    t265_yaw_rad: float | None
    t265_confidence: float | None
    fusion_state: str
    slam_state: str
    anchor_age_s: float | None
    scan_count: int
    scan_age_ms: float | None
    tf_age_ms: float | None


@dataclass(frozen=True)
class Snapshot:
    t_s: float
    x_m: float
    y_m: float
    yaw_unwrapped_rad: float
    sample_count: int


def wrapped_delta_rad(previous: float, current: float) -> float:
    return (current - previous + math.pi) % (2.0 * math.pi) - math.pi


def relative_delta(start_x: float, start_y: float, start_yaw_rad: float,
                   end_x: float, end_y: float, end_yaw_rad: float) -> tuple[float, float, float]:
    """Express the end displacement in the vehicle's starting +X/+Y frame."""
    dx, dy = end_x - start_x, end_y - start_y
    c, s = math.cos(start_yaw_rad), math.sin(start_yaw_rad)
    return c * dx + s * dy, -s * dx + c * dy, end_yaw_rad - start_yaw_rad


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.inf
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


def summarize_window(samples: list[PoseSample]) -> Snapshot:
    valid = [s for s in samples if s.x_m is not None and s.y_m is not None
             and s.yaw_unwrapped_rad is not None]
    if len(valid) < MIN_WINDOW_SAMPLES:
        raise ValueError("not enough valid fused poses in the last two seconds")
    return Snapshot(
        valid[-1].t_s,
        statistics.median(s.x_m for s in valid),
        statistics.median(s.y_m for s in valid),
        statistics.median(s.yaw_unwrapped_rad for s in valid),
        len(valid),
    )


def test_config_from_board(config):
    """Override only the in-memory probe profile; never edit the board TOML."""
    if not config.t265.enabled:
        raise ValueError("T265 must be enabled for the manual accuracy probe")
    if config.relay.enabled:
        raise ValueError("disable the payload relay before a sensor-only probe")
    return replace(
        config,
        fusion=replace(config.fusion, t265_min_tracker_confidence=MIN_TRACKER_CONFIDENCE),
        d500=replace(config.d500, enabled=True),
        d500_localization=replace(config.d500_localization, enable_wall_absolute=False),
        localization=LocalizationConfig(
            backend="slam_toolbox",
            slam=SlamLocalizationConfig(
                enabled=True, require_field_anchor=False,
                hardware_mission_validated=False,
            ),
        ),
    )


class ScanOnlyD500Source:
    """Read D500 revolutions for SLAM without running scan-to-scan ICP."""

    def __init__(self, runtime, port: str, baudrate: int, logger: JsonlEventLogger) -> None:
        self.runtime = runtime
        self.logger = logger
        self.assembler = RadarScanAssembler()
        self.serial = D500SerialDriver(
            port=port, baudrate=baudrate, on_packet=self._on_packet,
        )

    def _on_packet(self, packet) -> None:
        bridge = self.runtime.slam_bridge
        if bridge is None:
            return
        for scan in self.assembler.feed(packet):
            received_s = time.monotonic()
            measurement_s = self.runtime._map_d500_timestamp(scan.timestamp_ms, received_s)
            bridge.push_d500_scan(scan, measurement_s)
            self.logger.emit({
                "type": "accuracy_d500_scan", "monotonic_s": measurement_s,
                "point_count": len(scan.points),
                "rotation_speed_deg_s": scan.rotation_speed_deg_s,
            })

    def start(self) -> "ScanOnlyD500Source":
        self.assembler.reset()
        self.serial.start()
        return self

    def close(self) -> None:
        self.serial.close()


class AccuracySession:
    """Bounded sample buffer, manual markers, and independent floor truth."""

    def __init__(self, runtime, logger: JsonlEventLogger, output_dir: Path) -> None:
        self.runtime = runtime
        self.logger = logger
        self.output_dir = output_dir
        self.samples: deque[PoseSample] = deque(maxlen=12000)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.worker: threading.Thread | None = None
        self.error: str | None = None
        self.preflight_ok = False
        self.start_snapshot: Snapshot | None = None
        self.end_snapshot: Snapshot | None = None
        self.segment_label: str | None = None
        self.segment_kind = "move"
        self.start_truth = FloorPose(0.0, 0.0, 0.0)
        self.current_truth = self.start_truth
        self.last_end_snapshot: Snapshot | None = None
        self.loop_start_snapshot: Snapshot | None = None
        self.loop_start_truth: FloorPose | None = None
        self.loop_start_result_index: int | None = None
        self.results: list[dict] = []
        self.closures: list[dict] = []
        self.start_time_s: float | None = None
        self._last_valid_pose_sample: PoseSample | None = None

    def start(self) -> None:
        self.runtime.start()
        self.start_time_s = self.runtime._start_time_s
        self.worker = threading.Thread(target=self._sample_loop, name="accuracy-sampler", daemon=True)
        self.worker.start()

    def close(self) -> None:
        self.stop_event.set()
        worker_alive = False
        if self.worker is not None:
            self.worker.join(timeout=10.0)
            worker_alive = self.worker.is_alive()
        self.runtime.close()
        summary = {
            "segments": len(self.results),
            "passed": sum(r["status"] == "PASS" for r in self.results),
            "failed": sum(r["status"] == "FAIL" for r in self.results),
            "invalid": sum(r["status"] == "INVALID" for r in self.results),
            "closures": self.closures,
            "sampler_error": self.error,
            "log_error": self.logger.write_error,
            "dropped_events": self.logger.dropped_events,
            "t265_min_tracker_confidence": MIN_TRACKER_CONFIDENCE,
        }
        (self.output_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        if worker_alive:
            raise RuntimeError("sampler did not stop within ten seconds")

    def _sample_loop(self) -> None:
        try:
            while not self.stop_event.is_set():
                started = time.monotonic()
                step = self.runtime.step()
                if step.error:
                    raise RuntimeError(step.error)
                estimate = step.estimate
                pose = estimate.pose
                t265 = self.runtime.fusion.continuous_t265_pose
                bridge = self.runtime.slam_bridge
                metrics = bridge.metrics() if bridge is not None else {}
                anchor = bridge.latest_slam_anchor() if bridge is not None else None
                with self.lock:
                    yaw_unwrapped = None
                    if pose is not None:
                        yaw_unwrapped = pose.yaw_rad
                        last_valid = self._last_valid_pose_sample
                        if last_valid is not None:
                            yaw_unwrapped = last_valid.yaw_unwrapped_rad + wrapped_delta_rad(
                                last_valid.yaw_rad, pose.yaw_rad
                            )
                    sample = PoseSample(
                        t_s=step.now_s,
                        x_m=None if pose is None else pose.x_m,
                        y_m=None if pose is None else pose.y_m,
                        yaw_rad=None if pose is None else pose.yaw_rad,
                        yaw_unwrapped_rad=yaw_unwrapped,
                        t265_x_m=None if t265 is None else t265.x_m,
                        t265_y_m=None if t265 is None else t265.y_m,
                        t265_yaw_rad=None if t265 is None else t265.yaw_rad,
                        t265_confidence=estimate.t265_confidence,
                        fusion_state=estimate.state.value,
                        slam_state="SLAM_FAILED" if bridge is None else bridge.state(step.now_s),
                        anchor_age_s=None if anchor is None else anchor.age_s(step.now_s),
                        scan_count=metrics.get("slam.scan_publish_count", 0),
                        scan_age_ms=metrics.get("slam.scan_age_ms"),
                        tf_age_ms=metrics.get("slam.nearest_t265_tf_age_ms"),
                    )
                    self.samples.append(sample)
                    if pose is not None:
                        self._last_valid_pose_sample = sample
                self.logger.emit({
                    "t": sample.t_s - self.start_time_s, "type": "accuracy_sample",
                    "monotonic_s": sample.t_s,
                    "fused_x_m": sample.x_m, "fused_y_m": sample.y_m,
                    "fused_yaw_rad": sample.yaw_rad,
                    "fused_yaw_unwrapped_rad": sample.yaw_unwrapped_rad,
                    "t265_x_m": sample.t265_x_m, "t265_y_m": sample.t265_y_m,
                    "t265_yaw_rad": sample.t265_yaw_rad,
                    "t265_confidence": sample.t265_confidence,
                    "fusion_state": sample.fusion_state,
                    "slam_state": sample.slam_state,
                    "anchor_age_s": sample.anchor_age_s,
                    "scan_count": sample.scan_count,
                    "scan_age_ms": sample.scan_age_ms,
                    "nearest_t265_tf_age_ms": sample.tf_age_ms,
                })
                self.stop_event.wait(max(0.0, SAMPLE_PERIOD_S - (time.monotonic() - started)))
        except Exception as exc:
            self.error = "%s: %s" % (type(exc).__name__, exc)
            self.stop_event.set()

    def _recent(self, seconds: float) -> list[PoseSample]:
        with self.lock:
            if not self.samples:
                return []
            cutoff = self.samples[-1].t_s - seconds
            return [s for s in self.samples if s.t_s >= cutoff]

    @staticmethod
    def _stationary_snapshot(samples: list[PoseSample]) -> Snapshot:
        snap = summarize_window(samples)
        if samples[-1].t_s - samples[0].t_s < 1.8:
            raise ValueError("hold still for at least two seconds before marking")
        radial = [math.hypot(s.x_m - snap.x_m, s.y_m - snap.y_m)
                  for s in samples if s.x_m is not None and s.y_m is not None]
        yaw = [abs(math.degrees(s.yaw_unwrapped_rad - snap.yaw_unwrapped_rad))
               for s in samples if s.yaw_unwrapped_rad is not None]
        if percentile(radial, 0.95) > 0.01 or percentile(yaw, 0.95) > 1.0:
            raise ValueError("pose has not settled within 1 cm and 1 degree")
        return snap

    @staticmethod
    def _health_reasons(samples: list[PoseSample]) -> list[str]:
        if not samples:
            return ["no samples"]
        reasons = []
        if any(s.x_m is None or s.y_m is None or s.yaw_unwrapped_rad is None for s in samples):
            reasons.append("missing fused pose")
        # PoseFusion reports tracker confidence normalized from 0..3 to 0..1.
        minimum = MIN_TRACKER_CONFIDENCE / 3.0
        if any(s.t265_confidence is None or s.t265_confidence < minimum for s in samples):
            reasons.append("T265 tracker confidence below 2/3")
        if any(s.slam_state != "SLAM_OK" or s.fusion_state != "ok" for s in samples):
            reasons.append("SLAM/fusion not healthy throughout segment")
        if any(s.anchor_age_s is None or s.anchor_age_s > 0.5 for s in samples):
            reasons.append("SLAM anchor older than 0.5 s")
        valid = [s for s in samples if s.x_m is not None and s.y_m is not None]
        if any(math.hypot(b.x_m - a.x_m, b.y_m - a.y_m) > JUMP_LIMIT_M
               for a, b in zip(valid, valid[1:])):
            reasons.append("fused position jump above 0.30 m")
        return reasons

    def preflight(self) -> tuple[bool, list[str]]:
        samples = self._recent(10.0)
        reasons = self._health_reasons(samples)
        if len(samples) < 100 or samples[-1].t_s - samples[0].t_s < 9.0:
            reasons.append("need ten seconds of stationary samples")
        try:
            snap = summarize_window(samples)
            radial = [math.hypot(s.x_m - snap.x_m, s.y_m - snap.y_m)
                      for s in samples if s.x_m is not None and s.y_m is not None]
            yaw = [abs(math.degrees(s.yaw_unwrapped_rad - snap.yaw_unwrapped_rad))
                   for s in samples if s.yaw_unwrapped_rad is not None]
            if percentile(radial, 0.95) > 0.01:
                reasons.append("stationary position jitter p95 above 1 cm")
            if percentile(yaw, 0.95) > 1.0:
                reasons.append("stationary yaw jitter p95 above 1 degree")
        except ValueError as exc:
            reasons.append(str(exc))
        tf_ages = [s.tf_age_ms for s in samples if s.tf_age_ms is not None]
        if percentile(tf_ages, 0.95) >= 30.0:
            reasons.append("T265/scan TF alignment p95 is not below 30 ms")
        if len(samples) >= 2:
            rate = (samples[-1].scan_count - samples[0].scan_count) / max(
                0.001, samples[-1].t_s - samples[0].t_s
            )
            if not 3.5 <= rate <= 6.0:
                reasons.append("published scan rate outside 3.5-6 Hz: %.2f" % rate)
        self.preflight_ok = not reasons
        return self.preflight_ok, reasons

    def begin(self, label: str, kind: str = "move") -> Snapshot:
        if not self.preflight_ok:
            raise ValueError("run a passing stationary preflight first")
        if self.segment_label is not None:
            raise ValueError("finish the current segment first")
        if kind not in {"move", "pivot"}:
            raise ValueError("segment kind must be move or pivot")
        recent = self._recent(WINDOW_S)
        reasons = self._health_reasons(recent)
        if reasons:
            raise ValueError("unhealthy start window: " + "; ".join(reasons))
        snap = self._stationary_snapshot(recent)
        self.segment_label = label
        self.segment_kind = kind
        self.start_snapshot = snap
        self.start_truth = self.current_truth
        self.logger.emit({"t": snap.t_s - self.start_time_s,
                          "monotonic_s": snap.t_s, "type": "accuracy_begin",
                          "label": label, "kind": kind}, priority=True)
        return snap

    def end(self) -> tuple[Snapshot, tuple[float, float, float]]:
        if self.segment_label is None or self.start_snapshot is None:
            raise ValueError("no active segment")
        if self.end_snapshot is not None:
            raise ValueError("enter ground truth for the previous end first")
        recent = self._recent(WINDOW_S)
        self.end_snapshot = self._stationary_snapshot(recent)
        if self.end_snapshot.t_s <= self.start_snapshot.t_s:
            self.end_snapshot = None
            raise ValueError("end must be later than start")
        delta = relative_delta(
            self.start_snapshot.x_m, self.start_snapshot.y_m,
            self.start_snapshot.yaw_unwrapped_rad,
            self.end_snapshot.x_m, self.end_snapshot.y_m,
            self.end_snapshot.yaw_unwrapped_rad,
        )
        self.logger.emit({"t": self.end_snapshot.t_s - self.start_time_s,
                          "monotonic_s": self.end_snapshot.t_s, "type": "accuracy_end",
                          "label": self.segment_label}, priority=True)
        return self.end_snapshot, delta

    def truth(self, measured: FloorPose) -> dict:
        if self.segment_label is None or self.start_snapshot is None or self.end_snapshot is None:
            raise ValueError("use begin and end before entering ground truth")
        if not all(math.isfinite(v) for v in (measured.x_m, measured.y_m, measured.yaw_deg)):
            raise ValueError("ground truth must be finite")
        start, end = self.start_snapshot, self.end_snapshot
        with self.lock:
            interval = [s for s in self.samples if start.t_s <= s.t_s <= end.t_s]
        reasons = self._health_reasons(interval)
        if end.t_s - start.t_s > MAX_SEGMENT_S:
            reasons.append("segment exceeded 180 s sample retention limit")
        if not interval or interval[0].t_s - start.t_s > 0.2:
            reasons.append("segment samples were lost from the bounded buffer")
        estimated = relative_delta(start.x_m, start.y_m, start.yaw_unwrapped_rad,
                                   end.x_m, end.y_m, end.yaw_unwrapped_rad)
        actual = relative_delta(
            self.start_truth.x_m, self.start_truth.y_m,
            math.radians(self.start_truth.yaw_deg),
            measured.x_m, measured.y_m, math.radians(measured.yaw_deg),
        )
        position_error = math.hypot(estimated[0] - actual[0], estimated[1] - actual[1])
        yaw_error_deg = math.degrees(estimated[2] - actual[2])
        tf_ages = [s.tf_age_ms for s in interval if s.tf_age_ms is not None]
        scan_rate = ((interval[-1].scan_count - interval[0].scan_count)
                     / max(0.001, interval[-1].t_s - interval[0].t_s)) if len(interval) >= 2 else 0.0
        if percentile(tf_ages, 0.95) >= 30.0:
            reasons.append("T265/scan TF alignment p95 is not below 30 ms")
        if scan_rate < 3.5:
            reasons.append("scan publication below 3.5 Hz")
        if self.logger.write_error or self.logger.dropped_events:
            reasons.append("event log write failed or dropped events")
        status = "INVALID" if reasons else (
            "PASS" if position_error <= POSITION_LIMIT_M and abs(yaw_error_deg) <= YAW_LIMIT_DEG
            else "FAIL"
        )
        row = {
            "label": self.segment_label, "kind": self.segment_kind, "status": status,
            "duration_s": round(end.t_s - start.t_s, 3),
            "truth_x_cm": round(measured.x_m * 100, 2),
            "truth_y_cm": round(measured.y_m * 100, 2),
            "truth_yaw_deg": round(measured.yaw_deg, 2),
            "actual_dx_cm": round(actual[0] * 100, 2),
            "actual_dy_cm": round(actual[1] * 100, 2),
            "actual_dyaw_deg": round(math.degrees(actual[2]), 2),
            "estimate_dx_cm": round(estimated[0] * 100, 2),
            "estimate_dy_cm": round(estimated[1] * 100, 2),
            "estimate_dyaw_deg": round(math.degrees(estimated[2]), 2),
            "position_error_cm": round(position_error * 100, 2),
            "yaw_error_deg": round(yaw_error_deg, 2),
            "scan_hz": round(scan_rate, 2),
            "tf_age_p95_ms": None if not tf_ages else round(percentile(tf_ages, 0.95), 2),
            "invalid_reasons": "; ".join(reasons),
        }
        with (self.output_dir / "segments.csv").open("a", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(row))
            if stream.tell() == 0:
                writer.writeheader()
            writer.writerow(row)
        self.logger.emit({"t": end.t_s - self.start_time_s,
                          "monotonic_s": end.t_s, "type": "accuracy_result",
                          **row}, priority=True)
        self.results.append(row)
        self.current_truth = measured
        self.last_end_snapshot = end
        self.segment_label = None
        self.segment_kind = "move"
        self.start_snapshot = self.end_snapshot = None
        return row

    def mark_loop_start(self, label: str) -> Snapshot:
        if self.segment_label is not None:
            raise ValueError("finish the active segment before marking a loop")
        recent = self._recent(WINDOW_S)
        reasons = self._health_reasons(recent)
        if reasons:
            raise ValueError("unhealthy loop start: " + "; ".join(reasons))
        snap = self._stationary_snapshot(recent)
        self.loop_start_snapshot = snap
        self.loop_start_truth = self.current_truth
        self.loop_start_result_index = len(self.results)
        self.logger.emit({"t": snap.t_s - self.start_time_s,
                          "monotonic_s": snap.t_s, "type": "accuracy_loop_start",
                          "label": label}, priority=True)
        return snap

    def closure(self, label: str) -> dict:
        if self.segment_label is not None:
            raise ValueError("finish the active segment before checking closure")
        if (self.loop_start_snapshot is None or self.loop_start_truth is None
                or self.loop_start_result_index is None or self.last_end_snapshot is None):
            raise ValueError("mark a loop start and finish four segments first")
        origin, end = self.loop_start_snapshot, self.last_end_snapshot
        estimated = relative_delta(origin.x_m, origin.y_m, origin.yaw_unwrapped_rad,
                                   end.x_m, end.y_m, end.yaw_unwrapped_rad)
        actual = relative_delta(self.loop_start_truth.x_m, self.loop_start_truth.y_m,
                                math.radians(self.loop_start_truth.yaw_deg),
                                self.current_truth.x_m,
                                self.current_truth.y_m, math.radians(self.current_truth.yaw_deg))
        position_error = math.hypot(estimated[0] - actual[0], estimated[1] - actual[1])
        yaw_error = math.degrees(estimated[2] - actual[2])
        recent_segments = self.results[self.loop_start_result_index:]
        invalid = len(recent_segments) != 4 or any(
            result["status"] == "INVALID" for result in recent_segments
        )
        row = {
            "label": label,
            "status": "INVALID" if invalid else (
                "PASS" if position_error <= 0.15 and abs(yaw_error) <= 5.0 else "FAIL"
            ),
            "truth_x_cm": round(self.current_truth.x_m * 100, 2),
            "truth_y_cm": round(self.current_truth.y_m * 100, 2),
            "truth_yaw_deg": round(self.current_truth.yaw_deg, 2),
            "estimate_dx_cm": round(estimated[0] * 100, 2),
            "estimate_dy_cm": round(estimated[1] * 100, 2),
            "estimate_dyaw_deg": round(math.degrees(estimated[2]), 2),
            "position_error_cm": round(position_error * 100, 2),
            "yaw_error_deg": round(yaw_error, 2),
            "invalid_reasons": "loop requires four valid segments" if invalid else "",
        }
        self.closures.append(row)
        with (self.output_dir / "closures.csv").open("a", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(row))
            if stream.tell() == 0:
                writer.writeheader()
            writer.writerow(row)
        self.logger.emit({"t": end.t_s - self.start_time_s,
                          "monotonic_s": end.t_s, "type": "accuracy_closure",
                          **row}, priority=True)
        self.loop_start_snapshot = None
        self.loop_start_truth = None
        self.loop_start_result_index = None
        return row

    def status(self) -> str:
        recent = self._recent(5.0)
        if not recent:
            return "waiting for fused pose"
        last = recent[-1]
        rate = 0.0 if len(recent) < 2 else (
            (last.scan_count - recent[0].scan_count) / max(0.001, last.t_s - recent[0].t_s)
        )
        return ("fusion=%s slam=%s T265_conf=%s anchor_age=%s scan=%.2fHz "
                "pose=%s" % (
                    last.fusion_state, last.slam_state, last.t265_confidence,
                    "-" if last.anchor_age_s is None else "%.2fs" % last.anchor_age_s,
                    rate,
                    "-" if last.x_m is None else "(%.2f, %.2f, %.1f°)" % (
                        last.x_m, last.y_m, math.degrees(last.yaw_rad)),
                ))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", required=True, help="existing board TOML; read only")
    parser.add_argument("--output", required=True, help="new directory outside the Git checkout")
    args = parser.parse_args(argv)
    output_dir = Path(args.output).resolve()
    if output_dir.exists():
        parser.error("output directory already exists; choose a new run name")
    if REPO_ROOT == output_dir or REPO_ROOT in output_dir.parents:
        parser.error("output must be outside the Git checkout")
    config = test_config_from_board(load_v2_config(args.config))
    output_dir.mkdir(parents=True)
    logger = JsonlEventLogger(output_dir / "events.jsonl")
    logger.emit({
        "type": "accuracy_profile", "t265_min_tracker_confidence": MIN_TRACKER_CONFIDENCE,
        "position_limit_cm": POSITION_LIMIT_M * 100, "yaw_limit_deg": YAW_LIMIT_DEG,
    }, priority=True)
    runtime = None
    session = None
    try:
        runtime = build_runtime(config, RuntimeMode.HARDWARE_PROBE,
                                event_logger=logger, sensor_only=True)
        runtime.d500_source = ScanOnlyD500Source(
            runtime, config.d500.port, config.d500.baudrate, logger,
        )
        session = AccuracySession(runtime, logger, output_dir)
        session.start()
        print("SENSOR-ONLY; no C10B or relay port opened. Commands: status, preflight, begin LABEL [move|pivot], end, truth X_CM Y_CM YAW_DEG, loop-start LABEL, closure LABEL, quit", flush=True)
        print("accuracy> ", end="", flush=True)
        while not session.stop_event.is_set():
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 1.0)
                if not ready:
                    continue
                line = sys.stdin.readline()
                if not line:
                    break
                line = line.strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                print("accuracy> ", end="", flush=True)
                continue
            parts = line.split()
            try:
                if parts[0] == "status" and len(parts) == 1:
                    print(session.status(), flush=True)
                elif parts[0] == "preflight" and len(parts) == 1:
                    ok, reasons = session.preflight()
                    print("PREFLIGHT PASS" if ok else "PREFLIGHT FAIL: " + "; ".join(reasons), flush=True)
                elif parts[0] == "begin" and len(parts) in {2, 3}:
                    session.begin(parts[1], parts[2] if len(parts) == 3 else "move")
                    print("BEGIN %s — move only now" % parts[1], flush=True)
                elif parts[0] == "end" and len(parts) == 1:
                    session.end()
                    print("END captured; enter independently measured floor truth", flush=True)
                elif parts[0] == "truth" and len(parts) == 4:
                    row = session.truth(FloorPose(float(parts[1]) / 100,
                                                   float(parts[2]) / 100, float(parts[3])))
                    print("%s %s: measured (%+.2f,%+.2f,%+.2f°), estimated (%+.2f,%+.2f,%+.2f°); position error %.2f cm, yaw error %+.2f° %s" % (
                        row["status"], row["label"],
                        row["actual_dx_cm"], row["actual_dy_cm"], row["actual_dyaw_deg"],
                        row["estimate_dx_cm"], row["estimate_dy_cm"], row["estimate_dyaw_deg"],
                        row["position_error_cm"], row["yaw_error_deg"], row["invalid_reasons"]
                    ), flush=True)
                elif parts[0] == "loop-start" and len(parts) == 2:
                    session.mark_loop_start(parts[1])
                    print("LOOP START %s" % parts[1], flush=True)
                elif parts[0] == "closure" and len(parts) == 2:
                    row = session.closure(parts[1])
                    print("CLOSURE %s %s: position error %.2f cm, yaw error %+.2f°" % (
                        row["status"], row["label"], row["position_error_cm"],
                        row["yaw_error_deg"]
                    ), flush=True)
                elif parts[0] == "quit" and len(parts) == 1:
                    break
                else:
                    print("Unknown command. Use status, preflight, begin LABEL [move|pivot], end, truth X_CM Y_CM YAW_DEG, loop-start LABEL, closure LABEL, quit", flush=True)
            except (ValueError, RuntimeError) as exc:
                print("REJECTED: %s" % exc, flush=True)
            print("accuracy> ", end="", flush=True)
        return 0 if session.error is None else 2
    finally:
        try:
            if session is not None:
                session.close()
            elif runtime is not None:
                runtime.close()
        finally:
            logger.close()
            print("Logs: %s" % output_dir, flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
