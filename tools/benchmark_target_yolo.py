#!/usr/bin/env python3
"""Stationary camera/YOLO timing only; never opens motor, servo or alarm."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import statistics
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path,
                        default=Path(__file__).resolve().parents[1] / "models/best_car.pt")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--repeats", type=int, default=5)
    args = parser.parse_args()
    started = time.monotonic()
    import cv2
    import torch
    from ultralytics import YOLO
    import_s = time.monotonic() - started
    if not args.weights.is_file():
        raise FileNotFoundError(args.weights)
    model_started = time.monotonic()
    detector = YOLO(str(args.weights))
    load_s = time.monotonic() - model_started
    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    try:
        if not capture.isOpened():
            raise RuntimeError("camera cannot open")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        reads = []
        for _ in range(12):
            read_at = time.monotonic()
            ok, frame = capture.read()
            reads.append(time.monotonic() - read_at)
            if not ok or frame is None:
                raise RuntimeError("camera read failed")
    finally:
        capture.release()
    allowed = os.sched_getaffinity(0)
    # Highest-frequency policy identifies the big cores on this heterogeneous CPU.
    policies = []
    for policy in Path("/sys/devices/system/cpu/cpufreq").glob("policy*"):
        maximum = int((policy / "cpuinfo_max_freq").read_text())
        cpus = {int(cpu) for cpu in (policy / "related_cpus").read_text().split()}
        policies.append((maximum, cpus))
    big = set().union(*(cpus for freq, cpus in policies if freq == max(f for f, _ in policies))) & allowed
    report = {"import_s": import_s, "model_load_s": load_s,
              "camera_read_median_s": statistics.median(reads[2:]),
              "camera_shape": list(frame.shape), "model_classes": detector.names,
              "model_parameters": sum(p.numel() for p in detector.model.parameters()),
              "torch": torch.__version__, "ultralytics": __import__("ultralytics").__version__,
              "cpu_policies": [(freq, sorted(cpus)) for freq, cpus in policies],
              "cases": []}
    print(json.dumps({k: v for k, v in report.items() if k != "cases"}), flush=True)
    try:
        for affinity_name, affinity in (("all", allowed), ("big", big)):
            if not affinity:
                continue
            os.sched_setaffinity(0, affinity)
            for threads in (1, 2, 4):
                torch.set_num_threads(threads)
                for imgsz in (416, 320):
                    times, speeds, counts = [], [], []
                    for _ in range(args.repeats + 1):
                        t0 = time.monotonic()
                        result = detector.predict(frame, imgsz=imgsz, conf=.5,
                                                  device="cpu", verbose=False)[0]
                        times.append(time.monotonic() - t0)
                        speeds.append(result.speed)
                        counts.append(len(result.boxes))
                    case = {"affinity": affinity_name, "cpus": sorted(affinity),
                            "threads": threads, "imgsz": imgsz,
                            "cold_s": times[0], "median_s": statistics.median(times[1:]),
                            "max_s": max(times[1:]), "speed_ms": speeds[-1],
                            "box_counts": counts}
                    report["cases"].append(case)
                    print(json.dumps(case), flush=True)
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    args.output.write_text(json.dumps(report, indent=2) + "\n")
    finally:
        os.sched_setaffinity(0, allowed)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
