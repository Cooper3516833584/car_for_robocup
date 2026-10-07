#!/usr/bin/env python3
"""Live camera size/latency comparison; never opens motion, servo or GPIO."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))
from center_target_route import MODEL_PATH, central_target, pin_vision_worker
from components.yellow_yolo_adapter import load_detector


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=MODEL_PATH)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sizes", type=int, nargs="+", default=[416, 320, 256])
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--trigger-confidence", type=float, default=.5)
    args = parser.parse_args()
    if args.frames < 10 or any(size < 160 or size % 32 for size in args.sizes):
        parser.error("at least 10 frames, input sizes >=160 and divisible by32 required")
    if not args.weights.is_file():
        parser.error(f"weights missing: {args.weights}")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    cpus = pin_vision_worker()
    started = time.monotonic()
    import cv2
    import torch
    import ultralytics
    import_s = time.monotonic() - started
    torch.set_num_threads(1)
    started = time.monotonic()
    model = load_detector(args.weights)
    load_s = time.monotonic() - started
    capture = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    report = {"weights": str(args.weights), "import_s": import_s, "load_s": load_s,
              "cpu_affinity": cpus, "threads": 1, "torch": torch.__version__,
              "ultralytics": ultralytics.__version__,
              "trigger_confidence": args.trigger_confidence, "classes": model.names,
              "note": "live observations of this scene, not a labelled accuracy evaluation",
              "cases": []}

    def read():
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError("camera frame unavailable")
        return frame

    def predict(frame, size):
        return model.predict(frame, imgsz=size, conf=.25, device="cpu", verbose=False)[0]

    try:
        if not capture.isOpened():
            raise RuntimeError("camera cannot open")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_FPS, 30)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        for _ in range(12):
            frame = read()
        cv2.imwrite(str(args.output_dir / "scene.jpg"), frame)
        report["camera_shape"] = list(frame.shape)
        report["camera_reported_fps"] = capture.get(cv2.CAP_PROP_FPS)
        report["camera_fourcc"] = int(capture.get(cv2.CAP_PROP_FOURCC))
        print(json.dumps({k: v for k, v in report.items() if k != "cases"}), flush=True)
        for size in args.sizes:
            for _ in range(3):
                predict(read(), size)
                # The first predictor setup resets this setting to seven.
                torch.set_num_threads(1)
            rows, detected, central, labels = [], 0, 0, Counter()
            best_result, best_confidence = None, -1.
            wall_start = time.monotonic()
            for index in range(args.frames):
                t0 = time.monotonic()
                frame = read()
                t1 = time.monotonic()
                result = predict(frame, size)
                t2 = time.monotonic()
                boxes = [] if result.boxes is None else result.boxes.data.cpu().tolist()
                accepted = [box for box in boxes if box[4] >= args.trigger_confidence]
                detected += bool(accepted)
                central += central_target(boxes, frame.shape[1], frame.shape[0],
                                          args.trigger_confidence) is not None
                labels.update(str(model.names[int(box[5])]) for box in accepted)
                maximum = max((box[4] for box in boxes), default=0.)
                if maximum > best_confidence:
                    best_confidence, best_result = maximum, result
                rows.append({"frame": index, "read_ms": (t1-t0)*1000,
                             "predict_ms": (t2-t1)*1000, "total_ms": (time.monotonic()-t0)*1000,
                             "speed_ms": result.speed, "max_confidence": maximum, "boxes": boxes})
            wall_s = time.monotonic() - wall_start
            case = {"imgsz": size, "frames": args.frames, "wall_s": wall_s,
                    "live_fps": args.frames / wall_s,
                    "predict_median_ms": statistics.median(row["predict_ms"] for row in rows),
                    "read_median_ms": statistics.median(row["read_ms"] for row in rows),
                    "total_p95_ms": sorted(row["total_ms"] for row in rows)[int(.95*(len(rows)-1))],
                    "detected_frames": detected, "central_trigger_frames": central,
                    "accepted_classes": dict(labels),
                    "confidence_median": statistics.median(row["max_confidence"] for row in rows),
                    "confidence_min": min(row["max_confidence"] for row in rows), "samples": rows}
            report["cases"].append(case)
            print(json.dumps({k: v for k, v in case.items() if k != "samples"}), flush=True)
            if best_result is not None:
                cv2.imwrite(str(args.output_dir / f"detected_{size}.jpg"), best_result.plot())
            (args.output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    finally:
        capture.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
