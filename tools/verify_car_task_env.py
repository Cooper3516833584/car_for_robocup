#!/usr/bin/env python3
"""Import task dependencies and run static task-board images; never open hardware."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.metadata
import json
from pathlib import Path
import sys
import time
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--source-bundle", type=Path)
    args = parser.parse_args()
    environment = {"python": sys.executable, "version": sys.version, "modules": {}}
    for name in ("numpy", "cv2", "rclpy", "pyrealsense2", "onnxruntime", "rapidocr"):
        module = importlib.import_module(name)
        environment["modules"][name] = {"path": module.__file__, "version": getattr(module, "__version__", None)}
    for name in ("rapidocr", "onnxruntime"):
        environment["modules"][name]["version"] = importlib.metadata.version(name)
    print(json.dumps({"environment": environment}, ensure_ascii=False), flush=True)
    verified_files = None
    if args.source_bundle:
        with zipfile.ZipFile(args.source_bundle) as archive:
            manifest = json.loads(archive.read("manifest.json"))
        for entry in manifest["files"]:
            checksum = hashlib.sha256((ROOT / entry["path"]).read_bytes()).hexdigest()
            if checksum != entry["sha256"]:
                raise RuntimeError(f"source differs from sync snapshot: {entry['path']}")
        if any((ROOT / name).exists() for name in manifest["delete"]):
            raise RuntimeError("a removed legacy file is still present")
        verified_files = len(manifest["files"])
    import cv2
    from components.task_board_reader import TaskBoardReader

    dataset = args.dataset.resolve()
    manifest = json.loads((dataset / "manifest.json").read_text(encoding="utf-8"))
    views = {view["file"]: view for view in manifest["views"]}
    paths = sorted(path for path in dataset.iterdir() if path.suffix.lower() in {".png", ".jpg", ".jpeg"})
    reader = TaskBoardReader()
    rows = []
    for index, path in enumerate(paths, start=1):
        started = time.monotonic()
        result = reader.recognize_frame(cv2.imread(str(path))).to_json_dict()
        view = views.get(path.name)
        expected = None if view is None else view.get("expected", manifest["expected"])
        row = {"file": path.name, "result": result, "elapsed_ms": (time.monotonic() - started) * 1000,
               "expected": expected, "matches_expected": None if expected is None else result["task"] == expected}
        rows.append(row)
        print(json.dumps({"progress": f"{index}/{len(paths)}", **row}, ensure_ascii=False), flush=True)
    report = {"environment": environment, "verified_source_files": verified_files,
              "total": len(rows), "valid": sum(row["result"]["valid"] for row in rows),
              "expected_views": sum(row["expected"] is not None for row in rows),
              "matched_views": sum(row["matches_expected"] is True for row in rows), "images": rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key not in {"images", "environment"}}), flush=True)
    return 0 if report["valid"] == report["total"] and report["matched_views"] == report["expected_views"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
