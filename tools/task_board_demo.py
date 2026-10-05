#!/usr/bin/env python3
"""Manual image/camera smoke test. This command never controls the chassis."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))
from components.task_board_reader import TaskBoardConfig, TaskBoardReader


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path)
    source.add_argument("--camera", help="camera index or stable device path")
    parser.add_argument("--debug-dir", type=Path)
    parser.add_argument("--no-infer-missing", action="store_true")
    args = parser.parse_args(argv)
    reader = TaskBoardReader(TaskBoardConfig(
        debug_directory=args.debug_dir,
        allow_missing_color_inference=not args.no_infer_missing,
    ))
    try:
        if args.image is not None:
            import cv2

            frame = cv2.imread(str(args.image))
            result = reader.recognize_frame(frame)
        else:
            camera = int(args.camera) if args.camera.isdigit() else args.camera
            result = reader.recognize_camera(camera)
    except Exception as exc:
        print(f"task-board test failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result.to_json_dict(), ensure_ascii=False, indent=2))
    return 0 if result.valid else 3


if __name__ == "__main__":
    raise SystemExit(main())
