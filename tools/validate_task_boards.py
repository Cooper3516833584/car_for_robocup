#!/usr/bin/env python3
"""Validate installed task-board code against an external synthetic manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

import cv2
import numpy as np
from components.task_board_reader import TaskBoardReader, find_task_board_quad, order_quad


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path, help="directory containing manifest.json and view images")
    parser.add_argument("--ocr", action="store_true", help="also run the real RapidOCR backend")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.dataset / "manifest.json").read_text(encoding="utf-8"))
    reader = TaskBoardReader()
    rows = []
    for view in manifest["views"]:
        frame = cv2.imread(str(args.dataset / view["file"]))
        found = find_task_board_quad(frame)
        error = None if found is None else float(np.linalg.norm(
            order_quad(found) - order_quad(view["corners_tl_tr_br_bl"]), axis=1,
        ).mean())
        row = {"file": view["file"], "detected": found is not None,
               "mean_corner_error_px": error, "geometry_passed": error is not None and error <= 15.0}
        if args.ocr:
            started = time.monotonic()
            result = reader.recognize_frame(frame)
            row.update(ocr=result.to_json_dict(), ocr_ms=(time.monotonic() - started) * 1000.0,
                       ocr_passed=result.task == {"red": 2, "blue": 1, "green": 1})
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False), flush=True)
    errors = [row["mean_corner_error_px"] for row in rows if row["detected"]]
    report = {"views": rows, "total": len(rows),
              "detected": sum(row["detected"] for row in rows),
              "mean_corner_error_px": sum(errors) / len(errors) if errors else None,
              "worst_mean_corner_error_px": max(errors) if errors else None,
              "ocr_passed": sum(row.get("ocr_passed", False) for row in rows) if args.ocr else None,
              "passed": all(row["geometry_passed"] and row.get("ocr_passed", True) for row in rows)}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "views"}, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
