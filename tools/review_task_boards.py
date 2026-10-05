#!/usr/bin/env python3
"""Run current single-frame recognition on every image and render an offline review."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
import shutil
import sys
import time
from urllib.parse import quote

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

from components.task_board_reader import (
    RapidOCRBackend, TaskBoardConfig, TaskBoardReader, order_quad,
)

IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for path in (Path("C:/Windows/Fonts/msyh.ttc"),
                 Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")):
        if path.exists():
            return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def annotate_tokens(source: Path, destination: Path, tokens: list[dict]) -> None:
    with Image.open(source) as source_image:
        canvas = source_image.convert("RGB")
    draw = ImageDraw.Draw(canvas)
    label_font = font(20)
    for index, token in enumerate(tokens, start=1):
        if token["box"] is None:
            continue
        points = [(float(x), float(y)) for x, y in token["box"]]
        draw.line(points + points[:1], fill="#0077ff", width=3)
        x = max(0, min(point[0] for point in points))
        y = max(0, min(point[1] for point in points) - 25)
        label = str(index)
        box = draw.textbbox((x, y), label, font=label_font)
        draw.rectangle((box[0] - 2, box[1] - 2, box[2] + 3, box[3] + 2), fill="#0077ff")
        draw.text((x, y), label, font=label_font, fill="white")
    canvas.save(destination)


def task_label(result: dict) -> str:
    task = result.get("task")
    if task is None:
        return "未得到有效任务"
    return f"红 {task['red']} / 蓝 {task['blue']} / 绿 {task['green']}"


def render_gallery(output: Path, rows: list[dict]) -> None:
    tile_width, tile_height, columns = 480, 340, 3
    canvas = Image.new("RGB", (tile_width * columns, tile_height * ((len(rows) + columns - 1) // columns)), "#edf1f6")
    draw = ImageDraw.Draw(canvas)
    title_font, label_font = font(20), font(18)
    for index, row in enumerate(rows):
        x, y = (index % columns) * tile_width, (index // columns) * tile_height
        scene = output / row["scene"] if row.get("scene") else None
        if scene is not None:
            with Image.open(scene) as image:
                thumbnail = ImageOps.contain(image.convert("RGB"), (456, 246))
            canvas.paste(thumbnail, (x + (tile_width - thumbnail.width) // 2, y + 12 + (246 - thumbnail.height) // 2))
        draw.text((x + 12, y + 265), row["file"], fill="#182435", font=title_font)
        color = "#157347" if row.get("result", {}).get("task") else "#b34b00"
        draw.text((x + 12, y + 294), task_label(row.get("result", {})), fill=color, font=label_font)
    canvas.save(output / "overview.png")


def render_html(output: Path, report: dict) -> None:
    def esc(value: object) -> str:
        return html.escape(str(value))

    def image_link(path: str, label: str) -> str:
        url = quote(path, safe="/")
        return f'<figure><figcaption>{esc(label)}</figcaption><a href="{url}" target="_blank"><img src="{url}" loading="lazy" alt="{esc(label)}"></a></figure>'

    cards = []
    for row in report["images"]:
        result = row.get("result", {})
        expected = row.get("expected")
        comparison = "数量符合标注" if row.get("matches_expected") else "数量不符合标注"
        if expected is None:
            comparison = "辅助图，无独立场景标注"
        reason = result.get("reason") or ""
        pictures = image_link(row["scene"], "场景：绿色框为当前代码检测到的任务板") if row.get("scene") else ""
        final_attempt = next((attempt for attempt in reversed(row["attempts"])
                              if attempt.get("result", {}).get("source") == result.get("source")), None)
        if final_attempt is not None and final_attempt.get("overlay"):
            pictures += image_link(final_attempt["overlay"], "实际 OCR 输入与文字框（编号对应下方文本）")
        attempts = []
        for attempt in row["attempts"]:
            tokens = "".join(f'<tr><td>{index}</td><td>{esc(token["text"])}</td><td>{token["score"]:.4f}</td></tr>'
                             for index, token in enumerate(attempt["tokens"], start=1))
            attempt_image = image_link(attempt["overlay"], "OCR 文字框") if attempt.get("overlay") else ""
            attempt_result = attempt.get("result") or {}
            attempts.append(f'<details open><summary>{esc(attempt["label"])}</summary>'
                            f'<p>{esc(task_label(attempt_result))} {esc(attempt_result.get("reason") or "")}</p>'
                            f'<table><tr><th>编号</th><th>实际识别文字</th><th>OCR 分数</th></tr>{tokens}</table>'
                            f'<details><summary>查看这次尝试的图片</summary>{attempt_image}</details></details>')
        style = "ok" if result.get("task") else "failed"
        cards.append(f'<article id="{esc(Path(row["file"]).stem)}"><h2>{esc(row["file"])}</h2>'
                     f'<p class="{style}">{esc(task_label(result))} · {esc(comparison)}</p>'
                     f'<p>解析置信度：{result.get("confidence", 0):.4f} · 路径：{esc(result.get("source", "none"))}'
                     f' · 用时：{row["elapsed_ms"]:.0f} ms · 推断颜色：{esc(result.get("inferred_colors", []))}</p>'
                     f'<p>{esc(reason)}</p><div class="images">{pictures}</div>{"".join(attempts)}'
                     f'<p><a href="{quote(row["result_json"], safe="/")}">完整识别 JSON</a> · '
                     f'<a href="{quote(row["original"], safe="/")}">原图</a></p></article>')
    navigation = " · ".join(f'<a href="#{esc(Path(row["file"]).stem)}">{esc(row["file"])}</a>' for row in report["images"])
    document = f'''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>任务板识别检查</title><style>
body{{font:16px/1.6 system-ui,"Microsoft YaHei",sans-serif;background:#edf1f6;color:#182435;margin:0}}
main{{max-width:1450px;margin:auto;padding:24px}} article,header{{background:white;border-radius:12px;padding:24px;margin-bottom:24px}}
h1,h2{{margin-top:0}} a{{color:#066ac9}} .images{{display:grid;grid-template-columns:1fr 1fr;gap:18px}}
figure{{margin:0}} img{{width:100%;height:auto;border:1px solid #dae1eb}} figcaption{{margin:8px 0}}
table{{border-collapse:collapse;width:100%;margin:12px 0}}td,th{{padding:7px;text-align:left;border-bottom:1px solid #dde3ea}}
details{{margin:12px 0}}summary{{cursor:pointer;font-weight:600}}.ok{{color:#157347;font-weight:700}}.failed{{color:#b34b00;font-weight:700}}
@media(max-width:800px){{.images{{grid-template-columns:1fr}}main{{padding:10px}}}}
</style><main><header><h1>任务板识别检查</h1>
<p>输入图片 {report['total']} 张；得到有效任务 {report['valid']} 张；有标注的视角测试 {report['matched_views']}/{report['expected_views']} 张符合数量。</p>
<p>调用当前 TaskBoardReader.recognize_frame 和真实 RapidOCR，保留默认识别配置。这是每张图片的一次单帧识别，votes=1，未执行相机多帧一致性检查。每次 OCR 尝试的文字、分数及输入图均保留。</p>
<p>任务板原图和汇总拼图也进行了识别；它们是辅助图，单独列出，不计入 12 个视角的通过率。蓝框对应 OCR 文字位置，绿色框对应板面检测。</p>
<p>代码 SHA-256：{esc(report['reader_sha256'])}</p><p><a href="summary.json">汇总 JSON</a> · <a href="overview.png">图片总览</a></p><p>{navigation}</p></header>
{''.join(cards)}</main></html>'''
    (output / "index.html").write_text(document, encoding="utf-8")


def main() -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--output", required=True, type=Path, help="new or empty output directory")
    args = parser.parse_args()
    dataset, output = args.dataset.resolve(), args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("output directory is not empty; choose a new directory to preserve previous results")
    inputs = sorted(path for path in dataset.iterdir() if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES)
    if not inputs:
        parser.error("no images in dataset")
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = dataset / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    views = {view["file"]: view for view in manifest.get("views", [])}
    backend = RapidOCRBackend()
    rows = []
    for index, path in enumerate(inputs, start=1):
        directory = output / f"{index:02d}_{path.stem}"
        directory.mkdir()
        original = directory / ("original" + path.suffix.lower())
        shutil.copyfile(path, original)
        row = {"file": path.name, "input_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
               "original": original.relative_to(output).as_posix(), "attempts": []}
        started = time.monotonic()
        frame = cv2.imread(str(path))
        reader = TaskBoardReader(TaskBoardConfig(debug_directory=directory), ocr_backend=backend)
        try:
            result = reader.recognize_frame(frame).to_json_dict()
        except Exception as exc:
            result = {"task": None, "confidence": 0.0, "source": "none", "votes": 1,
                      "reason": f"{type(exc).__name__}: {exc}"}
        row.update(result=result, elapsed_ms=(time.monotonic() - started) * 1000)
        for debug_json in sorted(directory.glob("[0-9][0-9][0-9][0-9]_*.json")):
            payload = json.loads(debug_json.read_text(encoding="utf-8"))
            attempt = {"label": debug_json.stem, **payload}
            ocr_image = directory / f"{debug_json.stem}_ocr.jpg"
            if ocr_image.exists():
                overlay = directory / f"{debug_json.stem}_ocr_boxes.png"
                annotate_tokens(ocr_image, overlay, payload["tokens"])
                attempt["overlay"] = overlay.relative_to(output).as_posix()
            row["attempts"].append(attempt)
        if frame is not None:
            scene = frame.copy()
            quad = result.get("board_quad")
            if quad is not None:
                cv2.polylines(scene, [np.asarray(quad, dtype=np.int32)], True, (0, 255, 0), 3)
            scene_path = directory / "scene.png"
            if not cv2.imwrite(str(scene_path), scene):
                raise OSError(f"could not write {scene_path}")
            row["scene"] = scene_path.relative_to(output).as_posix()
        view = views.get(path.name)
        if view is not None:
            row["expected"] = view.get("expected", manifest.get("expected"))
            row["matches_expected"] = result.get("task") == row["expected"]
            quad = result.get("board_quad")
            row["mean_corner_error_px"] = None if quad is None else float(np.linalg.norm(
                order_quad(quad) - order_quad(view["corners_tl_tr_br_bl"]), axis=1).mean())
        row["result_json"] = (directory / "result.json").relative_to(output).as_posix()
        write_json(directory / "result.json", row)
        rows.append(row)
        print(json.dumps({"progress": f"{index}/{len(inputs)}", "file": path.name,
                          "task": result.get("task"), "source": result.get("source"),
                          "reason": result.get("reason"), "elapsed_ms": round(row["elapsed_ms"])}, ensure_ascii=False), flush=True)
    reader_source = Path(__file__).resolve().parents[1] / "code/components/task_board_reader.py"
    report = {"created_at": datetime.now(timezone.utc).isoformat(), "dataset": str(dataset),
              "recognition_method": "TaskBoardReader.recognize_frame; default configuration; real RapidOCR",
              "reader_sha256": hashlib.sha256(reader_source.read_bytes()).hexdigest(),
              "total": len(rows), "valid": sum(row["result"].get("task") is not None for row in rows),
              "expected_views": sum("expected" in row for row in rows),
              "matched_views": sum(row.get("matches_expected", False) for row in rows), "images": rows}
    write_json(output / "summary.json", report)
    render_gallery(output, rows)
    render_html(output, report)
    print(json.dumps({key: report[key] for key in ("total", "valid", "expected_views", "matched_views")}, ensure_ascii=False), flush=True)
    print(str(output / "index.html"), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
