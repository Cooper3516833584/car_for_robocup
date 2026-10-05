from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import subprocess

import numpy as np

HERE = Path(__file__).resolve().parent
CODE_ROOT = HERE.parent
if (CODE_ROOT / "components").exists():
    sys.path.insert(0, str(CODE_ROOT))

try:
    from components.task_board_reader import (
        OCRToken,
        RapidOCRBackend,
        TaskBoardConfig,
        TaskBoardReader,
        TaskCounts,
        find_task_board_quad,
        parse_task_tokens,
        warp_task_board,
    )
except ModuleNotFoundError:
    sys.path.insert(0, str(HERE))
    from task_board_reader import (
        OCRToken,
        RapidOCRBackend,
        TaskBoardConfig,
        TaskBoardReader,
        TaskCounts,
        find_task_board_quad,
        parse_task_tokens,
        warp_task_board,
    )

try:
    import cv2
except ImportError:
    cv2 = None


def token(text: str, score: float = 0.95, y: float = 0.0) -> OCRToken:
    box = ((10.0, y), (500.0, y), (500.0, y + 40.0), (10.0, y + 40.0))
    return OCRToken(text, score, box)


class ParserTests(unittest.TestCase):
    def test_count_is_bound_to_quantity_semantics_not_first_number(self):
        result = parse_task_tokens([OCRToken(
            "红色1号地块需要投入2个物资 蓝色2号地块需要投入1个物资 绿色3号地块需要投入1个物资"
        )])
        self.assertEqual(result.counts, TaskCounts(2, 1, 1), result.reason)

    def test_quantity_layouts_and_long_color_spans(self):
        red_spans = (
            "自由投放区域需要投入2个救援物资", "区域物资数量：2", "区域 2 个物资",
            "区域 2 件救援物资", "区域数量：2", "区域需投：二个物资",
            "区域需投入：两件物资", "区域需投放，兩个物资", "区域投放（2）个物资",
            "区域需要投放2个物资", "1号区域投入2", "区域2", "2号区域2号位置",
            "自由投放区域这是长度超过原先十六字符限制的任务板说明需要投入2个救援物资",
            "自由区域这是长度远远超过原先字符距离限制的任务板说明2",
            "12号区域物资数量：2",
        )
        for span in red_spans:
            with self.subTest(span=span):
                result = parse_task_tokens([OCRToken(f"红色{span}蓝色数量1绿色数量1")])
                self.assertEqual(result.counts, TaskCounts(2, 1, 1), result.reason)

    def test_ambiguous_span_is_rejected_even_when_missing_color_inference_is_enabled(self):
        result = parse_task_tokens([OCRToken("红色1号区域2号位置蓝色1绿色1")])
        self.assertFalse(result.valid)
        self.assertIn("ambiguous", result.reason)

    def test_conflicting_quantity_semantics_are_not_guessed(self):
        result = parse_task_tokens([OCRToken("红色投入2个物资，物资数量3蓝色1绿色1")])
        self.assertFalse(result.valid)
        self.assertIn("ambiguous", result.reason)

    def test_semantic_count_does_not_truncate_invalid_values_or_use_region_number(self):
        for count in ("12", "5", "一二", "十", "-1", "1.5"):
            with self.subTest(count=count):
                result = parse_task_tokens([OCRToken(f"红色1号区域投入{count}个物资蓝色1绿色1")])
                self.assertFalse(result.valid)
                self.assertIn("invalid count", result.reason)

    def test_quantity_semantics_never_cross_the_next_color(self):
        result = parse_task_tokens([OCRToken("红色区域需要投入蓝色2个物资绿色2个物资")],
                                   allow_missing_color_inference=False)
        self.assertFalse(result.valid)
        self.assertIn("red", result.reason)

    def test_region_identifiers_split_from_quantity_text_do_not_override_semantics(self):
        for region in ("1号区域", "12号区域"):
            with self.subTest(region=region):
                result = parse_task_tokens([OCRToken(text) for text in (
                    "红色" + region, "需要投入2个物资", "蓝色2号区域", "需要投入1个物资",
                    "绿色3号区域", "需要投入1个物资",
                )])
                self.assertEqual(result.counts, TaskCounts(2, 1, 1), result.reason)

    def test_all_fifteen_legal_combinations(self):
        for red in range(5):
            for blue in range(5 - red):
                green = 4 - red - blue
                with self.subTest(red=red, blue=blue, green=green):
                    result = parse_task_tokens([OCRToken(f"红{red}蓝{blue}绿{green}")])
                    self.assertEqual(result.counts, TaskCounts(red, blue, green))

    def test_repeated_color_conflict_on_same_line(self):
        result = parse_task_tokens([OCRToken("红2红3蓝1绿1")])
        self.assertFalse(result.valid)
        self.assertIn("conflicting", result.reason)

    def test_next_color_cannot_supply_missing_count(self):
        result = parse_task_tokens([OCRToken("红色地块蓝色1绿色1")], allow_missing_color_inference=False)
        self.assertFalse(result.valid)
        self.assertIn("red", result.reason)

    def test_invalid_numbers_are_not_truncated_or_inferred(self):
        for count in ("12", "5", "一二", "十", "-1", "1.5"):
            with self.subTest(count=count):
                result = parse_task_tokens([OCRToken(f"红{count}蓝1绿1")])
                self.assertFalse(result.valid)
                self.assertIn("invalid count", result.reason)

    def test_low_score_counts_do_not_enter_parser(self):
        result = parse_task_tokens([token("红2", 0.1), token("蓝1", y=90), token("绿1", y=180)],
                                   allow_missing_color_inference=False)
        self.assertFalse(result.valid)

    def test_two_missing_colors_are_not_inferred(self):
        self.assertFalse(parse_task_tokens([OCRToken("红2")]).valid)

    def test_inference_lowers_confidence(self):
        full = parse_task_tokens([OCRToken("红2蓝1绿1", 0.95)])
        partial = parse_task_tokens([OCRToken("红2蓝1", 0.95)])
        self.assertLess(partial.confidence, full.confidence)

    def test_strict_integer_counts(self):
        for values in ((1.5, 1.5, 1), (True, 2, 1), (-1, 4, 1), (5, 0, -1)):
            with self.subTest(values=values), self.assertRaises(ValueError):
                TaskCounts(*values)

    def test_no_boxes_split_sentence(self):
        result = parse_task_tokens([OCRToken(text) for text in ("红色投入", "2个", "蓝色投入", "1个", "绿色投入", "1个")])
        self.assertEqual(result.counts, TaskCounts(2, 1, 1))

    def test_exact_sample_board(self) -> None:
        parsed = parse_task_tokens(
            [
                token("□红色地块投入2个物资；", y=20),
                token("□蓝色地块投入1个物资；", y=90),
                token("□绿色地块投入1个物资；", y=160),
            ]
        )
        self.assertTrue(parsed.valid, parsed.reason)
        self.assertEqual(parsed.counts, TaskCounts(2, 1, 1))
        self.assertEqual(parsed.inferred_colors, ())

    def test_chinese_digits_are_accepted(self) -> None:
        parsed = parse_task_tokens(
            [
                token("红色地块投入二个物资", y=20),
                token("蓝色地块投入一个物资", y=90),
                token("绿色地块投入一个物资", y=160),
            ]
        )
        self.assertEqual(parsed.counts, TaskCounts(2, 1, 1))

    def test_split_boxes_on_same_line_are_joined(self) -> None:
        tokens = [
            OCRToken("红色地块投入", 0.94, ((10, 20), (220, 20), (220, 60), (10, 60))),
            OCRToken("2个物资", 0.96, ((230, 21), (340, 21), (340, 61), (230, 61))),
            token("蓝色地块投入1个物资", y=100),
            token("绿色地块投入1个物资", y=180),
        ]
        parsed = parse_task_tokens(tokens)
        self.assertEqual(parsed.counts, TaskCounts(2, 1, 1))

    def test_one_missing_color_can_be_inferred_from_total_four(self) -> None:
        parsed = parse_task_tokens(
            [token("红色地块投入2个物资", y=20), token("蓝色地块投入1个物资", y=90)],
            allow_missing_color_inference=True,
        )
        self.assertEqual(parsed.counts, TaskCounts(2, 1, 1))
        self.assertEqual(parsed.inferred_colors, ("green",))

    def test_missing_color_is_rejected_when_inference_disabled(self) -> None:
        parsed = parse_task_tokens(
            [token("红色地块投入2个物资", y=20), token("蓝色地块投入1个物资", y=90)],
            allow_missing_color_inference=False,
        )
        self.assertFalse(parsed.valid)
        self.assertIn("green", parsed.reason or "")

    def test_invalid_sum_is_rejected(self) -> None:
        parsed = parse_task_tokens(
            [
                token("红色地块投入2个物资", y=20),
                token("蓝色地块投入2个物资", y=90),
                token("绿色地块投入1个物资", y=160),
            ]
        )
        self.assertFalse(parsed.valid)
        self.assertIn("add up to 4", parsed.reason or "")

    def test_conflicting_same_color_is_rejected(self) -> None:
        parsed = parse_task_tokens(
            [
                token("红色地块投入2个物资", y=20),
                token("红色地块投入3个物资", y=90),
                token("蓝色地块投入1个物资", y=160),
            ]
        )
        self.assertFalse(parsed.valid)
        self.assertIn("conflicting", parsed.reason or "")


class FakeOCR:
    def __init__(self, sequences):
        self.sequences = list(sequences)
        self.index = 0

    def recognize(self, _image):
        if not self.sequences:
            return ()
        value = self.sequences[min(self.index, len(self.sequences) - 1)]
        self.index += 1
        return value


@unittest.skipIf(cv2 is None, "OpenCV unavailable")
class ReaderTests(unittest.TestCase):
    def test_two_votes_do_not_establish_consensus(self):
        reader = TaskBoardReader(TaskBoardConfig(enhanced_ocr_retry=False), FakeOCR([[OCRToken("红2蓝1绿1")]]))
        result = reader.recognize_frames([self.frame(), self.frame()])
        self.assertIsNone(result.task)
        self.assertEqual(result.votes, 2)

    def test_maximum_frames_does_not_pull_extra_frame(self):
        seen = []
        def frames():
            for index in range(20):
                seen.append(index)
                yield self.frame()
        reader = TaskBoardReader(TaskBoardConfig(maximum_frames=3, enhanced_ocr_retry=False), FakeOCR([()]))
        self.assertFalse(reader.recognize_frames(frames()).valid)
        self.assertEqual(len(seen), 3)

    def test_backend_exception_is_explicit_failure(self):
        ocr = FakeOCR([])
        with patch.object(ocr, "recognize", side_effect=RuntimeError("model missing")):
            result = TaskBoardReader(ocr_backend=ocr).recognize_frame(self.frame())
        self.assertIsNone(result.task)
        self.assertIn("model missing", result.reason)

    def test_large_bright_wall_is_not_a_board(self):
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        image[:400] = 240
        self.assertIsNone(find_task_board_quad(image))

    def test_camera_is_released_after_interrupt(self):
        from unittest.mock import MagicMock
        capture = MagicMock()
        capture.isOpened.return_value = True
        capture.read.side_effect = KeyboardInterrupt
        with patch("components.task_board_reader.cv2.VideoCapture", return_value=capture):
            with self.assertRaises(KeyboardInterrupt):
                TaskBoardReader().recognize_camera("camera")
        capture.release.assert_called_once()

    def test_camera_open_failure_has_no_task(self):
        from unittest.mock import MagicMock
        capture = MagicMock()
        capture.isOpened.return_value = False
        with patch("components.task_board_reader.cv2.VideoCapture", return_value=capture):
            result = TaskBoardReader().recognize_camera("missing")
        self.assertIsNone(result.task)
        capture.release.assert_called_once()

    @staticmethod
    def frame() -> np.ndarray:
        return np.full((480, 640, 3), 180, dtype=np.uint8)

    def test_consensus_requires_repeated_valid_tuple(self) -> None:
        correct = [
            token("红色地块投入2个物资", y=20),
            token("蓝色地块投入1个物资", y=90),
            token("绿色地块投入1个物资", y=160),
        ]
        wrong_but_valid = [
            token("红色地块投入1个物资", y=20),
            token("蓝色地块投入2个物资", y=90),
            token("绿色地块投入1个物资", y=160),
        ]
        config = TaskBoardConfig(
            required_consensus_votes=3,
            maximum_frames=6,
            full_frame_fallback=True,
            enhanced_ocr_retry=False,
        )
        reader = TaskBoardReader(config, FakeOCR([wrong_but_valid, correct, correct, correct]))
        result = reader.recognize_frames([self.frame() for _ in range(4)])
        self.assertTrue(result.valid, result.reason)
        self.assertEqual(result.counts, TaskCounts(2, 1, 1))
        self.assertEqual(result.votes, 3)

    def test_reader_never_defaults_after_failed_ocr(self) -> None:
        config = TaskBoardConfig(required_consensus_votes=1, maximum_frames=1, enhanced_ocr_retry=False)
        reader = TaskBoardReader(config, FakeOCR([()]))
        result = reader.recognize_frame(self.frame())
        self.assertFalse(result.valid)
        self.assertIsNone(result.task)

    def test_board_detector_and_warp_on_synthetic_quad(self) -> None:
        image = np.full((720, 1280, 3), 55, dtype=np.uint8)
        source = np.full((300, 700, 3), 245, dtype=np.uint8)
        cv2.rectangle(source, (2, 2), (697, 297), (10, 10, 10), 5)
        cv2.putText(source, "TASK 2 1 1", (80, 170), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 0), 4)
        src = np.array([[0, 0], [699, 0], [699, 299], [0, 299]], dtype=np.float32)
        dst = np.array([[250, 150], [1030, 210], [950, 520], [310, 560]], dtype=np.float32)
        matrix = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(source, matrix, (1280, 720))
        mask = cv2.warpPerspective(np.full((300, 700), 255, dtype=np.uint8), matrix, (1280, 720))
        image[mask > 0] = warped[mask > 0]

        config = replace(TaskBoardConfig(), minimum_board_brightness=100.0)
        quad = find_task_board_quad(image, config)
        self.assertIsNotNone(quad)
        rectified = warp_task_board(image, quad, width=1200, height=520)
        self.assertEqual(rectified.shape[:2], (520, 1200))
        self.assertGreater(float(rectified.mean()), 150.0)


class BackendTests(unittest.TestCase):
    def test_numpy_output_scores_are_supported(self):
        backend = RapidOCRBackend()
        output = SimpleNamespace(txts=("红2蓝1绿1",), scores=np.array([0.95]),
                                 boxes=np.array([[[0, 0], [100, 0], [100, 20], [0, 20]]]))
        backend._engine = lambda *args, **kwargs: output
        self.assertEqual(backend.recognize(np.zeros((30, 100, 3)))[0].text, "红2蓝1绿1")

    def test_import_and_construction_have_no_camera_model_or_threads(self):
        script = '''
import sys, threading, types
from unittest.mock import patch
def forbidden(*args, **kwargs):
    raise AssertionError("import side effect")
try:
    import cv2
except ImportError:
    cv2 = types.ModuleType('cv2')
    cv2.VideoCapture = forbidden
    sys.modules['cv2'] = cv2
sys.modules['rapidocr'] = types.SimpleNamespace(RapidOCR=forbidden)
before = tuple(threading.enumerate())
with patch.object(cv2, 'VideoCapture', forbidden), patch.object(threading.Thread, 'start', forbidden):
    from components.task_board_reader import TaskBoardReader
    import task_board_startup
    reader = TaskBoardReader()
    assert reader.ocr._engine is None
assert tuple(threading.enumerate()) == before
'''
        result = subprocess.run([sys.executable, "-c", script], cwd=CODE_ROOT,
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
