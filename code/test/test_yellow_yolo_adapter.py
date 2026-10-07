from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.yellow_yolo_adapter import load_detector, select_yellow


def box(cid, confidence, xyxy):
    return SimpleNamespace(cls=SimpleNamespace(item=lambda: cid),
                           conf=SimpleNamespace(item=lambda: confidence),
                           xyxy=[SimpleNamespace(tolist=lambda: xyxy)])


class YellowAdapterTests(unittest.TestCase):
    def test_highest_yellow_and_center(self):
        result = SimpleNamespace(names={0: "red", 3: "yellow"}, boxes=[
            box(0, 0.99, (0, 0, 10, 10)), box(3, 0.6, (0, 0, 10, 10)),
            box(3, 0.9, (20, 40, 100, 80))])
        detection = select_yellow([result])
        self.assertEqual((detection.cx_px, detection.cy_px, detection.width_px,
                          detection.height_px, detection.confidence), (60, 60, 80, 40, 0.9))

    def test_low_conf_empty_and_bad_boxes(self):
        for boxes in (None, [], [box(3, 0.54, (0, 0, 10, 10))],
                      [box(3, float("nan"), (0, 0, 10, 10))],
                      [box(3, 0.8, (20, 0, 10, 10))]):
            self.assertIsNone(select_yellow([SimpleNamespace(names={3: "yellow"}, boxes=boxes)]))

    def test_missing_model_does_not_download(self):
        with self.assertRaises(FileNotFoundError):
            load_detector(Path(__file__).parent / "missing.pt")


if __name__ == "__main__":
    unittest.main()
