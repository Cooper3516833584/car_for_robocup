"""Regressions for backend thread reset and caller-affinity isolation."""

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components import yolo_cpu


class CpuDetectorTests(unittest.TestCase):
    def test_backend_setup_reset_is_corrected_and_control_affinity_restored(self):
        torch = Mock()
        threads = [7]
        affinity = [set(range(8))]
        torch.set_num_threads.side_effect = lambda count: threads.__setitem__(0, count)

        def predict(frame, **kwargs):
            self.assertEqual(threads[0], 1)
            self.assertEqual(affinity[0], {4, 5})
            self.assertEqual((kwargs["imgsz"], kwargs["device"]), (320, "cpu"))
            threads[0] = 7  # Simulate Ultralytics' first CPU backend setup.
            return "result"

        backend = Mock()
        backend.predict.side_effect = predict
        with patch.dict(sys.modules, {"torch": torch}), \
             patch.object(yolo_cpu.os, "sched_getaffinity", create=True,
                          side_effect=lambda _: affinity[0].copy()), \
             patch.object(yolo_cpu.os, "sched_setaffinity", create=True,
                          side_effect=lambda _, cpus: affinity.__setitem__(0, set(cpus))), \
             patch.object(yolo_cpu, "pin_vision_worker",
                          side_effect=lambda: affinity.__setitem__(0, {4, 5})):
            detector = yolo_cpu.CpuDetector(backend)
            self.assertEqual(detector.predict("frame"), "result")
            self.assertEqual(threads[0], 1)
            self.assertEqual(affinity[0], set(range(8)))
            self.assertEqual(detector.predict("next frame"), "result")
            self.assertEqual(affinity[0], set(range(8)))

    def test_prediction_failure_still_restores_affinity_and_threads(self):
        torch = Mock()
        backend = Mock()
        backend.predict.side_effect = RuntimeError("backend failed")
        with patch.dict(sys.modules, {"torch": torch}), \
             patch.object(yolo_cpu.os, "sched_getaffinity", create=True, return_value={0, 4}), \
             patch.object(yolo_cpu.os, "sched_setaffinity", create=True) as restore, \
             patch.object(yolo_cpu, "pin_vision_worker"):
            with self.assertRaisesRegex(RuntimeError, "backend failed"):
                yolo_cpu.CpuDetector(backend).predict("frame")
        restore.assert_called_once_with(0, {0, 4})
        self.assertEqual(torch.set_num_threads.call_count, 2)
        torch.set_num_threads.assert_called_with(1)

    def test_platform_without_affinity_still_predicts(self):
        backend, torch = Mock(), Mock()
        with patch.dict(sys.modules, {"torch": torch}), \
             patch.object(yolo_cpu.os, "sched_getaffinity", create=True,
                          side_effect=AttributeError("not supported")), \
             patch.object(yolo_cpu, "pin_vision_worker", return_value=[]):
            self.assertIs(yolo_cpu.CpuDetector(backend).predict("frame"), backend.predict.return_value)


if __name__ == "__main__":
    unittest.main()
