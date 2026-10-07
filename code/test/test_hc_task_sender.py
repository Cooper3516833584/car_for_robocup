from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from components.hc_task_sender import build_task_message, send_task_once


class HCTaskSenderTests(unittest.TestCase):
    counts = SimpleNamespace(red=2, blue=1, green=1)

    def test_raw_message_and_exactly_one_write(self):
        self.assertEqual(build_task_message(self.counts), b"TASK,2,1,1\n")
        driver = Mock()
        driver.wait_connected.return_value = True
        factory = Mock(return_value=driver)
        self.assertTrue(send_task_once(self.counts, driver_factory=factory))
        self.assertFalse(factory.call_args.kwargs["bridge_envelope"])
        self.assertEqual(factory.call_args.kwargs["baudrate"], 115200)
        driver.write.assert_called_once_with(b"TASK,2,1,1\n")
        driver.close.assert_called_once()

    def test_bridge_and_custom_template(self):
        driver = Mock()
        factory = Mock(return_value=driver)
        self.assertTrue(send_task_once(self.counts, bridge_envelope=True,
                                      template="{green}:{blue}:{red}", driver_factory=factory))
        self.assertTrue(factory.call_args.kwargs["bridge_envelope"])
        driver.write.assert_called_once_with(b"1:1:2")

    def test_connection_timeout_does_not_write(self):
        driver = Mock()
        driver.wait_connected.return_value = False
        self.assertFalse(send_task_once(self.counts, driver_factory=Mock(return_value=driver)))
        driver.write.assert_not_called()
        driver.close.assert_called_once()

    def test_failures_close_without_resending(self):
        for method in ("start", "wait_connected", "write", "close"):
            with self.subTest(method=method):
                driver = Mock()
                getattr(driver, method).side_effect = OSError(method)
                self.assertFalse(send_task_once(self.counts, driver_factory=Mock(return_value=driver)))
                self.assertLessEqual(driver.write.call_count, 1)
                driver.close.assert_called_once()

    def test_interrupt_closes(self):
        driver = Mock()
        driver.write.side_effect = KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            send_task_once(self.counts, driver_factory=Mock(return_value=driver))
        driver.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
