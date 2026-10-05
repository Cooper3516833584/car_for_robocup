"""Hardware-free tests for the HC-15 UART and ground-station bridge codec."""

from __future__ import annotations

from pathlib import Path
import errno
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.serial_communication import (  # noqa: E402
    FCWirelessBridgeCodec,
    DEFAULT_HC14_PORT,
    DEFAULT_HC15_PORT,
    HC14SerialDriver,
    HC15SerialDriver,
    SerialCommunicationDriver,
    SerialDriverError,
)


class FCWirelessBridgeCodecTests(unittest.TestCase):
    def test_ground_station_compatible_encoding(self) -> None:
        self.assertEqual(
            FCWirelessBridgeCodec.encode(b"\xAA\x22\x01"),
            b"\xBB\x33\x03\xAA\x22\x01",
        )

    def test_fragmented_frame(self) -> None:
        codec = FCWirelessBridgeCodec()
        frame = FCWirelessBridgeCodec.encode(b"\xAA\x22payload")
        self.assertEqual(codec.feed(frame[:1]), [])
        self.assertEqual(codec.feed(frame[1:4]), [])
        self.assertEqual(codec.feed(frame[4:]), [b"\xAA\x22payload"])
        self.assertEqual(codec.stats.decoded_frames, 1)

    def test_noise_resynchronization_and_multiple_frames(self) -> None:
        codec = FCWirelessBridgeCodec()
        stream = (
            b"noise"
            + FCWirelessBridgeCodec.encode(b"one")
            + FCWirelessBridgeCodec.encode(b"two")
        )
        self.assertEqual(codec.feed(stream), [b"one", b"two"])
        self.assertEqual(codec.stats.discarded_bytes, 5)

    def test_invalid_zero_length_recovers(self) -> None:
        codec = FCWirelessBridgeCodec()
        data = b"\xBB\x33\x00" + FCWirelessBridgeCodec.encode(b"ok")
        self.assertEqual(codec.feed(data), [b"ok"])
        self.assertEqual(codec.stats.invalid_lengths, 1)

    def test_payload_limits(self) -> None:
        with self.assertRaises(ValueError):
            FCWirelessBridgeCodec.encode(b"")
        with self.assertRaises(ValueError):
            FCWirelessBridgeCodec.encode(bytes(256))


class HC14SerialDriverValidationTests(unittest.TestCase):
    @staticmethod
    def _fake_termios():
        return SimpleNamespace(
            B9600=9600,
            B115200=115200,
            IGNPAR=1,
            CS8=2,
            CREAD=4,
            CLOCAL=8,
            VMIN=6,
            VTIME=5,
            TCSANOW=0,
            TCIOFLUSH=2,
            TIOCM_DTR=2,
            TIOCM_RTS=4,
            TIOCMBIC=8,
            tcgetattr=mock.Mock(return_value=[0, 0, 0, 0, 0, 0, [0] * 32]),
            tcsetattr=mock.Mock(),
            tcflush=mock.Mock(),
        )

    def test_default_component_is_bridge_enabled(self) -> None:
        driver = HC14SerialDriver(on_bytes=lambda data: None)
        self.assertTrue(driver.bridge_envelope)
        self.assertEqual(driver.baudrate, 115200)
        self.assertEqual(driver.port, "/dev/ttyS4")
        self.assertFalse(driver.connected)
        self.assertFalse(driver.wait_connected(0.0))

    def test_hc15_and_legacy_imports_share_the_new_wiring(self) -> None:
        from components import HC15SerialDriver as exported_driver
        from components import DEFAULT_HC15_PORT as exported_port
        self.assertIs(HC14SerialDriver, HC15SerialDriver)
        self.assertIs(SerialCommunicationDriver, HC15SerialDriver)
        self.assertIs(exported_driver, HC15SerialDriver)
        self.assertEqual(DEFAULT_HC14_PORT, DEFAULT_HC15_PORT)
        self.assertEqual(exported_port, "/dev/ttyS4")
        # Explicit port overrides still support an operator-selected USB link.
        driver = HC15SerialDriver(on_bytes=lambda data: None, port="/dev/serial/by-path/operator-selected")
        self.assertEqual(driver.port, "/dev/serial/by-path/operator-selected")

    def test_callback_is_required(self) -> None:
        with self.assertRaises(TypeError):
            HC14SerialDriver(on_bytes=None)  # type: ignore[arg-type]

    def test_open_serial_acquires_process_exclusive_lock(self) -> None:
        fake_fcntl = SimpleNamespace(
            LOCK_EX=1,
            LOCK_NB=2,
            flock=mock.Mock(),
            ioctl=mock.Mock(),
        )
        fake_termios = self._fake_termios()
        driver = HC14SerialDriver(on_bytes=lambda data: None)

        with mock.patch.multiple(
                "components.serial_communication.os",
                O_RDWR=2,
                O_NOCTTY=0,
                O_NONBLOCK=0,
                create=True,
        ), mock.patch("components.serial_communication.fcntl", fake_fcntl), \
                mock.patch("components.serial_communication.termios", fake_termios), \
                mock.patch("components.serial_communication.os.open", return_value=41), \
                mock.patch("components.serial_communication.os.close") as close:
            self.assertEqual(41, driver._open_serial())

        fake_fcntl.flock.assert_called_once_with(41, 3)
        fake_fcntl.ioctl.assert_called_once_with(41, fake_termios.TIOCMBIC, struct.pack("I", 6))
        fake_termios.tcgetattr.assert_called_once_with(41)
        close.assert_not_called()

    def test_native_uart_opens_without_modem_control_support(self) -> None:
        for error_number in (errno.ENOTTY, errno.EINVAL, errno.EOPNOTSUPP):
            with self.subTest(errno=error_number):
                fake_fcntl = SimpleNamespace(
                    LOCK_EX=1, LOCK_NB=2, flock=mock.Mock(),
                    ioctl=mock.Mock(side_effect=OSError(error_number, "unsupported")),
                )
                fake_termios = self._fake_termios()
                driver = HC15SerialDriver(on_bytes=lambda data: None)
                with mock.patch.multiple("components.serial_communication.os",
                                         O_NOCTTY=0, O_NONBLOCK=0, create=True), \
                     mock.patch("components.serial_communication.fcntl", fake_fcntl), \
                     mock.patch("components.serial_communication.termios", fake_termios), \
                     mock.patch("components.serial_communication.os.open", return_value=41) as opened, \
                     mock.patch("components.serial_communication.os.close") as close:
                    self.assertEqual(driver._open_serial(), 41)
                self.assertEqual(opened.call_args.args[0], "/dev/ttyS4")
                configured = fake_termios.tcsetattr.call_args.args[2]
                self.assertEqual(configured[2], 115200 | 2 | 4 | 8)
                self.assertEqual(configured[4:6], [115200, 115200])
                close.assert_not_called()

    def test_real_modem_control_io_error_closes_the_port(self) -> None:
        fake_fcntl = SimpleNamespace(
            LOCK_EX=1, LOCK_NB=2, flock=mock.Mock(),
            ioctl=mock.Mock(side_effect=OSError(errno.EIO, "I/O error")),
        )
        driver = HC15SerialDriver(on_bytes=lambda data: None)
        with mock.patch.multiple("components.serial_communication.os",
                                 O_NOCTTY=0, O_NONBLOCK=0, create=True), \
             mock.patch("components.serial_communication.fcntl", fake_fcntl), \
             mock.patch("components.serial_communication.termios", self._fake_termios()), \
             mock.patch("components.serial_communication.os.open", return_value=42), \
             mock.patch("components.serial_communication.os.close") as close:
            with self.assertRaises(OSError) as caught:
                driver._open_serial()
        self.assertEqual(caught.exception.errno, errno.EIO)
        close.assert_called_once_with(42)

    def test_simulator_accepts_new_and_legacy_opt_in_flags(self) -> None:
        from fleet_car_pose_simulator import build_parser
        parser = build_parser()
        self.assertFalse(parser.parse_args([]).connect_hc15)
        self.assertTrue(parser.parse_args(["--connect-hc15"]).connect_hc15)
        self.assertTrue(parser.parse_args(["--connect-hc14"]).connect_hc15)

    def test_open_serial_rejects_second_process_and_closes_fd(self) -> None:
        fake_fcntl = SimpleNamespace(
            LOCK_EX=1,
            LOCK_NB=2,
            flock=mock.Mock(side_effect=BlockingIOError("held")),
            ioctl=mock.Mock(),
        )
        fake_termios = self._fake_termios()
        driver = HC14SerialDriver(on_bytes=lambda data: None)

        with mock.patch.multiple(
                "components.serial_communication.os",
                O_RDWR=2,
                O_NOCTTY=0,
                O_NONBLOCK=0,
                create=True,
        ), mock.patch("components.serial_communication.fcntl", fake_fcntl), \
                mock.patch("components.serial_communication.termios", fake_termios), \
                mock.patch("components.serial_communication.os.open", return_value=42), \
                mock.patch("components.serial_communication.os.close") as close:
            with self.assertRaisesRegex(SerialDriverError, "already in use"):
                driver._open_serial()

        close.assert_called_once_with(42)
        fake_termios.tcgetattr.assert_not_called()


if __name__ == "__main__":
    unittest.main()
