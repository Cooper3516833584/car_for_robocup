from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from components.radar_driver import D500SerialDriver, RadarDriverError
from config.v2_loader import load_v2_config
from config.v2_runtime import RuntimeMode
from config.v2_factory import build_differential_drive
import robocup_runtime


class D500BaudrateTests(unittest.TestCase):
    def test_non_default_baudrate_controls_termios_input_and_output_speed(self) -> None:
        fake_termios = types.SimpleNamespace(
            CS8=8,
            CREAD=128,
            CLOCAL=2048,
            B115200=4098,
            VMIN=6,
            VTIME=5,
            TCSANOW=0,
            TCIFLUSH=0,
            tcgetattr=lambda _fd: [0, 0, 0, 0, 0, 0, {}],
            tcsetattr=lambda *_args: None,
            tcflush=lambda *_args: None,
        )
        fake_os = types.SimpleNamespace(
            O_RDONLY=0,
            O_NOCTTY=1,
            O_NONBLOCK=2,
            open=lambda *_args: 42,
            close=lambda *_args: None,
        )
        with (
            patch("components.radar_driver.termios", fake_termios),
            patch("components.radar_driver.fcntl", object()),
            patch("components.radar_driver.os", fake_os),
            patch.object(fake_termios, "tcsetattr", wraps=fake_termios.tcsetattr) as set_attrs,
        ):
            driver = D500SerialDriver(on_packet=lambda _packet: None, port="/dev/fake", baudrate=115200)
            driver._open()
            configured = set_attrs.call_args.args[2]
            self.assertEqual(configured[4], fake_termios.B115200)
            self.assertEqual(configured[5], fake_termios.B115200)
            driver.close()

    def test_unsupported_baudrate_fails_closed(self) -> None:
        fake_termios = types.SimpleNamespace()
        with (
            patch("components.radar_driver.termios", fake_termios),
            patch("components.radar_driver.fcntl", object()),
        ):
            driver = D500SerialDriver(on_packet=lambda _packet: None, baudrate=12345)
            with self.assertRaisesRegex(RadarDriverError, "unsupported D500 baudrate"):
                driver._open()

    def test_hardware_factory_passes_configured_baudrate(self) -> None:
        source = load_v2_config()
        config = replace(
            source,
            calibration=replace(source.calibration, geometry_measured=True, sensor_extrinsics_measured=True),
            competition_map=replace(source.competition_map, measured=True),
            footprint=replace(source.footprint, measured=True),
            d500_localization=replace(source.d500_localization, reference_measured=True),
            d500=replace(source.d500, baudrate=115200),
            t265=replace(source.t265, enabled=False),
        )
        with patch.object(
            robocup_runtime,
            "build_differential_drive",
            side_effect=lambda cfg, **kwargs: build_differential_drive(cfg, fake=True, clock=kwargs["clock"]),
        ):
            runtime = robocup_runtime.build_runtime(config, RuntimeMode.HARDWARE_MISSION, clock=lambda: 1.0)
        self.assertEqual(runtime.d500_source.serial.baudrate, 115200)
        runtime.close()


if __name__ == "__main__":
    unittest.main()
