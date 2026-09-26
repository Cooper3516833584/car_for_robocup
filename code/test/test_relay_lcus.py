"""Pure-software tests for the LCUS relay driver; no serial port is opened.

A pyserial-like double is injected through ``LCUSRelay(serial_factory=...)`` so
frame encoding, read-back verification, retry behavior, timeouts and the
fail-closed paths run on any machine, including Windows without a relay board.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# The driver warns on every deliberately failed read-back; keep suite output clean.
logging.getLogger("components.relay_lcus").setLevel(logging.CRITICAL)

from components.relay_lcus import (
    DEFAULT_BAUDRATE,
    FakeLCUSRelay,
    LCUSRelay,
    RELAY_PORT_ENV,
    build_channel_command,
    format_port_list,
    format_states,
    list_serial_ports,
    parse_status_response,
    resolve_relay_settings,
)


def status_text(states: dict[int, bool], channels: int) -> bytes:
    """Build the board's fixed-width ``CHn: ON /OFF`` reply for *channels* lines."""

    return "".join(
        f"CH{channel}: {'ON ' if states.get(channel) else 'OFF'}\r\n"
        for channel in range(1, channels + 1)
    ).encode("ascii")


class FakeSerial:
    """Minimal pyserial-like double: one queued FF reply per query."""

    def __init__(self, responses=()) -> None:
        self.responses = [bytes(response) for response in responses]
        self.writes = bytearray()
        self.dtr = None
        self.rts = None
        self.is_open = False
        self.open_count = 0
        self.close_count = 0
        self._buffer = b""

    # -- configuration surface used by LCUSRelay.open()
    def open(self) -> None:
        self.is_open = True
        self.open_count += 1

    def close(self) -> None:
        self.is_open = False
        self.close_count += 1

    def setDTR(self, value) -> None:
        self.dtr = value

    def setRTS(self, value) -> None:
        self.rts = value

    def reset_input_buffer(self) -> None:
        self._buffer = b""

    # -- read/write surface used by LCUSRelay
    @property
    def in_waiting(self) -> int:
        return len(self._buffer)

    def read(self, size: int = 1) -> bytes:
        chunk = self._buffer[:size]
        self._buffer = self._buffer[size:]
        return chunk

    def write(self, data) -> int:
        self.writes.extend(data)
        # The board answers a FF query; every other write is fire-and-forget.
        if bytes(data) == b"\xff" and self.responses:
            self._buffer = self.responses.pop(0)
        return len(data)

    def flush(self) -> None:
        return None

    def command_frames(self) -> list[bytes]:
        """Split the write stream into 4-byte control frames and 1-byte queries."""

        frames: list[bytes] = []
        index = 0
        while index < len(self.writes):
            if self.writes[index] == 0xFF:
                frames.append(b"\xff")
                index += 1
            else:
                frames.append(bytes(self.writes[index : index + 4]))
                index += 4
        return frames


def make_relay(serial: FakeSerial, *, channel_count: int = 8, query_timeout: float = 0.05) -> LCUSRelay:
    return LCUSRelay(
        port="COM_TEST",
        channel_count=channel_count,
        query_timeout=query_timeout,
        serial_factory=lambda: serial,
    )


class ChannelCommandTests(unittest.TestCase):
    ON_FRAMES = {
        1: "A00101A2", 2: "A00201A3", 3: "A00301A4", 4: "A00401A5",
        5: "A00501A6", 6: "A00601A7", 7: "A00701A8", 8: "A00801A9",
    }
    OFF_FRAMES = {
        1: "A00100A1", 2: "A00200A2", 3: "A00300A3", 4: "A00400A4",
        5: "A00500A5", 6: "A00600A6", 7: "A00700A7", 8: "A00800A8",
    }

    def test_command_table_matches_the_datasheet(self) -> None:
        for channel, expected in self.ON_FRAMES.items():
            self.assertEqual(build_channel_command(channel, True).hex().upper(), expected)
        for channel, expected in self.OFF_FRAMES.items():
            self.assertEqual(build_channel_command(channel, False).hex().upper(), expected)

    def test_checksum_is_the_low_byte_of_the_prefix_channel_and_state_sum(self) -> None:
        command = build_channel_command(7, True)
        self.assertEqual(command[3], (command[0] + command[1] + command[2]) & 0xFF)

    def test_invalid_channel_is_rejected(self) -> None:
        for invalid in (0, 9, -1, 1.0, "1", True):
            with self.assertRaises(ValueError):
                build_channel_command(invalid, True)

    def test_invalid_channel_count_is_rejected(self) -> None:
        for invalid in (0, 9, True, 2.5):
            with self.assertRaises(ValueError):
                LCUSRelay(port="COM_TEST", channel_count=invalid)


class StatusParsingTests(unittest.TestCase):
    def test_parses_a_complete_four_channel_reply(self) -> None:
        payload = status_text({1: True, 2: False, 3: True, 4: False}, 4)
        self.assertEqual(
            parse_status_response(payload, 4),
            {1: True, 2: False, 3: True, 4: False},
        )
        self.assertEqual(len(payload), 40)

    def test_tolerates_fragments_noise_and_spacing(self) -> None:
        self.assertEqual(parse_status_response(b"CH1: O", 4), {})
        self.assertEqual(parse_status_response(b"\x00\xffCH1: ON \r\n", 4), {1: True})
        self.assertEqual(parse_status_response("ch 3 : on\r\n", 4), {3: True})

    def test_ignores_channels_above_the_configured_count(self) -> None:
        self.assertEqual(parse_status_response(b"CH5: ON \r\n", 4), {})

    def test_format_states_is_sorted_and_explicit(self) -> None:
        self.assertEqual(format_states({3: True, 1: False}), "CH1=OFF CH3=ON")
        self.assertEqual(format_states({}), "(空)")


class PortResolutionTests(unittest.TestCase):
    def test_explicit_port_wins_over_the_environment(self) -> None:
        with patch.dict(os.environ, {RELAY_PORT_ENV: "/dev/from-env"}):
            self.assertEqual(resolve_relay_settings("COM7", None), ("COM7", DEFAULT_BAUDRATE))

    def test_environment_port_is_used_when_no_argument_is_given(self) -> None:
        with patch.dict(os.environ, {RELAY_PORT_ENV: "/dev/from-env"}):
            self.assertEqual(resolve_relay_settings(None, 19200), ("/dev/from-env", 19200))

    def test_missing_port_is_rejected_with_the_environment_variable_name(self) -> None:
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(ValueError, RELAY_PORT_ENV):
                resolve_relay_settings(None, None)

    def test_invalid_baudrate_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            resolve_relay_settings("COM7", "fast")
        with self.assertRaises(ValueError):
            resolve_relay_settings("COM7", 0)


class SerialPortListingTests(unittest.TestCase):
    def test_listing_degrades_without_pyserial(self) -> None:
        unavailable = {"serial": None, "serial.tools": None, "serial.tools.list_ports": None}
        with patch.dict(sys.modules, unavailable):
            self.assertEqual(list_serial_ports(), [])
            self.assertEqual(format_port_list(), "未发现串口")


class OpenCloseTests(unittest.TestCase):
    def test_open_pulls_dtr_rts_low_and_is_idempotent(self) -> None:
        serial = FakeSerial()
        relay = make_relay(serial)
        self.assertFalse(relay.connected)
        relay.open()
        self.assertTrue(relay.connected)
        self.assertTrue(serial.is_open)
        self.assertIs(serial.dtr, False)
        self.assertIs(serial.rts, False)
        relay.open()
        self.assertEqual(serial.open_count, 1)
        self.assertEqual(len(serial.writes), 0, "opening the port must not touch a contact")
        relay.close()
        relay.close()
        self.assertEqual(serial.close_count, 1)
        self.assertFalse(relay.connected)

    def test_serial_factory_is_lazy_until_open(self) -> None:
        created: list[FakeSerial] = []

        def factory() -> FakeSerial:
            serial = FakeSerial()
            created.append(serial)
            return serial

        relay = LCUSRelay(port="COM_TEST", serial_factory=factory)
        self.assertEqual(created, [])
        relay.open()
        self.assertEqual(len(created), 1)

    def test_operations_require_an_open_port(self) -> None:
        relay = make_relay(FakeSerial())
        with self.assertRaisesRegex(RuntimeError, "未打开"):
            relay.query_status()
        with self.assertRaisesRegex(RuntimeError, "未打开"):
            relay.set_channel(1, True)
        with self.assertRaisesRegex(RuntimeError, "未打开"):
            relay.get_channel_state(1)

    def test_missing_pyserial_fails_as_a_runtime_error(self) -> None:
        with patch.dict(sys.modules, {"serial": None}):
            relay = LCUSRelay(port="COM_TEST")
            with self.assertRaisesRegex(RuntimeError, "pyserial"):
                relay.open()


class ControlTests(unittest.TestCase):
    def test_set_channel_without_verify_sends_exactly_one_frame(self) -> None:
        serial = FakeSerial()
        relay = make_relay(serial, channel_count=4)
        relay.open()
        self.assertTrue(relay.set_channel(2, True))
        self.assertEqual(serial.command_frames(), [bytes.fromhex("A00201A3")])
        relay.close()

    def test_verify_confirms_after_one_idempotent_resend(self) -> None:
        # The board reports the old state for roughly 50 ms after a command, so
        # the first comparison fails and the identical resend is confirmed.
        serial = FakeSerial([
            status_text({1: False}, 4),
            status_text({1: True, 2: True}, 4),
        ])
        relay = make_relay(serial, channel_count=4)
        relay.open()
        self.assertTrue(relay.set_channel(1, True, verify=True))
        self.assertEqual(
            serial.command_frames(),
            [bytes.fromhex("A00101A2"), b"\xff", bytes.fromhex("A00101A2"), b"\xff"],
        )
        relay.close()

    def test_verify_reports_failure_after_all_retries(self) -> None:
        serial = FakeSerial([status_text({1: False}, 4)] * 3)
        relay = make_relay(serial, channel_count=4)
        relay.open()
        self.assertFalse(relay.set_channel(1, True, verify=True, retries=2))
        self.assertEqual(
            serial.command_frames().count(bytes.fromhex("A00101A2")),
            3,
        )
        relay.close()

    def test_verify_fails_closed_when_the_board_stays_silent(self) -> None:
        serial = FakeSerial([])
        relay = make_relay(serial, channel_count=1)
        relay.open()
        self.assertFalse(relay.set_channel(1, True, verify=True, retries=0))
        relay.close()

    def test_all_off_drives_every_channel_low(self) -> None:
        serial = FakeSerial()
        relay = make_relay(serial, channel_count=4)
        relay.open()
        self.assertTrue(relay.all_off())
        self.assertEqual(
            [frame.hex().upper() for frame in serial.command_frames()],
            ["A00100A1", "A00200A2", "A00300A3", "A00400A4"],
        )
        relay.close()

    def test_set_all_keeps_going_after_a_failed_channel(self) -> None:
        # Channel 2 never reports ON; the remaining channels are still attempted
        # and the failed one is reported through the return value, not rolled back.
        serial = FakeSerial([
            status_text({1: True, 2: False, 3: False, 4: False}, 4),
            status_text({1: True, 2: False, 3: False, 4: False}, 4),
            status_text({1: True, 2: False, 3: True, 4: False}, 4),
            status_text({1: True, 2: False, 3: True, 4: True}, 4),
        ])
        relay = make_relay(serial, channel_count=4)
        relay.open()
        self.assertFalse(relay.set_all(True, verify=True, retries=0))
        self.assertEqual(
            [frame.hex().upper() for frame in serial.command_frames() if len(frame) == 4],
            ["A00101A2", "A00201A3", "A00301A4", "A00401A5"],
        )
        relay.close()


class QueryTests(unittest.TestCase):
    def test_query_status_reads_all_channels(self) -> None:
        serial = FakeSerial([status_text({1: True, 2: False, 3: True, 4: False}, 4)])
        relay = make_relay(serial, channel_count=4)
        relay.open()
        self.assertEqual(relay.query_status(), {1: True, 2: False, 3: True, 4: False})
        self.assertEqual(serial.command_frames(), [b"\xff"])
        relay.close()

    def test_query_status_returns_the_partial_snapshot_it_received(self) -> None:
        serial = FakeSerial([status_text({1: True, 2: False, 3: False}, 3)])
        relay = make_relay(serial, channel_count=4)
        relay.open()
        self.assertEqual(relay.query_status(), {1: True, 2: False, 3: False})
        relay.close()

    def test_query_status_returns_none_without_a_reply(self) -> None:
        relay = make_relay(FakeSerial([]), channel_count=4)
        relay.open()
        self.assertIsNone(relay.query_status())
        relay.close()

    def test_get_channel_state_returns_none_for_a_missing_channel(self) -> None:
        reply = b"CH1: ON \r\n"
        serial = FakeSerial([reply, reply])
        relay = make_relay(serial, channel_count=2)
        relay.open()
        self.assertIsNone(relay.get_channel_state(2))
        self.assertTrue(relay.get_channel_state(1))
        relay.close()

    def test_detect_channel_count_identifies_a_four_channel_board(self) -> None:
        serial = FakeSerial([status_text({}, 4)])
        relay = make_relay(serial, channel_count=8)
        relay.open()
        self.assertEqual(relay.detect_channel_count(), 4)
        self.assertEqual(relay.channel_count, 8, "detection must not change the configuration")
        relay.close()

    def test_detect_channel_count_returns_none_without_a_reply(self) -> None:
        relay = make_relay(FakeSerial([]), channel_count=8)
        relay.open()
        self.assertIsNone(relay.detect_channel_count())
        relay.close()


class FakeRelayTests(unittest.TestCase):
    def test_fake_relay_mirrors_the_driver_surface_without_a_port(self) -> None:
        relay = FakeLCUSRelay(channel_count=4)
        self.assertEqual(relay.port, "fake")
        self.assertFalse(relay.connected)
        with self.assertRaisesRegex(RuntimeError, "未打开"):
            relay.query_status()

        relay.open()
        self.assertTrue(relay.set_channel(1, True, verify=True))
        self.assertEqual(relay.query_status(), {1: True, 2: False, 3: False, 4: False})
        self.assertTrue(relay.all_off(verify=True))
        self.assertEqual(relay.off_requests, 1)
        self.assertEqual(relay.get_channel_state(1), False)
        self.assertEqual(relay.detect_channel_count(), 4)
        self.assertEqual(relay.commands[0], build_channel_command(1, True, 4))
        self.assertEqual(relay.commands[-1], build_channel_command(4, False, 4))
        with self.assertRaises(ValueError):
            relay.set_channel(5, True)
        relay.close()
        self.assertEqual(relay.close_count, 1)
        self.assertFalse(relay.connected)


if __name__ == "__main__":
    unittest.main()
