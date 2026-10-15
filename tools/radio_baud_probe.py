"""Passively compare UART receive baud rates; no writes or command handling.

Stop competition-boot.service first. Requires pyserial; exclusive port locking
prevents concurrent consumption. Restore the service after this probe exits.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code'))
from components.fleet_protocol import FrameParser
from components.serial_communication import FCWirelessBridgeCodec

RATES = (4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds-per-rate', type=float, default=2)
    args = parser.parse_args()
    if not 1 <= args.seconds_per_rate <= 5:
        parser.error('--seconds-per-rate must be between 1 and 5')
    import serial
    port = serial.Serial(port=None, baudrate=115200, timeout=.05, exclusive=True)
    port.port = '/dev/ttyS4'
    port.dtr = port.rts = False
    try:
        port.open()
        for rate in RATES:
            bridge, wrapped, direct = FCWirelessBridgeCodec(), FrameParser(), FrameParser()
            data = bytearray()
            found = []
            port.baudrate = rate
            port.reset_input_buffer()
            deadline = time.monotonic() + args.seconds_per_rate
            print(json.dumps(dict(event='rate_start', baudrate=rate)), flush=True)
            while time.monotonic() < deadline:
                chunk = port.read(port.in_waiting or 1)
                data.extend(chunk)
                for framing, frames in [('direct', direct.feed(chunk))] + [
                        ('BB33', wrapped.feed(payload)) for payload in bridge.feed(chunk)]:
                    found.extend(dict(framing=framing, src=int(frame.src), dst=int(frame.dst),
                                      kind=int(frame.kind), payload_hex=frame.payload.hex())
                                 for frame in frames)
            print(json.dumps(dict(event='rate_result', baudrate=rate, bytes=len(data),
                                  hex=data[:256].hex(), frames=found,
                                  bridge=asdict(bridge.stats), direct=asdict(direct.stats))), flush=True)
    finally:
        if port.is_open:
            try:
                port.baudrate = 115200
            finally:
                port.close()


if __name__ == '__main__':
    main()
