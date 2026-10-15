"""Receive-only HC-15 framing probe; stop competition-boot before running.

Never transmits, handles commands or opens actuators. The transport's exclusive
port lock prevents this probe from stealing bytes from a running listener.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))
from components.fleet_models import NodeId
from components.fleet_protocol import FrameParser
from components.serial_communication import FCWirelessBridgeCodec, HC15SerialDriver


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error("--seconds must be between 1 and 60")
    bridge = FCWirelessBridgeCodec()
    wrapped = FrameParser(local_node=NodeId.CAR)
    direct = FrameParser(local_node=NodeId.CAR)
    received = 0

    def emit(event, **values):
        print(json.dumps(dict(event=event, **values)), flush=True)

    def frames(mode, values):
        for frame in values:
            emit("frame", framing=mode, src=int(frame.src), dst=int(frame.dst),
                 kind=int(frame.kind), seq=frame.seq, payload_hex=frame.payload.hex())

    def receive(data):
        nonlocal received
        received += len(data)
        emit("rx", bytes=len(data), hex=data[:256].hex())
        frames("direct", direct.feed(data))
        for payload in bridge.feed(data):
            frames("BB33", wrapped.feed(payload))

    driver = HC15SerialDriver(on_bytes=receive, bridge_envelope=False,
        on_connected=lambda: emit("connected", port="/dev/ttyS4", baudrate=115200),
        on_disconnected=lambda error: emit("disconnected", error=str(error)),
        on_callback_error=lambda error: emit("callback_error", error=str(error)))
    try:
        driver.start()
        if not driver.wait_connected(3):
            raise RuntimeError(f"radio unavailable: {driver.last_error}")
        emit("listening", seconds=args.seconds, transmit=False, mission=False)
        time.sleep(args.seconds)
    finally:
        driver.close()
        emit("summary", bytes=received, bridge=asdict(bridge.stats),
             wrapped=asdict(wrapped.stats), direct=asdict(direct.stats))


if __name__ == "__main__":
    main()
