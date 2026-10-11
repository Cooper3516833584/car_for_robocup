"""Cross-repository handoff test: actual codecs/nodes, emulated UART2, no hardware."""
import argparse
import importlib.util
from pathlib import Path
import sys
import threading
from types import SimpleNamespace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone-sdk", type=Path,
        default=Path(__file__).resolve().parents[2]/"drone/python_sdk")
    args = parser.parse_args()
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"code"))
    from components.fleet_task_sender import FleetTaskSender
    from components.serial_communication import FCWirelessBridgeCodec
    from components.fleet_protocol import decode_command, unpack_frame
    from components.fleet_models import CommandId

    # Isolated name avoids the car's reserved fleet_bus package shadowing the drone.
    package = args.drone_sdk.resolve()/"fleet_bus"
    spec = importlib.util.spec_from_file_location("drone_link_test", package/"__init__.py",
                                                 submodule_search_locations=[str(package)])
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    from drone_link_test.rescue_task import RescueTaskReceiver, RescueFleetNode
    from drone_link_test.command_queue import AirCommandQueue
    from drone_link_test.models import AirFleetState
    from drone_link_test.hc14_transport import HC14BridgeCodec

    cases = 0
    for red in range(5):
        for blue in range(5-red):
            green = 4-red-blue
            counts = SimpleNamespace(red=red, blue=blue, green=green)
            for mode in ("normal", "lost_ack", "corrupt_request"):
                stop = threading.Event()
                receiver = RescueTaskReceiver(stop, allow_start=True)
                calls = {"task": 0, "start": 0}
                set_task, start = receiver.set_task, receiver.request_start
                def counted_task(*values):
                    calls["task"] += 1
                    return set_task(*values)
                def counted_start(*values):
                    calls["start"] += 1
                    return start(*values)
                receiver.set_task, receiver.request_start = counted_task, counted_start
                outbound_codec = HC14BridgeCodec()
                inbound_codec = FCWirelessBridgeCodec()
                seen = {}
                sent = []

                class Driver:
                    def write(self, raw):
                        sent.append(raw)
                        request = unpack_frame(raw)
                        key = (request.session, request.seq)
                        seen[key] = seen.get(key, 0)+1
                        wire = FCWirelessBridgeCodec.encode(raw)
                        # Flight firmware unwraps BB33; emulate fragmented UART reads.
                        chunks = []
                        for offset in range(0, len(wire), 3):
                            chunks.extend(outbound_codec.feed(wire[offset:offset+3]))
                        for inner in chunks:
                            if mode == "corrupt_request" and seen[key] == 1:
                                inner = bytearray(inner)
                                inner[-3] ^= 0x01
                            for frame in node.parser.feed(inner):
                                reply = node._handle(frame)
                                if reply is None or (mode == "lost_ack" and seen[key] == 1):
                                    continue
                                # FC firmware wraps drone send_to_wireless replies.
                                reply_wire = HC14BridgeCodec.encode(reply)
                                for offset in range(0, len(reply_wire), 2):
                                    for payload in inbound_codec.feed(reply_wire[offset:offset+2]):
                                        sender.feed_bytes(payload)

                driver = Driver()
                node = RescueFleetNode(driver, lambda: AirFleetState(0, 0), AirCommandQueue(),
                                       stop, receiver=receiver)
                sender = FleetTaskSender(driver, response_timeout_s=.003, retries=1, session=123)
                assert sender.send_task_and_start(counts, task_id=100+cases), (counts, mode)
                task = receiver.wait_for_start(.01)
                assert task.allocation() == dict(red=red, blue=blue, green=green)
                assert calls == {"task": 1, "start": 1}, (mode, calls)
                ids = [decode_command(unpack_frame(raw).payload).command_id for raw in sent]
                assert ids[-1] == CommandId.DRONE_START_DROP_TASK
                if mode != "normal":
                    assert sent[0] == sent[1] and sent[2] == sent[3]
                cases += 1
    print(f"PASS: {cases} cross-repository cases; all 15 allocations, ACK loss, CRC corruption, fragmented BB33; no hardware opened")


if __name__ == "__main__":
    main()
