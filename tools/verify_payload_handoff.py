"""Hardware-free car/drone v3 handoff and single-yellow release acceptance.

Run from the car checkout: python tools/verify_payload_handoff.py --drone-repo ../drone
Uses real FleetBus codecs, sender, receiver and extracted task release logic;
all transport, FC, navigation and servo outputs are fakes.
"""
import argparse
import ast
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from unittest.mock import Mock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drone-repo', type=Path, required=True)
    args = parser.parse_args()
    sdk = args.drone_repo.resolve() / 'python_sdk'
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'code'))
    sys.path.insert(0, str(sdk))
    from competition_task import split_task_allocation
    from components.task_board_reader import TaskCounts
    from components import drop_task_protocol as car_task
    from components import fleet_protocol as car_wire
    from components.fleet_models import CommandId, AckStatus
    from components.fleet_task_sender import FleetTaskSender
    from fleet_bus import drop_task_protocol as drone_task
    from fleet_bus import protocol as drone_wire
    from fleet_bus.models import AirFleetState, NodeId
    from fleet_bus.command_queue import AirCommandQueue
    from fleet_bus.rescue_task import RescueFleetNode, RescueTaskReceiver
    import payload_servos

    # Extract only constants, quota ledger and release methods. Never import the
    # flight entry or FlightController, even when hardware dependencies exist.
    source = sdk / 'rescue_drop_2026.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    constants = {'FREE_DROP_COUNT', 'MANDATORY_DROP_COUNT', 'TOTAL_DROP_COUNT',
                 'STM32_CHANNEL_BY_SLOT', 'FREE_COLORS', 'MANDATORY_COLOR'}
    nodes = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id in constants for t in node.targets):
            nodes.append(node)
        elif isinstance(node, ast.ClassDef) and node.name == 'DropLedger':
            nodes.append(node)
        elif isinstance(node, ast.ClassDef) and node.name == 'Mission':
            node.body = [m for m in node.body if isinstance(m, ast.FunctionDef)
                         and m.name in ('_drop', '_skip_unconfigured_drop')]
            nodes.append(node)
    future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
    namespace = dict(logger=Mock(), slot_for_drop_attempt=payload_servos.slot_for_drop_attempt,
                     drop_plan_ready=payload_servos.drop_plan_ready)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[future]+nodes, type_ignores=[])),
                 str(source), 'exec'), namespace)
    assert namespace['TOTAL_DROP_COUNT'] == payload_servos.PAYLOAD_COUNT == 1
    assert car_task.TASK_VERSION == drone_task.TASK_VERSION == 3
    assert car_wire.VERSION == drone_wire.VERSION == 1

    simulations = 0
    for board in ((2,1,1), (1,2,1), (1,1,2)):
        for task_id in (1, 42, 0xffffffff):
            car, drone = split_task_allocation(TaskCounts(*board))
            assert tuple(car[c] for c in ('red','blue','green')) == board
            assert sum(car.values())+1 == 5 and drone.as_dict() == dict(red=0,blue=0,green=0)
            stop = threading.Event()
            payload = Mock(connected=True)
            payload.get_states.return_value = {c: False for c in range(1,5)}
            fc = SimpleNamespace(connected=True, state=SimpleNamespace(
                is_fresh=lambda _: True, unlock=SimpleNamespace(value=False)))
            navigation = Mock(running=True)
            receiver = RescueTaskReceiver(stop, allow_start=True, payload=payload,
                fc=fc, navigation=navigation, validate_plan=payload_servos.validate_drop_plan,
                preflight_result=dict(needs_calibration=False, pose={'stamp_ns': 123}))
            node = RescueFleetNode(Mock(), lambda: AirFleetState(0,0), AirCommandQueue(),
                                   stop, receiver=receiver)
            frames = drone_wire.FrameParser(local_node=NodeId.DRONE)
            exchanges = []
            def write(raw):
                for byte in raw:  # Exercise actual fragmented framing in both directions.
                    for frame in frames.feed(bytes([byte])):
                        command = drone_wire.decode_command(frame.payload)
                        ack_raw = node._handle(frame)
                        assert ack_raw is not None
                        ack = drone_wire.decode_ack(drone_wire.unpack_frame(ack_raw).payload)
                        exchanges.append((int(command.command_id), int(ack.status)))
                        for value in ack_raw:
                            sender.feed_bytes(bytes([value]))
            sender = FleetTaskSender(SimpleNamespace(write=write), session=123,
                                     response_timeout_s=.01, retries=0)
            assert sender.close_drone_payload(7)
            assert sender.begin_drone_remap(7)
            assert sender.wait_drone_remap(7, timeout_s=.1, poll_s=.01)
            assert sender.send_task_and_start(drone, task_id=task_id)
            assert exchanges[-2:] == [(int(CommandId.DRONE_SET_DROP_TASK), int(AckStatus.COMPLETED)),
                                      (int(CommandId.DRONE_START_DROP_TASK), int(AckStatus.ACCEPTED))]
            saved = receiver.wait_for_start(.01)
            assert saved.allocation() == drone.as_dict()
            assert drone_task.decode_drop_task(car_task.encode_drop_task(
                car_task.DropTask(task_id,0,0,0))) == saved
            assert car_task.decode_drop_start(drone_task.encode_drop_start(task_id)) == task_id
            mission = namespace['Mission']()
            mission.ledger = namespace['DropLedger'](saved.allocation())
            mission.payload, mission._check = payload, Mock()
            mission._completed_free_colors = set()
            for color in ('red','blue','green'):
                assert not mission.ledger.has_quota(color)
            mission._drop('yellow', 'simulated-yellow', True)
            payload.open_servo.assert_called_once_with(3)
            assert mission.ledger.sent == dict(red=0,yellow=1,blue=0,green=0)
            try:
                mission._drop('yellow', 'simulated-yellow', True)
            except (ValueError, RuntimeError):
                pass
            else:
                raise AssertionError('second release was accepted')
            payload.open_servo.assert_called_once_with(3)
            payload.close_all.assert_not_called()  # Operator preflight CLOSE only checks states.
            payload.open_all.assert_not_called()
            simulations += 1
    for protocol in (car_task, drone_task):
        for decode, body in ((protocol.decode_drop_task, b'\x02\x01\x00\x00\x00\x00\x00\x00'),
                             (protocol.decode_drop_start, b'\x02\x01\x00\x00\x00')):
            try:
                decode(body)
            except ValueError:
                pass
            else:
                raise AssertionError('v2 packet was accepted')
    print('%d simulations passed: three boards x three IDs; v3 zero RGB, one yellow, V1 framing/CLOSE/REMAP' % simulations)


if __name__ == '__main__':
    main()
