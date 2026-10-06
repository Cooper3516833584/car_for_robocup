"""Two-host HC15 diagnostic. No AT or actuator commands; password is not stored."""
import argparse
import getpass
import json
from pathlib import Path
import shlex
import uuid

import paramiko

AGENT = r'''
import errno, fcntl, json, os, select, struct, subprocess, sys, termios, time, zlib
port, role, run = sys.argv[1:4]
count, baud = map(int, sys.argv[4:6])
peer = 'C2G' if role == 'G2C' else 'G2C'
def packet(direction, index):
    body = ('HC15TEST|%s|%s|%d|0123456789abcdefghij' % (run, direction, index)).encode()
    return body + b'|' + ('%08X' % zlib.crc32(body)).encode() + b'\n'
owners = subprocess.run(['fuser', port], capture_output=True, text=True)
if owners.stdout.strip():
    raise RuntimeError('port already owned: ' + owners.stdout.strip())
fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
saved = None
rx, tx, unexpected, prefixes = [], [], [], []
buf = b''
try:
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    saved = termios.tcgetattr(fd)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, 'B' + str(baud))
    attrs[0], attrs[1] = termios.IGNPAR, 0
    attrs[2], attrs[3] = speed | termios.CS8 | termios.CREAD | termios.CLOCAL, 0
    attrs[4], attrs[5] = speed, speed
    attrs[6][termios.VMIN], attrs[6][termios.VTIME] = 0, 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    try:
        fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack('I', termios.TIOCM_DTR | termios.TIOCM_RTS))
    except OSError as exc:
        if exc.errno not in (errno.ENOTTY, errno.EINVAL, errno.EOPNOTSUPP):
            raise
    print(json.dumps({'event': 'ready', 'port': port, 'baud': baud}), flush=True)
    start = time.monotonic()
    next_tx = start + (2 if role == 'G2C' else 16)
    expected = {packet(peer, i): i for i in range(count)}
    marker = ('HC15TEST|%s|%s|' % (run, peer)).encode()
    while time.monotonic() - start < 36:
        now = time.monotonic()
        if len(tx) < count and now >= next_tx:
            msg = packet(role, len(tx))
            pending = msg
            deadline = now + 1
            while pending:
                if time.monotonic() > deadline:
                    raise RuntimeError('serial write timeout')
                if select.select([], [fd], [], .1)[1]:
                    n = os.write(fd, pending)
                    if n <= 0:
                        raise RuntimeError('serial write failed')
                    pending = pending[n:]
            tx.append(msg.decode().strip())
            next_tx += 1
            print(json.dumps({'event': 'tx', 'index': len(tx) - 1, 'elapsed_s': round(now - start, 3)}), flush=True)
        if select.select([fd], [], [], .05)[0]:
            data = os.read(fd, 4096)
            if not data:
                raise RuntimeError('serial disconnected')
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                line += b'\n'
                pos = line.find(marker)
                candidate = line[pos:] if pos >= 0 else line
                if candidate in expected:
                    if pos > 0:
                        prefixes.append(line[:pos].hex())
                    rx.append(expected[candidate])
                    print(json.dumps({'event': 'rx', 'index': expected[candidate], 'elapsed_s': round(time.monotonic() - start, 3), 'packet': candidate.decode().strip()}), flush=True)
                else:
                    unexpected.append(line.hex())
            if len(buf) > 4096:
                unexpected.append(buf.hex())
                buf = b''
    passed = sorted(rx) == list(range(count)) and len(tx) == count
    print(json.dumps({'event': 'result', 'role': role, 'sent': len(tx), 'received': len(rx), 'rx_indices': rx, 'unexpected_hex': unexpected[:8], 'discarded_prefix_hex': prefixes, 'partial_hex': buf.hex(), 'passed': passed}), flush=True)
finally:
    if saved is not None:
        termios.tcsetattr(fd, termios.TCSANOW, saved)
    os.close(fd)
sys.exit(0 if passed else 1)
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--baud', choices=(9600, 115200), type=int, default=9600)
    parser.add_argument('--count', choices=range(1, 6), type=int, default=5)
    args = parser.parse_args()
    password = getpass.getpass('Ground SSH password (not stored): ')
    run = uuid.uuid4().hex[:12]
    clients, channels = {}, {}
    results, events = {}, {}
    try:
        for name, host, user in [('ground', '192.168.31.107', 'cooper'), ('car', '192.168.31.224', 'radxa')]:
            client = paramiko.SSHClient()
            client.load_system_host_keys()
            client.set_missing_host_key_policy(paramiko.RejectPolicy())
            clients[name] = client
            credentials = {'password': password} if name == 'ground' else {'key_filename': str(Path.home() / '.ssh' / 'id_ed25519_rock5a')}
            client.connect(host, username=user, look_for_keys=False, allow_agent=False, timeout=10, banner_timeout=10, auth_timeout=10, **credentials)
        password = credentials = None
        ports = {'ground': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0', 'car': '/dev/ttyS4'}
        for name, client in clients.items():
            role = 'G2C' if name == 'ground' else 'C2G'
            command = 'python3 -u - ' + ' '.join(shlex.quote(str(v)) for v in (ports[name], role, run, args.count, args.baud))
            stdin, stdout, stderr = client.exec_command(command, timeout=50)
            stdin.write(AGENT)
            stdin.flush()
            stdin.channel.shutdown_write()
            channels[name] = (stdout, stderr)
        for name, (stdout, stderr) in channels.items():
            lines = stdout.read().decode()
            error = stderr.read().decode()
            status = stdout.channel.recv_exit_status()
            print(name + ':\n' + lines, flush=True)
            if error:
                print(error, flush=True)
            events[name] = [json.loads(line) for line in lines.splitlines()]
            result = next((item for item in events[name] if item['event'] == 'result'), None)
            results[name] = {'exit_status': status, 'result': result, 'error': error}
        passed = all(item['exit_status'] == 0 and item['result'] and item['result']['passed'] for item in results.values())
        released = {}
        for name, client in clients.items():
            _, out, err = client.exec_command('fuser ' + shlex.quote(ports[name]), timeout=10)
            owner = out.read().decode().strip()
            status = out.channel.recv_exit_status()
            released[name] = {'owners': owner, 'exit_status': status}
        report = {'run_id': run, 'baud': args.baud, 'count_per_direction': args.count, 'passed': passed, 'results': results, 'events': events, 'ports_after_test': released}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print('PASS' if passed else 'FAIL', str(args.output), flush=True)
        return 0 if passed else 1
    finally:
        for client in clients.values():
            client.close()


if __name__ == '__main__':
    raise SystemExit(main())
