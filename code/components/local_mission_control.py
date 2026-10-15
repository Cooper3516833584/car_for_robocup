"""Local operator commands to the existing car-only mission listener."""
import json
import os
from pathlib import Path
import socket
import threading

CONTROL_SOCKET = Path('/run/robocup-car/control.sock')


def request_local(action, path=CONTROL_SOCKET):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(5)
        connection.connect(str(path))
        connection.sendall(json.dumps({'action': action}).encode() + b'\n')
        with connection.makefile('rb') as stream:
            return json.loads(stream.readline(4096))


class LocalMissionControl:
    def __init__(self, handler, path=CONTROL_SOCKET):
        self.handler = handler
        self.path = Path(path)
        self._socket = None
        self._thread = None
        self._stop = threading.Event()

    def bind(self):
        # Never unlink an existing listener's socket or open a second radio owner.
        connection = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            connection.bind(str(self.path))
            os.chmod(self.path, 0o600)
            connection.listen(4)
            connection.settimeout(.2)
        except Exception:
            connection.close()
            raise
        self._socket = connection

    def start(self):
        self._thread = threading.Thread(target=self._serve, name='local-mission-control', daemon=True)
        self._thread.start()

    def _serve(self):
        while not self._stop.is_set():
            try:
                connection, _ = self._socket.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                raise
            with connection:
                connection.settimeout(2)
                try:
                    with connection.makefile('rb') as stream:
                        request = json.loads(stream.readline(513))
                    result = self.handler(request['action'])
                except Exception as exc:
                    result = {'ok': False, 'error': str(exc)}
                try:
                    connection.sendall(json.dumps(result).encode() + b'\n')
                except OSError:
                    pass

    def close(self):
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
            if self._thread is not None:
                self._thread.join(timeout=3)
            self.path.unlink(missing_ok=True)
            self._socket = None
