"""Run-button launchers and local commands to the existing competition listener."""
import json
import signal
import shutil
import subprocess
import time
import os
from pathlib import Path
import socket
import threading

CONTROL_SOCKET = Path('/run/robocup-car/control.sock')


def request_local(action, path=CONTROL_SOCKET, *, options=None):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(5)
        connection.connect(str(path))
        request = {'action': action}
        if options is not None:
            request['options'] = options
        connection.sendall(json.dumps(request).encode() + b'\n')
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
                    if 'options' in request:
                        result = self.handler(request['action'], options=request['options'])
                    else:
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


def run_local(options=None):
    log = None
    started = False
    previous_handler = signal.getsignal(signal.SIGTERM)
    def interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        if shutil.which('journalctl'):
            log = subprocess.Popen(['journalctl', '-u', 'competition-boot.service',
                                    '--since', 'now', '-f', '-o', 'cat', '--no-pager'])
        result = request_local('start', options=options) if options is not None else request_local('start')
        if not result.get('ok'):
            print('启动未接受：' + json.dumps(result, ensure_ascii=False), flush=True)
            return 1
        started = True
        print('本机全流程已启动：关闭物资后立即运行；Ctrl+C 停止。', flush=True)
        while request_local('status')['running']:
            time.sleep(.5)
        print('本轮任务已结束，结果见上方任务日志。', flush=True)
        return 0
    except KeyboardInterrupt:
        if started:
            request_local('stop')
            print('已请求停车并结束本轮任务。', flush=True)
        return 130
    except (OSError, ValueError, KeyError) as exc:
        print('本机任务连接失败：%s；请检查 competition-boot.service。' % exc, flush=True)
        return 2
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
        if log is not None:
            log.terminate()
            log.wait(timeout=3)
