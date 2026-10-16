"""Run this file without arguments in VS Code Remote SSH to start the full mission.

The boot service uses --listen; local and radio RUN share that single mission
owner, including countdown, SLAM, OCR, real drive, dual SG90 and cleanup.
"""
import argparse
import json
import shutil
import signal
import subprocess
import time

from components.local_mission_control import request_local


def run_local():
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
        result = request_local('start')
        if not result.get('ok'):
            print('启动未接受：' + json.dumps(result, ensure_ascii=False), flush=True)
            return 1
        started = True
        print('本机全流程已启动：关闭物资后计时 20 秒，重建图后蜂鸣 5 秒；Ctrl+C 停止。', flush=True)
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
        if started:
            try:
                request_local('stop')
            except (OSError, ValueError):
                pass
        print('本机任务连接失败：%s；请检查 competition-boot.service。' % exc, flush=True)
        return 2
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
        if log is not None:
            log.terminate()
            log.wait(timeout=3)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--listen', action='store_true', help='boot-service listener; does not auto-start a mission')
    mode.add_argument('--stop', action='store_true', help='stop the current car-only mission')
    mode.add_argument('--status', action='store_true', help='read local listener status without starting')
    args = parser.parse_args(argv)
    if args.listen:
        from competition_boot import main as boot_main
        return boot_main(car_only_test=True, local_control=True)
    if args.stop or args.status:
        try:
            result = request_local('stop' if args.stop else 'status')
            print(json.dumps(result, ensure_ascii=False))
            return 0 if result.get('ok') else 1
        except (OSError, ValueError) as exc:
            print('本机任务连接失败：%s' % exc)
            return 2
    return run_local()


if __name__ == "__main__":
    raise SystemExit(main())
