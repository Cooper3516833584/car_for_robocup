"""VS Code Remote SSH: run this file to start the fixed red/blue/green 1/2/1 mission.

No task-board OCR. Uses the existing boot listener, real drive and five payload
servos; Ctrl+C requests STOP. No command-line arguments are required.
"""
# 快速调整区：距离单位米，顺序为左边、底边、右边、顶边。
WALL_RIGHT_DISTANCE_M = (1.530, 1.825, 1.620, 2.260)
STANDARD_DROP_M = 0.43       # 普通投放前进距离（槽位 0..3）
RIGHT_MOUNT_DROP_M = 0.32    # 右侧投放前进距离（黄色，槽位 4）
FINISH_FRONT_DISTANCE_M = 1.10
TASK_COUNTS = (1, 2, 1)     # 红、蓝、绿；固定规则，跳过任务板

import argparse
import json
from components.local_mission_control import request_local, run_local as _run_local
from components.competition_options import CompetitionOptions


def run_local():
    return _run_local(CompetitionOptions(
        wall_right_distance_m=WALL_RIGHT_DISTANCE_M,
        standard_drop_m=STANDARD_DROP_M, right_mount_drop_m=RIGHT_MOUNT_DROP_M,
        finish_front_m=FINISH_FRONT_DISTANCE_M, task_counts=TASK_COUNTS).as_dict())


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
