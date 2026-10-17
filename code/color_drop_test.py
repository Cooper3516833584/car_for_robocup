"""直接运行：直行找四色目标，各投一次，返回巡路位置后继续；无导航。

摄像头保持 +90 度侧视。前置：左转90/前进43cm/投/倒回/右转90；
黄色右侧：左转90/前进32cm/左转90/投/右转90/倒回/右转90。
T265 只测量动作的距离与角度；不用地图、导航或比赛任务。
运行：cd /home/radxa/car && python3 code/color_drop_test.py
Ctrl+C 或 SIGTERM 停车。运行前装好各槽位的载荷。
"""

import math
from pathlib import Path
import signal
import time

from components.payload_servos import close_payload, open_payload, payload_configured
from components.servo_positioning import park_servo
from components.t265_pose_adapter import T265PoseAdapter
from config.v2_factory import build_differential_drive, build_servo, build_t265_source
from config.v2_loader import load_v2_config
from core.types import Twist2D
from target_patrol import MODEL_PATH, YoloVision

# 仅在这里调参数；槽位从 0 开始，每种颜色只释放一个槽位。
SLOTS = {"red": 1, "blue": 2, "green": 0, "yellow": 4}
FORWARD_M_S = 0.08
TURN_RAD_S = 0.35
FRONT_DROP_M = 0.43
RIGHT_DROP_M = 0.32
MAX_SEARCH_M = 3.0             # 每两次投放之间最多直行 3 米
MAX_SECONDS = 240.0
POSITION_TOLERANCE_M = 0.015
ANGLE_TOLERANCE_RAD = math.radians(2)
PERIOD_S = 0.02


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main():
    config = load_v2_config(Path(__file__).resolve().parents[1] / "configs/robocup_diffdrive.toml")
    if (config.drive.protocol_mode != "differential_vx_vz"
            or not config.calibration.c10b_diff_firmware_verified
            or not config.drive.allow_in_place_rotation):
        raise RuntimeError("测试需要已验证的差速固件和原地转向")
    drive = build_differential_drive(config)
    source = build_t265_source(config)
    servo = build_servo(config)
    if source is None or servo is None:
        raise RuntimeError("测试需要启用 T265 和摄像头舵机")
    # 读取超时小于底盘看门狗周期；读不到可靠姿态就停车退出。
    source.timeout_ms = 100
    adapter = T265PoseAdapter(config.t265_mount,
        min_tracker_confidence=max(2, config.fusion.t265_min_tracker_confidence),
        max_age_s=min(0.15, config.fusion.t265_max_age_s))
    vision = YoloVision(MODEL_PATH, 0, target_class_name=None, target_region="frame")
    stopped = False
    owned = False
    vision_active = False
    previous = None
    deadline = time.monotonic() + MAX_SECONDS
    handlers = {}

    def request_stop(*_):
        nonlocal stopped
        stopped = True
        drive.stop()

    def guard():
        if stopped:
            raise InterruptedError("测试已停止")
        if time.monotonic() >= deadline:
            raise TimeoutError("测试超过总时限")

    def pose():
        nonlocal previous
        guard()
        raw = source.read()
        now = time.monotonic()
        guard()
        if raw is None:
            raise RuntimeError("T265 无数据")
        update = adapter.adapt(raw, now_s=now)
        current = update.pose
        if current is None or not 0 <= now - current.timestamp_s <= adapter.max_age_s:
            raise RuntimeError("T265 姿态不可用：%s" % update.reason)
        if previous is not None:
            dt = current.timestamp_s - previous.timestamp_s
            distance = math.hypot(current.x_m - previous.x_m, current.y_m - previous.y_m)
            angle = abs(wrap(current.yaw_rad - previous.yaw_rad))
            if (dt <= 0 or dt > 0.3 or distance > 0.03 + 0.4 * dt
                    or angle > 0.06 + 1.2 * dt):
                raise RuntimeError("T265 数据中断或姿态跳变")
        if vision_active:
            vision.observe_colours(now)
        guard()
        previous = current
        return current

    def hold(seconds):
        drive.stop()
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            pose()
            time.sleep(PERIOD_S)

    def turn(yaw):
        print("转向 %.1f 度" % math.degrees(yaw), flush=True)
        until = time.monotonic() + 20
        while True:
            current = pose()
            error = wrap(yaw - current.yaw_rad)
            if abs(error) <= ANGLE_TOLERANCE_RAD:
                break
            if time.monotonic() > until:
                raise TimeoutError("转向超时")
            drive.rotate(math.copysign(max(0.10, min(TURN_RAD_S, 1.5 * abs(error))), error))
            time.sleep(PERIOD_S)
        hold(0.15)

    def travel(origin, yaw, distance):
        print("沿当前抛投方向移动 %.3f 米" % distance, flush=True)
        until = time.monotonic() + 20
        while True:
            current = pose()
            dx, dy = current.x_m - origin.x_m, current.y_m - origin.y_m
            along = dx * math.cos(yaw) + dy * math.sin(yaw)
            cross = -dx * math.sin(yaw) + dy * math.cos(yaw)
            error = distance - along
            tolerance = 0.005 if distance == 0 else POSITION_TOLERANCE_M
            if abs(error) <= tolerance:
                if abs(cross) > 0.03:
                    raise RuntimeError("返回直线横向偏差超过 3cm")
                break
            if abs(cross) > 0.06 or time.monotonic() > until:
                raise RuntimeError("直线动作偏离或超时")
            speed = math.copysign(max(0.025, min(FORWARD_M_S, 1.2 * abs(error))), error)
            omega = max(-0.2, min(0.2, 1.5 * wrap(yaw - current.yaw_rad)
                                  - math.copysign(1.0, speed) * 2 * cross))
            drive.command(Twist2D(speed, omega))
            time.sleep(PERIOD_S)
        hold(0.15)

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            handlers[sig] = signal.signal(sig, request_stop)
        drive.start()             # 先取得小车硬件锁，再操作摄像头与载荷
        owned = True
        if not all(payload_configured(slot) for slot in SLOTS.values()):
            raise RuntimeError("抛投舵机 PWM 不可用")
        park_servo(servo, 90)
        guard()
        source.start()
        vision.start()
        ready_until = time.monotonic() + 45
        # 模型预热及 T265 置信度建立时保持停车。
        while True:
            guard()
            if time.monotonic() > ready_until:
                raise TimeoutError("识别或 T265 启动超时")
            try:
                current = pose()
            except RuntimeError:
                previous = None
                time.sleep(PERIOD_S)
                continue
            if vision.ready:
                vision.observe_colours(time.monotonic())
                vision_active = True
                break
            time.sleep(PERIOD_S)
        if close_payload() is False:
            raise RuntimeError("载荷关闭失败")
        hold(0.5)
        done = set()
        search_start = pose()
        patrol_yaw = search_start.yaw_rad
        last_frame = None
        candidate, confirmations = None, 0
        print("开始直行：红/蓝/绿前置，黄色右侧，每色一次", flush=True)
        while len(done) < len(SLOTS):
            current = pose()
            stamp, colours = vision.observe_colours(time.monotonic())
            chosen = None
            if stamp != last_frame:
                last_frame = stamp
                width = vision.frame_size[0]
                # 沿用已标定的侧视触发：目标中心进入画面中央 10%。
                for color, box in sorted(colours, key=lambda item: item[1][4], reverse=True):
                    if color in SLOTS and color not in done and abs((box[0]+box[2])/2-width/2) <= width*0.05:
                        chosen = color
                        break
                confirmations = confirmations + 1 if chosen == candidate and chosen else int(chosen is not None)
                candidate = chosen
            if confirmations >= 2:
                hold(0.2)
                road = pose()   # 记录识别停车位置；返回这点而非启动点
                side_yaw = patrol_yaw + math.pi/2
                reach = RIGHT_DROP_M if candidate == "yellow" else FRONT_DROP_M
                print("识别 %s，释放槽位 %d" % (candidate, SLOTS[candidate]), flush=True)
                turn(side_yaw)
                travel(road, side_yaw, reach)
                if candidate == "yellow":
                    turn(patrol_yaw + math.pi)
                hold(0.15)
                guard()
                if open_payload(SLOTS[candidate]) is False:
                    raise RuntimeError("抛投 PWM 命令失败")
                hold(0.5)
                if candidate == "yellow":
                    turn(side_yaw)
                travel(road, side_yaw, 0)
                turn(patrol_yaw)
                returned = pose()
                if math.hypot(returned.x_m-road.x_m, returned.y_m-road.y_m) > 0.04:
                    raise RuntimeError("回到巡路位置误差超过 4cm")
                done.add(candidate)
                print("已回到巡路位置，完成：%s" % ", ".join(sorted(done)), flush=True)
                candidate, confirmations = None, 0
                search_start = returned
                continue
            if math.hypot(current.x_m-search_start.x_m, current.y_m-search_start.y_m) >= MAX_SEARCH_M:
                raise RuntimeError("直行搜索到达距离上限，停车")
            drive.command(Twist2D(FORWARD_M_S,
                max(-0.2, min(0.2, 1.5 * wrap(patrol_yaw - current.yaw_rad)))))
            time.sleep(PERIOD_S)
        print("四色抛投完成，停车", flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print("停车：%s" % exc, flush=True)
        return 1
    finally:
        drive.stop()             # 清理任何传感器之前先停止底盘
        try:
            if owned:
                open_payload()  # 完成、STOP、异常均打开所有载荷
        finally:
            try:
                vision.close()
            finally:
                try:
                    source.stop()
                finally:
                    try:
                        if owned:
                            servo.close(hold=True)
                    finally:
                        drive.close()
                        for sig, handler in handlers.items():
                            signal.signal(sig, handler)


if __name__ == "__main__":
    raise SystemExit(main())
