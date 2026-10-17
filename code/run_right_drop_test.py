#!/usr/bin/env python3
"""右侧抛投功能测试：不读定位、不巡线，整套动作只用“指令量 + 时间”开环驱动。

序列（侧向几何与正式 ``payload_detour`` 的 ``right_mount`` 一致）：

1. 巡线前进 ``FORWARD_1_S``（默认 2 s）后停车；
2. 左转 90°（先转出线，与正式 right_mount 同向）；
3. 侧向前进 ``SIDE_TIME_S``（约 0.32 m，离开巡线）；
4. 再左转 90°：车头朝原巡线反方向，右侧安装的 slot 4 朝外；
5. 停车释放 slot 4 并保持 ``RELEASE_HOLD_S``；
6. 右转 90° → 侧向倒回 ``SIDE_TIME_S`` → 右转 90°，回到巡线航向；
7. 继续前进 ``FORWARD_2_S``（默认 2 s）后停车。

本程序不构造融合位姿、不读 D500 墙面、不做巡线，因此转弯角度和侧向距离都是
开环估计：转角没有陀螺反馈，侧向距离没有里程反馈。默认值取自实测记录的换算，
首次现场运行必须实测核对并按需调整（见 ``docs/PAYLOAD_SG90.md``）。

需要 ``--confirm-motor-test`` 才动车；``--simulate-release`` 只跳过物资 PWM，
动作仍会执行。Ctrl+C / SIGTERM / 日志目录中的 ``STOP`` 文件都会立即停车。
退出时先停车，再把五只物资全部 OPEN（除非显式模拟）。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import logging
import math
from pathlib import Path
import signal
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))

from components.diagnostics_log import JsonlEventLogger  # noqa: E402
from components.payload_servos import close_payload, open_payload  # noqa: E402
from config.v2_factory import build_differential_drive  # noqa: E402
from config.v2_loader import load_v2_config  # noqa: E402
from core.types import Twist2D  # noqa: E402

# ---------------------------------------------------------------------------
# 快速调整区：现场可直接改这里的默认值（命令行同名参数优先）。
# ---------------------------------------------------------------------------
PAYLOAD_SLOT = 4              # 右侧安装的物资槽；正式 right_mount 只允许 4
TRAVEL_SPEED_M_S = 0.10       # 前进/侧向/倒回的指令速度（上限 0.20 m/s）
FORWARD_1_S = 2.0             # 第一次巡线前进时间
FORWARD_2_S = 2.0             # 回到巡线后继续前进时间
SIDE_TIME_S = 3.6             # 侧向时间：假设 0.32 m / (0.10 x 0.889) ≈ 3.6 s
SIDE_DISTANCE_M = 0.32        # 正式 right_mount 的侧向距离，仅用于打印预期值
WHEEL_TRACKING = 0.889        # 实测轮速环跟踪率，用于把距离换算成时间（未再复测）
TURN_RATE_RAD_S = 0.50        # 原地转向指令角速度（须高于约 0.3 rad/s 静摩擦死区）
TURN_TIME_S = 3.6             # 90° 时间：假设实测偏航率≈指令值 0.5 rad/s，
                              # 加/减速各约 0.125 rad，故 (1.571+0.25)/0.5 ≈ 3.6 s
TURN_ANGLE_DEG = 90.0         # 每次转向的目标角度，只用于打印预期值
DETOUR_DIRECTION = +1         # +1=先左转（与正式 right_mount 一致）；-1=先右转
RELEASE_HOLD_S = 0.5          # 释放后保持时间，与正式投放一致
CONTROL_PERIOD_S = 0.05       # 指令重发周期，须小于 command_timeout_s=0.25 s


@dataclass(frozen=True)
class RightDropSettings:
    """One timed right-drop sequence; every field is a direct drive input."""

    payload_slot: int = PAYLOAD_SLOT
    travel_speed_m_s: float = TRAVEL_SPEED_M_S
    forward_1_s: float = FORWARD_1_S
    forward_2_s: float = FORWARD_2_S
    side_time_s: float = SIDE_TIME_S
    turn_rate_rad_s: float = TURN_RATE_RAD_S
    turn_time_s: float = TURN_TIME_S
    release_hold_s: float = RELEASE_HOLD_S
    detour_direction: int = DETOUR_DIRECTION
    control_period_s: float = CONTROL_PERIOD_S

    def __post_init__(self):
        if (isinstance(self.payload_slot, bool) or not isinstance(self.payload_slot, int)
                or self.payload_slot not in range(5)):
            raise ValueError("payload slot must be 0..4")
        if self.detour_direction not in (-1, 1):
            raise ValueError("detour direction must be +1 (left first) or -1 (right first)")
        for name in ("travel_speed_m_s", "forward_1_s", "forward_2_s", "side_time_s",
                     "turn_rate_rad_s", "turn_time_s", "release_hold_s", "control_period_s"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")


def _hold_twist(drive, twist, seconds, *, clock, sleep, guard, settings):
    """Feed one constant twist until the deadline, then stop the drive.

    The command is re-sent every control period: the drive's watchdog zeroes the
    output after ``command_timeout_s`` without a new command, and the rate
    limiter needs the repeated timestamps to ramp.
    """
    deadline = float(clock()) + float(seconds)
    while True:
        guard()
        now = float(clock())
        if now >= deadline:
            break
        drive.command(twist, now_s=now)
        sleep(min(settings.control_period_s, max(0.0, deadline - now)))
    guard()
    drive.stop()


def run_right_drop_test(drive, settings, *, release=None, guard=lambda: None,
                        clock=time.monotonic, sleep=time.sleep,
                        on_event=lambda *args, **kwargs: None,
                        simulate_release=False) -> tuple[str, ...]:
    """Run the timed sequence; return the completed step names.

    ``release()`` performs the payload command and returns False when the PWM
    write was not complete. ``simulate_release`` skips it entirely while keeping
    every motion step, exactly like the existing payload simulation switch.
    """
    if release is None and not simulate_release:
        raise ValueError("the right drop test requires a payload release callable")
    completed: list[str] = []
    zero = Twist2D(0.0, 0.0)

    def run(label, seconds, twist):
        guard()
        on_event("right_drop_test_step", step=label, duration_s=seconds,
                 linear_x_m_s=twist.linear_x_m_s, angular_z_rad_s=twist.angular_z_rad_s)
        _hold_twist(drive, twist, seconds, clock=clock, sleep=sleep, guard=guard,
                    settings=settings)
        completed.append(label)
        on_event("right_drop_test_step_done", step=label)

    def turn(label, sign):
        run(label, settings.turn_time_s, Twist2D(0.0, sign * settings.turn_rate_rad_s))

    def travel(label, sign, seconds):
        run(label, seconds, Twist2D(sign * settings.travel_speed_m_s, 0.0))

    try:
        travel("forward_1", +1, settings.forward_1_s)
        turn("turn_out_1", settings.detour_direction)
        travel("side_out", +1, settings.side_time_s)
        turn("turn_out_2", settings.detour_direction)
        guard()
        if simulate_release:
            on_event("right_drop_test_release_skipped", slot=settings.payload_slot,
                     reason="explicit payload simulation; physical output disabled")
        else:
            on_event("right_drop_test_release_start", slot=settings.payload_slot)
            if release() is False:
                raise RuntimeError("payload release PWM command was not complete")
            completed.append("release")
            on_event("right_drop_test_release_done", slot=settings.payload_slot,
                     reason="PWM command written; no mechanical feedback")
        # The stopped hold keeps the drive fed and observes guard/STOP.
        _hold_twist(drive, zero, settings.release_hold_s, clock=clock, sleep=sleep,
                    guard=guard, settings=settings)
        turn("turn_back_1", -settings.detour_direction)
        travel("side_back", -1, settings.side_time_s)
        turn("turn_back_2", -settings.detour_direction)
        travel("forward_2", +1, settings.forward_2_s)
    finally:
        drive.stop()
    return tuple(completed)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=str(ROOT / "configs/robocup_diffdrive.toml"),
                        help="measured hardware profile; the board's own TOML, never the example")
    parser.add_argument("--log-dir", type=Path,
                        help="evidence directory; default logs/right-drop-test-<timestamp>")
    parser.add_argument("--confirm-motor-test", action="store_true",
                        help="required: this is an explicitly requested, operator-supervised motor test")
    parser.add_argument("--simulate-release", action="store_true",
                        help="run every motion step but skip the payload PWM command")
    parser.add_argument("--payload-slot", type=int, choices=(0, 1, 2, 3, 4), default=PAYLOAD_SLOT)
    parser.add_argument("--travel-speed-cm-s", type=float, default=TRAVEL_SPEED_M_S * 100.0)
    parser.add_argument("--forward-1-s", type=float, default=FORWARD_1_S)
    parser.add_argument("--forward-2-s", type=float, default=FORWARD_2_S)
    parser.add_argument("--side-time-s", type=float, default=SIDE_TIME_S)
    parser.add_argument("--turn-rate-rad-s", type=float, default=TURN_RATE_RAD_S)
    parser.add_argument("--turn-time-s", type=float, default=TURN_TIME_S)
    parser.add_argument("--release-hold-s", type=float, default=RELEASE_HOLD_S)
    parser.add_argument("--detour-direction", type=int, choices=(-1, 1), default=DETOUR_DIRECTION,
                        help="+1 turns left first (production right_mount), -1 turns right first")
    return parser


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not args.confirm_motor_test:
        parser.error("pass --confirm-motor-test for this explicitly requested hardware test")
    try:
        settings = RightDropSettings(
            payload_slot=args.payload_slot,
            travel_speed_m_s=args.travel_speed_cm_s / 100.0,
            forward_1_s=args.forward_1_s,
            forward_2_s=args.forward_2_s,
            side_time_s=args.side_time_s,
            turn_rate_rad_s=args.turn_rate_rad_s,
            turn_time_s=args.turn_time_s,
            release_hold_s=args.release_hold_s,
            detour_direction=args.detour_direction,
        )
    except ValueError as exc:
        parser.error(str(exc))
    try:
        config = load_v2_config(args.config)
    except (FileNotFoundError, ValueError, NotImplementedError) as exc:
        parser.error(f"cannot load {args.config}: {exc}")
    if settings.travel_speed_m_s > config.drive.max_linear_speed_m_s:
        parser.error(f"travel speed {settings.travel_speed_m_s:g} m/s exceeds the profile limit "
                     f"{config.drive.max_linear_speed_m_s:g} m/s")
    if settings.turn_rate_rad_s > config.drive.max_angular_speed_rad_s:
        parser.error(f"turn rate {settings.turn_rate_rad_s:g} rad/s exceeds the profile limit "
                     f"{config.drive.max_angular_speed_rad_s:g} rad/s")
    if settings.control_period_s >= config.drive.command_timeout_s:
        parser.error(f"control period {settings.control_period_s:g}s must stay below the drive "
                     f"command timeout {config.drive.command_timeout_s:g}s")

    log_dir = args.log_dir or Path("logs") / f"right-drop-test-{time.strftime('%Y%m%d-%H%M%S')}"
    try:
        log_dir.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        parser.error(f"log directory already exists: {log_dir}")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    stop_file = log_dir / "STOP"
    aborted = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: aborted.set())

    logger = JsonlEventLogger(log_dir / "events.jsonl")
    started = time.monotonic()

    def emit(event_type, **fields):
        logger.emit({"t": time.monotonic() - started, "type": event_type, **fields}, priority=True)

    step_count = 0

    def report(event_type, **fields):
        nonlocal step_count
        emit(event_type, **fields)
        if event_type == "right_drop_test_step":
            step_count += 1
            print(f"[右侧抛投] 第{step_count}步 {fields['step']}: {fields['duration_s']:.1f}s "
                  f"v={fields['linear_x_m_s']:+.2f} m/s w={fields['angular_z_rad_s']:+.2f} rad/s",
                  flush=True)
        elif event_type == "right_drop_test_release_done":
            print(f"[右侧抛投] slot {fields['slot']} 已释放（仅 PWM 命令，无机械反馈）", flush=True)

    direction = "左" if settings.detour_direction > 0 else "右"
    expected_side_m = settings.travel_speed_m_s * settings.side_time_s * WHEEL_TRACKING
    print(f"[右侧抛投] 开环时间序列：前进{settings.forward_1_s:g}s → {direction}转{TURN_ANGLE_DEG:g}° → "
          f"侧向{settings.side_time_s:g}s(预期约{expected_side_m:.2f}m，正式值{SIDE_DISTANCE_M:g}m) → "
          f"{direction}转{TURN_ANGLE_DEG:g}° → 释放slot{settings.payload_slot} → 反向转回 → "
          f"侧向倒回 → 前进{settings.forward_2_s:g}s", flush=True)
    print(f"[右侧抛投] 证据目录 {log_dir}；STOP 文件 {stop_file}", flush=True)
    if args.simulate_release:
        print("[右侧抛投] 模拟模式：不写物资 PWM，但整套动作仍会执行", flush=True)

    drive = build_differential_drive(config, fake=False)
    outcome_error = None
    try:
        drive.start()
        emit("right_drop_test_start", payload_slot=settings.payload_slot,
             simulate_release=args.simulate_release, detour_direction=settings.detour_direction,
             travel_speed_m_s=settings.travel_speed_m_s, forward_1_s=settings.forward_1_s,
             forward_2_s=settings.forward_2_s, side_time_s=settings.side_time_s,
             turn_rate_rad_s=settings.turn_rate_rad_s, turn_time_s=settings.turn_time_s,
             release_hold_s=settings.release_hold_s, config=str(args.config),
             feedback_source="timed_open_loop")
        if not args.simulate_release and close_payload() is False:
            raise RuntimeError("payload CLOSE command was not complete; refusing to move")
        emit("right_drop_test_payload_closed", slot=settings.payload_slot,
             simulate_release=args.simulate_release)

        def guard():
            if aborted.is_set():
                raise RuntimeError("right drop test interrupted")
            if stop_file.exists():
                raise RuntimeError("right drop test interrupted by STOP file")

        completed = run_right_drop_test(
            drive, settings,
            release=lambda: open_payload(settings.payload_slot),
            guard=guard, on_event=report, simulate_release=args.simulate_release)
        (log_dir / "result.txt").write_text("FINISHED\n")
        print(f"[右侧抛投] 完成：{','.join(completed)}", flush=True)
        return 0
    except BaseException as exc:
        outcome_error = f"{type(exc).__name__}: {exc}"
        try:
            drive.stop()
        except Exception as stop_error:
            print(f"[右侧抛投] 停车失败: {stop_error}", file=sys.stderr, flush=True)
        emit("right_drop_test_failed", reason=outcome_error)
        (log_dir / "result.txt").write_text(f"FAILED: {outcome_error}\n")
        print(f"[右侧抛投] 失败: {outcome_error}", file=sys.stderr, flush=True)
        return 1
    finally:
        cleanup_error = None
        for label, cleanup in (
                ("drive_stop", drive.stop),
                ("payload_open", open_payload if not args.simulate_release else (lambda: None)),
                ("drive_close", drive.close),
                ("logger_close", logger.close)):
            try:
                if cleanup() is False:
                    raise RuntimeError(f"{label} was not confirmed")
            except Exception as exc:
                cleanup_error = cleanup_error or exc
                print(f"[右侧抛投] {label} 收尾失败: {exc}", file=sys.stderr, flush=True)
        if cleanup_error is not None and outcome_error is None:
            (log_dir / "result.txt").write_text(f"FAILED: cleanup: {cleanup_error}\n")
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
