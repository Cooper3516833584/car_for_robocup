#!/usr/bin/env python3
"""LCUS 继电器单文件自检脚本（硬件联调用，不进入单元测试发现）。

默认只读：识别板子路数并查询状态，不会吸合任何继电器；
只有显式给出 --channel 配合 --on/--off，或给出 --all-off / --blink，才会动触点。

端口来源优先级: ``--port`` > ``[devices.relay] port``（配合 ``--config``）>
环境变量 ``D_TASK_RELAY_PORT``。

用法::

    python3 code/test/relay_selftest.py --list                              # 只列串口
    python3 code/test/relay_selftest.py --print-frames                      # 只打印指令帧
    python3 code/test/relay_selftest.py --port /dev/ttyUSB0 --identify      # 只读: 确认端口是 LCUS 继电器
    python3 code/test/relay_selftest.py --port /dev/relay_lcus              # 只读: 路数 + 状态
    python3 code/test/relay_selftest.py --config configs/robocup_diffdrive.example.toml
    python3 code/test/relay_selftest.py --port /dev/relay_lcus --channel 1 --on     # 会真实吸合!
    python3 code/test/relay_selftest.py --port /dev/relay_lcus --channel 1 --off
    python3 code/test/relay_selftest.py --port /dev/relay_lcus --all-off
    python3 code/test/relay_selftest.py --port /dev/relay_lcus --blink 2 --interval 1 --repeat 3

本项目当前装的是 4 路 LCUS 板（ASCII 返回），板子无损坏通道，所以自检默认按识别到的路数
逐路测试即可，不需要跳过任何通道。``verify=True`` 会先等待 ``DEFAULT_VERIFY_SETTLE``(0.1s)
再回读，正常情况下不再出现"先失败一次再重发"；若仍出现，说明板子比 100ms 更慢或首帧丢失。
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys
import time

CODE_ROOT = Path(__file__).resolve().parents[1]
if str(CODE_ROOT) not in sys.path:
    sys.path.insert(0, str(CODE_ROOT))

from components.relay_lcus import (
    DEFAULT_BAUDRATE,
    DEFAULT_CHANNEL_COUNT,
    LCUSRelay,
    build_channel_command,
    format_port_list,
    format_states,
    list_serial_ports,
    parse_status_response,
)

LOG = logging.getLogger("relay-selftest")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="LCUS relay self test")
    parser.add_argument("--port", help="串口, 例如 /dev/relay_lcus 或 /dev/ttyUSB0")
    parser.add_argument("--config", help="可选: schema-v2 TOML, 用其中的 [devices.relay] 端口")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUDRATE)
    parser.add_argument(
        "--channels",
        type=int,
        default=DEFAULT_CHANNEL_COUNT,
        help=f"配置路数, 默认 {DEFAULT_CHANNEL_COUNT}(本项目 4 路板); 只读识别后自动改用实际值",
    )
    parser.add_argument("--query-timeout", type=float, default=1.0)
    parser.add_argument("--channel", type=int, help="目标通道号 (1 开始)")
    state = parser.add_mutually_exclusive_group()
    state.add_argument("--on", action="store_true", help="吸合 --channel 指定的那一路")
    state.add_argument("--off", action="store_true", help="断开 --channel 指定的那一路")
    parser.add_argument("--all-off", action="store_true", help="断开全部通道")
    parser.add_argument("--blink", type=int, metavar="CH", help="对指定通道循环开/关(会反复吸合!)")
    parser.add_argument("--interval", type=float, default=1.0, help="--blink 的间隔秒数, 默认 1.0")
    parser.add_argument("--repeat", type=int, default=3, help="--blink 的循环次数, 默认 3")
    # 只读工具: 不打开串口, 也不动触点
    parser.add_argument("--list", action="store_true", help="列出当前串口后退出(不开继电器)")
    parser.add_argument(
        "--identify",
        action="store_true",
        help="只发一次 FF 查询并打印原始返回后退出(用来确认这个串口确实是 LCUS 继电器, 不动触点)",
    )
    parser.add_argument(
        "--print-frames",
        action="store_true",
        help="打印 1~N 路的开/关指令帧(不开串口, 不动触点; 只给该参数时打印后退出)",
    )
    return parser


def print_frames(channel_count: int) -> None:
    """打印指令帧速查表, 便于和 SSCOM32/minicom 手工核对。"""
    print("路号  打开指令      关闭指令")
    for channel in range(1, channel_count + 1):
        on_frame = " ".join(f"{byte:02X}" for byte in build_channel_command(channel, True, channel_count))
        off_frame = " ".join(f"{byte:02X}" for byte in build_channel_command(channel, False, channel_count))
        print(f"CH{channel:<3} {on_frame}      {off_frame}")
    print("查询指令 FF")


def print_states(relay) -> str:
    """读一次状态并格式化成 ``CH1=OFF CH2=OFF``; 读不到时明确写出来, 不猜。"""
    states = relay.query_status()
    if states is None:
        return "(FF 查询无有效返回)"
    return format_states(states)


def resolve_port(args) -> str | None:
    if args.port:
        return args.port
    if args.config:
        from config.v2_loader import load_v2_config

        relay = load_v2_config(args.config).relay
        if not relay.enabled or not relay.port:
            LOG.error("配置 %s 的 [devices.relay] 未启用或未填写 port", args.config)
            return None
        if args.baud == DEFAULT_BAUDRATE:
            args.baud = relay.baudrate
        if args.channels == DEFAULT_CHANNEL_COUNT:
            args.channels = relay.channel_count
        return relay.port
    return None


def identify_port(port: str, args) -> int:
    """只发一次 FF 查询并打印原始返回, 用来确认该串口确实是 LCUS 继电器。

    不会发送任何控制帧, 因此不会改变触点状态。LCUS 板回 ``CHn: ON/OFF`` 文本或
    ``channel_count`` 字节的二进制快照; 空返回说明这个端口不是 LCUS(或波特率/供电不对)。
    """
    with LCUSRelay(
        port=port,
        baudrate=args.baud,
        channel_count=args.channels,
        query_timeout=args.query_timeout,
    ) as relay:
        raw = relay.read_raw_response()
        print("port    :", port)
        print("raw len :", len(raw))
        print("raw hex :", " ".join(f"{byte:02X}" for byte in raw) if raw else "(空)")
        print("raw repr:", raw)
        states = parse_status_response(raw, args.channels)
    if not raw:
        print("没有收到任何返回: 端口不是 LCUS 继电器, 或波特率/供电/接线不对")
        return 1
    if not states:
        print("收到数据但无法解析成 LCUS 状态: 请用串口工具人工核对返回格式")
        return 1
    print("states  :", format_states(states))
    print("识别为该串口上的 LCUS 继电器(只发送了 FF 查询帧, 未动触点)")
    return 0


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = build_parser().parse_args(argv)

    if args.list:
        print("当前串口:")
        print(f"  {format_port_list()}")
        if not list_serial_ports():
            print("  pyserial 未安装或系统未发现任何串口")
    if args.print_frames:
        print_frames(args.channels)
        if not (args.port or args.config):
            return 0

    # 只给了 --list / --print-frames（没有要动硬件的动作, 也没有端口）: 打印完即结束
    if (args.list or args.print_frames) and not any(
        (args.port, args.config, args.channel is not None, args.all_off, args.blink is not None, args.identify)
    ):
        return 0

    if args.channel is not None and not (args.on or args.off):
        print("--channel 需要配合 --on 或 --off 之一")
        return 2
    if args.channel is None and (args.on or args.off):
        print("--on/--off 需要配合 --channel 使用; 全板操作请用 --all-off")
        return 2
    if args.blink is not None and (args.repeat <= 0 or args.interval <= 0):
        print("--blink 需要 --repeat 和 --interval 为正数")
        return 2
    port = resolve_port(args)
    if not port:
        print("必须给出 --port（或 --config，或设置环境变量 D_TASK_RELAY_PORT）")
        return 2

    if args.identify:
        # 只读: 发一次 FF 查询, 打印原始返回。LCUS 板会回 "CHn: ON/OFF" 文本或二进制快照,
        # 其他设备(例如 C10B)不会对 FF 有这种回应, 因此可用来确认端口归属。
        return identify_port(port, args)

    # 第一步: 只读识别路数(只发 FF 查询帧)
    with LCUSRelay(
        port=port,
        baudrate=args.baud,
        channel_count=args.channels,
        query_timeout=args.query_timeout,
    ) as probe:
        detected = probe.detect_channel_count()
    print("detected:", detected)
    if detected is None:
        print("没有收到有效返回: 请确认端口是继电器, 且波特率为 9600")
        return 1

    # 第二步: 用识别到的路数重新构造, 避免 4 路板出现"缺少通道"告警
    exit_code = 0
    with LCUSRelay(
        port=port,
        baudrate=args.baud,
        channel_count=detected,
        query_timeout=args.query_timeout,
    ) as relay:
        print("states  :", print_states(relay))
        if args.channel is not None:
            if not 1 <= args.channel <= detected:
                print(f"--channel 必须在 1~{detected} 之间")
                return 2
            ok = relay.set_channel(args.channel, bool(args.on), verify=True)
            print("set_channel ->", "OK" if ok else "FAILED")
            print("states  :", print_states(relay))
            exit_code = 0 if ok else 1
        if args.all_off:
            ok = relay.all_off(verify=True)
            print("all_off ->", ok)
            print("states  :", print_states(relay))
            if not ok:
                exit_code = 1
        if args.blink is not None:
            if not 1 <= args.blink <= detected:
                print(f"--blink 必须在 1~{detected} 之间")
                return 2
            # 风险: 反复吸合同一路, 注意负载与继电器机械寿命
            for index in range(args.repeat):
                for requested in (True, False):
                    if not relay.set_channel(args.blink, requested, verify=True):
                        print(f"第{args.blink}路切换 {'ON' if requested else 'OFF'} 未通过回读确认")
                        return 1
                    print(f"blink   : {index + 1}/{args.repeat} 第{args.blink}路 {'ON' if requested else 'OFF'}")
                    time.sleep(args.interval)
            print("states  :", print_states(relay))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
