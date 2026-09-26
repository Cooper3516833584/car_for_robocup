#!/usr/bin/env python3
"""LCUS 继电器单文件自检脚本（硬件联调用，不进入单元测试发现）。

默认只读：识别板子路数并查询状态，不会吸合任何继电器；
只有显式给出 --channel 配合 --on/--off，或给出 --all-off，才会动触点。

端口来源优先级: ``--port`` > ``[devices.relay] port``（配合 ``--config``）>
环境变量 ``D_TASK_RELAY_PORT``。

用法::

    python3 code/test/relay_selftest.py --port /dev/relay_lcus
    python3 code/test/relay_selftest.py --config configs/robocup_diffdrive.example.toml
    python3 code/test/relay_selftest.py --port /dev/relay_lcus --channel 1 --on     # 会真实吸合!
    python3 code/test/relay_selftest.py --port /dev/relay_lcus --channel 1 --off
    python3 code/test/relay_selftest.py --port /dev/relay_lcus --all-off
"""

from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys

CODE_ROOT = Path(__file__).resolve().parents[1]
if str(CODE_ROOT) not in sys.path:
    sys.path.insert(0, str(CODE_ROOT))

from components.relay_lcus import LCUSRelay

LOG = logging.getLogger("relay-selftest")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="LCUS relay self test")
    parser.add_argument("--port", help="串口, 例如 /dev/relay_lcus 或 /dev/ttyUSB0")
    parser.add_argument("--config", help="可选: schema-v2 TOML, 用其中的 [devices.relay] 端口")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--channels", type=int, default=8, help="配置路数, 识别后会自动改用实际值")
    parser.add_argument("--query-timeout", type=float, default=1.0)
    parser.add_argument("--channel", type=int, help="目标通道号 (1 开始)")
    state = parser.add_mutually_exclusive_group()
    state.add_argument("--on", action="store_true", help="吸合 --channel 指定的那一路")
    state.add_argument("--off", action="store_true", help="断开 --channel 指定的那一路")
    parser.add_argument("--all-off", action="store_true", help="断开全部通道")
    return parser


def resolve_port(args) -> str | None:
    if args.port:
        return args.port
    if args.config:
        from config.v2_loader import load_v2_config

        relay = load_v2_config(args.config).relay
        if not relay.enabled or not relay.port:
            LOG.error("配置 %s 的 [devices.relay] 未启用或未填写 port", args.config)
            return None
        if args.baud == 9600:
            args.baud = relay.baudrate
        if args.channels == 8:
            args.channels = relay.channel_count
        return relay.port
    return None


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = build_parser().parse_args(argv)
    if args.channel is not None and not (args.on or args.off):
        print("--channel 需要配合 --on 或 --off 之一")
        return 2
    if args.channel is None and (args.on or args.off):
        print("--on/--off 需要配合 --channel 使用; 全板操作请用 --all-off")
        return 2
    port = resolve_port(args)
    if not port:
        print("必须给出 --port（或 --config，或设置环境变量 D_TASK_RELAY_PORT）")
        return 2

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
        print("states  :", relay.query_status())
        if args.channel is not None:
            if not 1 <= args.channel <= detected:
                print(f"--channel 必须在 1~{detected} 之间")
                return 2
            ok = relay.set_channel(args.channel, bool(args.on), verify=True)
            print("set_channel ->", "OK" if ok else "FAILED")
            print("states  :", relay.query_status())
            exit_code = 0 if ok else 1
        if args.all_off:
            ok = relay.all_off(verify=True)
            print("all_off ->", ok)
            print("states  :", relay.query_status())
            if not ok:
                exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
