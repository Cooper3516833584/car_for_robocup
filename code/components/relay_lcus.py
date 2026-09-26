"""LCUS 系列 4/8 路 USB 继电器驱动（板载 CH340 转串口）。

移植说明（相对独立交付的单文件版本）:

- 日志由 ``loguru`` 改为标准库 ``logging``（本仓库统一约定），因此本模块的唯一第三方
  依赖是 ``pyserial``，而且仍是惰性导入：没有 pyserial 也能 import 本模块，并使用
  ``build_channel_command()`` / ``parse_status_response()`` 等纯函数；
- ``open()`` 支持注入 ``serial_factory``（串口样式对象的工厂），便于用假串口做单元测试；
- 新增 ``FakeLCUSRelay``：dry-run / 回放使用的内存继电器，不打开任何串口；
- 端口等参数由调用方给出。在本仓库正式运行时，参数来自 TOML 的 ``[devices.relay]``
  节（见 ``code/config/v2_factory.py`` 的 ``build_relay``）；组件本身不读 TOML。
  单文件独立使用时仍可显式传 ``port`` 或设置环境变量 ``D_TASK_RELAY_PORT``。

协议、控制帧、异常语义、锁粒度均与交付版本一致，未作改动。

硬件与协议（来自厂家说明与已知规律）:

- 设备: LCUS 型 USB 继电器, 4 路或 8 路, 板载 CH340 转串口, 固定 9600 8N1;
- 控制帧固定 4 字节: ``A0 | 路号 | 状态 | 校验``
    - 路号 ``0x01``~``0x08`` 对应第 1~8 路, 依次类推;
    - 状态 ``0x01`` = 开, ``0x00`` = 关;
    - 校验 = 前三字节按字节求和取低 8 位;
    - 例: 第 1 路开 ``A0 01 01 A2``, 第 1 路关 ``A0 01 00 A1``;
- 状态查询: 发送单字节 ``FF``, 继电器返回每一路 10 字节 ASCII 文本行, 例如
    ``"CH1: ON \\r\\nCH2: ON \\r\\nCH3: OFF\\r\\nCH4: OFF\\r\\n"``;
    开与关的返回长度相同, 8 路共 80 字节。控制帧本身没有应答,
    因此需要确认控制结果时用 ``set_channel(..., verify=True)`` 触发一次 FF 回读。

设计约束:

- 本模块与其他组件驱动同级, 但它是完全独立的 USB 串口设备: 不经过 C10B 串口,
  不接收也不调用任何底盘命令接口;
- 串口打开时显式关闭 DTR/RTS, 与本仓库其他 CH340 设备的既有处理一致;
- ``open()`` 不主动改变任何一路继电器状态, 需要全部断开时显式调用 ``all_off()``;
- 端口不得猜测: CH340 的 USB ID 与 HC-14 电台相同(``1a86:7523``), 必须显式传入
  ``port`` 或由 TOML / ``D_TASK_RELAY_PORT`` 指定。

风险与边界 (实机使用前必读):

- 状态保持: 继电器触点由板子自锁保持, ``close()`` 和进程退出都**不会**断开触点;
  需要断开必须显式调用 ``all_off()``, 不要假设"程序退出 = 断开"。
  实机观察: USB 断电重插后板子复位为全部 OFF, 但不得依赖该行为作为安全手段。
- 已验证范围(相对交付版本): 仅在一块 4 路 LCUS 板 (PC/COM3, CH340, 9600 8N1) 上验证过
  只读 FF 查询、``detect_channel_count()`` 和单路开/关; 8 路板、第 5~8 路映射、
  多路同时吸合、长时间稳定性、运行中拔插 USB 均**未验证**。
- 首次回读可能滞后: 实测控制帧后约 50ms 内 FF 回读仍返回旧状态, 因此 ``verify=True``
  会稳定出现"第一次判定失败 -> 同状态重发 -> 第二次确认成功"。
  同路同状态重发是幂等的、方向安全, 但每次带校验的操作可能多发一帧;
  若首帧真的丢失, 重发是必需的, 因此不要为了少发一帧而关掉重发/校验。
- DTR/RTS: 打开串口时显式拉低 DTR/RTS, 个别继电器板可能把这两根线接到其他电路,
  实机副作用尚未验证。
- 无硬件互锁: 本驱动走独立 USB 串口, 不受底盘看门狗、ACK/重试/心跳保护, 也没有任何
  硬件互锁。不要把它当作安全关键回路; 急停、超时和安全兜底由任务层负责。
- 异常语义: 参数错误抛 ``ValueError``, 串口未打开抛 ``RuntimeError``,
  串口读写故障(``serial.SerialException`` 等)直接上抛; 驱动不吞异常、也不做安全兜底。
- 并发边界: 内部 ``RLock`` 只保证单次控制帧/查询帧不被交错; ``set_all()`` 是逐路加锁,
  不保证跨路原子性(其他线程可能插入到两路之间)。
- 协议解析边界: 校验规则与返回格式来自说明书和 4 路板实测; 换用其他固件时,
  若返回行格式不同(例如缺行尾空格/换行不同), ``parse_status_response()`` 会忽略无法匹配的行,
  表现为"缺少通道"告警, 需要重新确认协议后再使用。
- 时序边界: 固定 9600 8N1, 4 字节控制帧约 4.2ms; 一次 FF 查询默认最多等待
  ``query_timeout``(默认 1.0s), 等待期间持有内部锁, 不要在高频控制回环里同步调用。

示例::

    from components.relay_lcus import LCUSRelay

    with LCUSRelay(port="/dev/relay_lcus") as relay:   # PC(Windows) 上换成 "COM5" 这样的串口名
        relay.set_channel(1, True, verify=True)        # 开第 1 路并用 FF 回读确认
        relay.get_channel_state(1)                     # 查询第 1 路状态
        relay.all_off()                                # 全部断开(退出前必须显式调用)

正式运行时不直接构造本类, 而是由 ``build_relay()`` 从 TOML 配置构造。
"""

from __future__ import annotations

import logging
import os
import re
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

LOG = logging.getLogger(__name__)

RELAY_PORT_ENV = "D_TASK_RELAY_PORT"

DEFAULT_BAUDRATE = 9600
DEFAULT_CHANNEL_COUNT = 8
MAX_CHANNEL_COUNT = 8
MIN_CHANNEL = 1

CMD_PREFIX = 0xA0
QUERY_BYTE = 0xFF
STATE_ON = 0x01
STATE_OFF = 0x00

# "CH1: ON \r\n" / "CH1: OFF\r\n", 每路固定 10 字节
STATUS_PATTERN = re.compile(r"CH\s*(\d+)\s*:\s*(ON|OFF)", re.IGNORECASE)

__all__ = [
    "LCUSRelay",
    "FakeLCUSRelay",
    "build_channel_command",
    "parse_status_response",
    "format_states",
    "list_serial_ports",
    "format_port_list",
    "resolve_relay_settings",
    "RELAY_PORT_ENV",
    "DEFAULT_BAUDRATE",
    "DEFAULT_CHANNEL_COUNT",
    "MAX_CHANNEL_COUNT",
    "MIN_CHANNEL",
]


def resolve_relay_settings(port: Optional[str] = None, baudrate: Optional[int] = None) -> Tuple[str, int]:
    """解析串口参数: 端口优先取显式参数, 其次取环境变量 ``D_TASK_RELAY_PORT``。"""
    resolved_port = port or os.environ.get(RELAY_PORT_ENV)
    if not resolved_port:
        raise ValueError(
            "未指定 LCUS 继电器串口: 请传入 port 参数或设置环境变量 "
            f"{RELAY_PORT_ENV}; 当前可用串口: {format_port_list()}"
        )
    raw_baudrate = DEFAULT_BAUDRATE if baudrate is None else baudrate
    try:
        resolved_baudrate = int(raw_baudrate)
    except (TypeError, ValueError) as exc:
        raise ValueError("LCUS 继电器波特率必须是整数") from exc
    if resolved_baudrate <= 0:
        raise ValueError("LCUS 继电器波特率必须为正数")
    return resolved_port, resolved_baudrate


def list_serial_ports() -> List[Tuple[str, str]]:
    """列出当前串口 ``(设备节点, hwid)``, 用于确认继电器实际接入的端口。"""
    try:
        from serial.tools import list_ports
    except ImportError:
        LOG.warning("[RELAY] 未安装 pyserial, 无法枚举串口")
        return []
    return [(port.device, port.hwid) for port in sorted(list_ports.comports(), key=lambda p: p.device)]


def format_port_list() -> str:
    """把当前串口格式化成一行文本, 便于写进日志和异常信息。"""
    ports = list_serial_ports()
    if not ports:
        return "未发现串口"
    return "; ".join(f"{device} ({hwid})" for device, hwid in ports)


def _validate_channel(channel: int, channel_count: int) -> int:
    if isinstance(channel, bool) or not isinstance(channel, int):
        raise ValueError(f"继电器路号必须是整数, 实际 {channel!r}")
    if channel < MIN_CHANNEL or channel > channel_count:
        raise ValueError(f"继电器路号必须在 {MIN_CHANNEL}~{channel_count} 之间, 实际 {channel}")
    return channel


def _validate_channel_count(channel_count: int) -> int:
    if isinstance(channel_count, bool) or not isinstance(channel_count, int):
        raise ValueError(f"继电器路数必须是整数, 实际 {channel_count!r}")
    if channel_count < MIN_CHANNEL or channel_count > MAX_CHANNEL_COUNT:
        raise ValueError(f"继电器路数必须在 {MIN_CHANNEL}~{MAX_CHANNEL_COUNT} 之间, 实际 {channel_count}")
    return channel_count


def build_channel_command(channel: int, on: bool, channel_count: int = DEFAULT_CHANNEL_COUNT) -> bytes:
    """构造单路控制帧 ``A0 | 路号 | 状态 | 校验``。

    例: ``build_channel_command(1, True) == bytes.fromhex("A00101A2")``,
    ``build_channel_command(1, False) == bytes.fromhex("A00100A1")``。
    """
    _validate_channel(channel, channel_count)
    state = STATE_ON if on else STATE_OFF
    checksum = (CMD_PREFIX + channel + state) & 0xFF
    return bytes((CMD_PREFIX, channel, state, checksum))


def parse_status_response(data: Union[bytes, bytearray, str], channel_count: int = DEFAULT_CHANNEL_COUNT) -> Dict[int, bool]:
    """解析 FF 查询返回的 ``CHn: ON/OFF`` 文本, 返回 ``{路号: True/False}``。

    容忍分片接收、前后噪声和行尾空白; 只保留 1~``channel_count`` 的结果。
    """
    if isinstance(data, (bytes, bytearray)):
        text = bytes(data).decode("ascii", errors="ignore")
    else:
        text = str(data)
    states: Dict[int, bool] = {}
    for match in STATUS_PATTERN.finditer(text):
        channel = int(match.group(1))
        if MIN_CHANNEL <= channel <= channel_count:
            states[channel] = match.group(2).upper() == "ON"
    return states


def format_states(states: Dict[int, bool]) -> str:
    """把 ``{路号: 状态}`` 格式化成 ``CH1=ON CH2=OFF`` 形式的文本。"""
    if not states:
        return "(空)"
    return " ".join(f"CH{channel}={'ON' if states[channel] else 'OFF'}" for channel in sorted(states))


def _hex(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


class LCUSRelay(object):
    """LCUS 4/8 路 USB 继电器驱动 (CH340 转串口, 9600 8N1)。

    串口访问由内部锁串行化, 避免多线程同时发控制帧和查询帧时请求/应答错配。
    参数错误抛 ``ValueError``; 串口未打开抛 ``RuntimeError``;
    串口读写故障(``serial.SerialException`` 等)直接上抛, 由调用方按现有安全路径处理。

    ``serial_factory`` 是给测试和离线联调用的注入点: 传入一个返回"串口样式对象"的
    可调用对象后, ``open()`` 不再导入 pyserial, 而是直接使用该对象; 注入对象需提供
    ``open()``/``close()``/``write()``/``flush()``/``read()``/``in_waiting``/
    ``reset_input_buffer()``/``setDTR()``/``setRTS()``。
    正式运行时保持 ``None``, 走 pyserial 默认路径。
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: Optional[int] = None,
        channel_count: int = DEFAULT_CHANNEL_COUNT,
        timeout: float = 0.2,
        query_timeout: float = 1.0,
        serial_factory: Optional[Callable[[], Any]] = None,
    ) -> None:
        """
        port: 串口设备, 例如 ``/dev/relay_lcus`` 或 ``COM5``; 为空时取环境变量 ``D_TASK_RELAY_PORT``
        baudrate: 波特率, 默认 9600
        channel_count: 继电器路数, 默认 8
        timeout: 单次串口读超时(秒)
        query_timeout: 一次 FF 状态查询等待完整返回的总超时(秒)
        serial_factory: 可选的串口样式对象工厂, 仅用于测试/离线联调
        """
        self._port, self._baudrate = resolve_relay_settings(port, baudrate)
        self.channel_count = _validate_channel_count(channel_count)
        self._timeout = float(timeout)
        self._query_timeout = float(query_timeout)
        self._serial_factory = serial_factory
        self._serial = None
        self._lock = threading.RLock()

    # ------------------------------------------------------------------ 基本信息

    @property
    def port(self) -> str:
        return self._port

    @property
    def baudrate(self) -> int:
        return self._baudrate

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._serial is not None

    # ------------------------------------------------------------------ 打开/关闭

    def _new_serial(self):
        """构造(尚未打开的)串口对象: 注入工厂优先, 否则惰性导入 pyserial 并配好参数。"""
        if self._serial_factory is not None:
            return self._serial_factory()
        try:
            import serial
        except ImportError as exc:  # 未安装 pyserial 时不影响本模块被导入
            raise RuntimeError("未安装 pyserial, 无法打开 LCUS 继电器串口") from exc

        serial_obj = serial.Serial()
        serial_obj.port = self._port
        serial_obj.baudrate = self._baudrate
        serial_obj.bytesize = serial.EIGHTBITS
        serial_obj.parity = serial.PARITY_NONE
        serial_obj.stopbits = serial.STOPBITS_ONE
        serial_obj.timeout = self._timeout
        serial_obj.write_timeout = 1
        serial_obj.xonxoff = False
        serial_obj.rtscts = False
        serial_obj.dsrdtr = False
        # 与本仓库其他 CH340 设备一致, 显式拉低 DTR/RTS。
        # 边界: 个别继电器板把 DTR/RTS 接到其他电路, 副作用未验证; 改这里前先实测。
        serial_obj.dtr = False
        serial_obj.rts = False
        return serial_obj

    def open(self) -> None:
        """打开串口。重复调用无副作用, 不发送任何指令, 不改变继电器状态。

        风险: 打开串口本身不动触点, 但后续任何 ``set_channel``/``all_on`` 都会真实吸合继电器;
        打开前请确认负载与电源安全。
        """
        with self._lock:
            if self._serial is not None:
                return
            serial_obj = self._new_serial()
            serial_obj.open()
            serial_obj.setDTR(False)
            serial_obj.setRTS(False)
            serial_obj.reset_input_buffer()
            self._serial = serial_obj
            LOG.info("[RELAY] 已打开 %s @ %s 8N1", self._port, self._baudrate)

    def close(self) -> None:
        """关闭串口并释放句柄。重复调用无副作用, 不改变继电器状态。

        风险: 关闭串口**不会**断开已吸合的触点, 继电器由板子自锁保持;
        需要断开时必须在 ``close()`` 之前显式调用 ``all_off()``。
        """
        with self._lock:
            serial_obj = self._serial
            self._serial = None
        if serial_obj is None:
            return
        try:
            serial_obj.close()
        except Exception as exc:  # 关闭失败只记录, 不再上抛
            LOG.warning("[RELAY] 关闭串口失败: %s", exc)
            return
        LOG.info("[RELAY] 已关闭 %s", self._port)

    def __enter__(self) -> "LCUSRelay":
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.close()
        return False

    def _require_serial(self):
        serial_obj = self._serial
        if serial_obj is None:
            raise RuntimeError("LCUS 继电器串口未打开, 请先调用 open()")
        return serial_obj

    # ------------------------------------------------------------------ 控制接口

    def set_channel(self, channel: int, on: bool, verify: bool = False, retries: int = 1) -> bool:
        """控制单路继电器开关。

        channel: 路号 1~``channel_count``
        on: True 开, False 关
        verify: True 时发送 FF 回读状态并比对, 用于确认控制是否成功
        retries: 校验失败后的重发次数(默认 1 次)

        return: 控制帧已发出返回 True; ``verify=True`` 时只有回读一致才返回 True。
        串口写入/读取故障会上抛异常。

        风险与边界:
        - 本方法会真实吸合/断开触点, 调用前必须确认负载安全;
        - 实测 4 路板在控制帧后约 50ms 内 FF 回读仍为旧状态, 所以 ``verify=True``
          通常会出现一次"判定失败 -> 同状态重发 -> 确认成功"; 这是预期行为,
          同路同状态重发幂等且方向安全, 不要为了少发一帧而把重发设为 0;
        - ``verify=False`` 只表示"控制帧已发出", 不能证明继电器真的动作了,
          需要确认时必须用 ``verify=True`` 或事后调用 ``query_status()``。
        """
        command = build_channel_command(channel, on, self.channel_count)
        state_text = "ON" if on else "OFF"
        attempts = 1 + max(0, int(retries))
        with self._lock:
            serial_obj = self._require_serial()
            for attempt in range(attempts):
                serial_obj.write(command)
                serial_obj.flush()
                LOG.info("[RELAY] 第%s路 -> %s (发送 %s)", channel, state_text, _hex(command))
                if not verify:
                    return True
                # 立即回读: 板子状态刷新有延迟时这里会读到旧状态, 由下面的重发兜底
                states = self._query_locked(serial_obj)
                if states is not None and states.get(channel) == bool(on):
                    LOG.info("[RELAY] 第%s路 %s 已确认(%s)", channel, state_text, format_states(states))
                    return True
                actual = "未返回该路状态" if not states else format_states(states)
                if attempt + 1 < attempts:
                    LOG.warning("[RELAY] 第%s路 %s 校验失败(%s), 重发第 %s 次", channel, state_text, actual, attempt + 2)
                else:
                    LOG.error("[RELAY] 第%s路 %s 校验失败(%s)", channel, state_text, actual)
            return False

    def turn_on(self, channel: int, verify: bool = False, retries: int = 1) -> bool:
        """打开一路继电器。"""
        return self.set_channel(channel, True, verify=verify, retries=retries)

    def turn_off(self, channel: int, verify: bool = False, retries: int = 1) -> bool:
        """关闭一路继电器。"""
        return self.set_channel(channel, False, verify=verify, retries=retries)

    def set_all(self, on: bool, verify: bool = False, retries: int = 1) -> bool:
        """依次设置全部通道。返回是否所有通道都成功。

        风险与边界:
        - 逐路调用 ``set_channel``, 只保证每路自身的原子性, 不保证跨路原子性:
          其他线程可能插入到两路之间, 所以不要把它当作一次"全板事务";
        - 中途失败**不会回滚**已动作的通道(会留下部分通道处于新状态),
          失败通道号会记入日志; 需要回到安全状态时请再调用 ``all_off()`` 并检查返回值。
        """
        state_text = "ON" if on else "OFF"
        failed: List[int] = []
        for channel in range(MIN_CHANNEL, self.channel_count + 1):
            if not self.set_channel(channel, on, verify=verify, retries=retries):
                failed.append(channel)
        if failed:
            LOG.error("[RELAY] 全部置 %s 失败通道: %s", state_text, failed)
            return False
        LOG.info("[RELAY] 全部 %s 路已置 %s", self.channel_count, state_text)
        return True

    def all_off(self, verify: bool = False, retries: int = 1) -> bool:
        """断开全部通道(安全方向)。结束程序前建议显式调用并检查返回值。"""
        return self.set_all(False, verify=verify, retries=retries)

    def all_on(self, verify: bool = False, retries: int = 1) -> bool:
        """闭合全部通道。

        风险: 会同时吸合所有通道, 请先确认负载、线径和电源余量;
        多路同时动作与保持时间均未在实机上验证, 不要用于关键动作。
        """
        return self.set_all(True, verify=verify, retries=retries)

    # ------------------------------------------------------------------ 查询接口

    def query_status(self, timeout: Optional[float] = None) -> Optional[Dict[int, bool]]:
        """发送 FF 查询全部通道状态 (只发查询帧, 不动触点)。

        return: ``{路号: True/False}``; 超时或没有任何有效返回时返回 None。

        风险与边界: 返回的是板子**状态快照**, 刚发完控制帧就查询可能拿到旧值
        (实测约 50ms 内滞后); 按 ``channel_count`` 配置的路数收不齐时只告警并返回已收到的部分,
        不会为缺失通道编造状态 —— 判定控制是否成功时不要只看返回值是否为 None。
        """
        with self._lock:
            serial_obj = self._require_serial()
            states = self._query_locked(serial_obj, timeout)
        if states is None:
            LOG.warning("[RELAY] FF 状态查询失败: 未收到有效返回")
        else:
            LOG.info("[RELAY] FF 状态查询: %s", format_states(states))
        return states

    def get_channel_state(self, channel: int, timeout: Optional[float] = None) -> Optional[bool]:
        """查询单路状态 (只发查询帧, 不动触点)。

        return: True 开 / False 关 / None 查询失败(或该路未出现在返回数据中)。

        边界: 同样受状态快照滞后影响; 反查失败时返回 None 而不是猜测,
        调用方需要自行区分"确定是关"和"没查到"。
        """
        _validate_channel(channel, self.channel_count)
        with self._lock:
            serial_obj = self._require_serial()
            states = self._query_locked(serial_obj, timeout)
        if states is None:
            LOG.warning("[RELAY] 第%s路状态查询失败: 未收到有效返回", channel)
            return None
        if channel not in states:
            LOG.warning("[RELAY] 第%s路状态未出现在返回数据中(%s)", channel, format_states(states))
            return None
        state = states[channel]
        LOG.info("[RELAY] 第%s路当前状态: %s", channel, "ON" if state else "OFF")
        return state

    def detect_channel_count(self, timeout: Optional[float] = None) -> Optional[int]:
        """发送 FF 识别板子实际报告的路数(例如 4 路板返回 4, 8 路板返回 8)。

        只读查询: 不发送任何控制帧, 不改变任何一路状态, 也不修改 ``self.channel_count``。
        识别结果由调用方决定怎么用, 例如 ``LCUSRelay(port=..., channel_count=detected)``。
        查询失败或没有任何有效返回时返回 None。

        边界:
        - 识别值 = FF 返回里出现的**最大路号**, 解析上限固定为 ``MAX_CHANNEL_COUNT``(8),
          因此按 4 路配置也能识别 8 路板; 但如果某路始终不返回, 识别值会偏小;
        - 识别只反映板子自述, 不代表每一路都接线正确或能带负载;
        - 用 8 路默认配置去查询 4 路板时, ``query_status()`` 会告警缺少通道 [5..8], 属预期行为,
          这时请用 ``LCUSRelay(..., channel_count=4)``。
        """
        with self._lock:
            serial_obj = self._require_serial()
            states = self._query_locked(
                serial_obj, timeout, channel_limit=MAX_CHANNEL_COUNT, warn_missing=False
            )
        if not states:
            LOG.warning("[RELAY] 路数识别失败: 未收到有效返回")
            return None
        detected = max(states)
        LOG.info("[RELAY] 检测到 %s 路继电器(%s)", detected, format_states(states))
        return detected

    def _query_locked(
        self,
        serial_obj,
        timeout: Optional[float] = None,
        channel_limit: Optional[int] = None,
        warn_missing: bool = True,
    ) -> Optional[Dict[int, bool]]:
        """在已持锁的前提下发送 FF 并收集返回。调用方必须已持有 self._lock。

        channel_limit: 解析与“收齐”判定的路数上限, 默认 ``self.channel_count``;
        识别路数时用 ``MAX_CHANNEL_COUNT``, 这样即使按 4 路配置也能识别到 8 路板。
        warn_missing: 是否对缺少的通道打告警(识别路数时为 False)。
        """
        limit = self.channel_count if channel_limit is None else int(channel_limit)
        serial_obj.reset_input_buffer()
        deadline = time.perf_counter() + (self._query_timeout if timeout is None else float(timeout))
        serial_obj.write(bytes((QUERY_BYTE,)))
        serial_obj.flush()
        LOG.debug("[RELAY] 发送状态查询 %s", _hex(bytes((QUERY_BYTE,))))

        buffer = b""
        states: Dict[int, bool] = {}
        while time.perf_counter() < deadline:
            waiting = serial_obj.in_waiting
            chunk = serial_obj.read(waiting) if waiting else serial_obj.read(1)
            if not chunk:
                continue
            buffer += chunk
            states = parse_status_response(buffer, limit)
            if len(states) >= limit:
                break
        if not states:
            return None
        missing = [c for c in range(MIN_CHANNEL, limit + 1) if c not in states]
        if missing and warn_missing:
            LOG.warning("[RELAY] FF 状态查询缺少通道: %s", missing)
        LOG.debug("[RELAY] FF 原始返回: %r", buffer)
        return states


class FakeLCUSRelay:
    """内存继电器（dry-run / 回放 / 单元测试用）: 不打开任何串口。

    与 ``LCUSRelay`` 保持同名方法, 便于运行时统一调用; 状态在内存中立即生效, 因此
    ``verify=True`` 恒成立。``commands`` 记录发出的控制帧(十六进制可在测试里比对),
    ``off_requests`` 记录 ``all_off()`` 次数, ``close_count`` 记录 ``close()`` 次数。
    """

    def __init__(self, channel_count: int = DEFAULT_CHANNEL_COUNT, *, initial_on: bool = False) -> None:
        self.channel_count = _validate_channel_count(channel_count)
        self._states: Dict[int, bool] = {
            channel: bool(initial_on) for channel in range(MIN_CHANNEL, self.channel_count + 1)
        }
        self.commands: List[bytes] = []
        self.off_requests = 0
        self.open_count = 0
        self.close_count = 0
        self._is_open = False

    @property
    def port(self) -> str:
        return "fake"

    @property
    def baudrate(self) -> int:
        return DEFAULT_BAUDRATE

    @property
    def connected(self) -> bool:
        return self._is_open

    def open(self) -> None:
        if not self._is_open:
            self._is_open = True
            self.open_count += 1

    def close(self) -> None:
        if self._is_open:
            self._is_open = False
            self.close_count += 1

    def __enter__(self) -> "FakeLCUSRelay":
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.close()
        return False

    def _require_open(self) -> None:
        if not self._is_open:
            raise RuntimeError("模拟 LCUS 继电器未打开, 请先调用 open()")

    def set_channel(self, channel: int, on: bool, verify: bool = False, retries: int = 1) -> bool:
        _validate_channel(channel, self.channel_count)
        self._require_open()
        self.commands.append(build_channel_command(channel, on, self.channel_count))
        self._states[channel] = bool(on)
        return True

    def turn_on(self, channel: int, verify: bool = False, retries: int = 1) -> bool:
        return self.set_channel(channel, True, verify=verify, retries=retries)

    def turn_off(self, channel: int, verify: bool = False, retries: int = 1) -> bool:
        return self.set_channel(channel, False, verify=verify, retries=retries)

    def set_all(self, on: bool, verify: bool = False, retries: int = 1) -> bool:
        for channel in range(MIN_CHANNEL, self.channel_count + 1):
            self.set_channel(channel, on, verify=verify, retries=retries)
        return True

    def all_off(self, verify: bool = False, retries: int = 1) -> bool:
        self.off_requests += 1
        return self.set_all(False, verify=verify, retries=retries)

    def all_on(self, verify: bool = False, retries: int = 1) -> bool:
        return self.set_all(True, verify=verify, retries=retries)

    def query_status(self, timeout: Optional[float] = None) -> Optional[Dict[int, bool]]:
        self._require_open()
        return dict(self._states)

    def get_channel_state(self, channel: int, timeout: Optional[float] = None) -> Optional[bool]:
        _validate_channel(channel, self.channel_count)
        self._require_open()
        return self._states[channel]

    def detect_channel_count(self, timeout: Optional[float] = None) -> Optional[int]:
        self._require_open()
        return self.channel_count
