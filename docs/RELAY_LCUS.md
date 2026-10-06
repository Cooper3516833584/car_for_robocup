# LCUS USB 继电器（relay_lcus）移植与实车检查清单

本页是独立交付的 `relay_lcus.py` + `relay_lcus_porting_checklist.md` 在 RoboCup 差速车仓库中的
落地说明与实车联调清单。驱动本体、控制帧和异常语义与交付版本一致；本文档同时给出**本仓库的接入点**
和第 1.1 → 1.12 的**实车逐项检查**。

组件位置：`code/components/relay_lcus.py`；自检工具：`code/test/relay_selftest.py`。

---

## 0. 交付内容与已知前提

### 0.1 仓库内文件

| 文件 | 说明 |
|---|---|
| `code/components/relay_lcus.py` | 驱动本体。单文件、协议与交付版本一致；只依赖 `pyserial`（惰性导入） |
| `code/test/relay_selftest.py` | 硬件联调 CLI（默认只读，不吸合触点；不进入单元测试发现） |
| `code/test/test_relay_lcus.py` | 30 项纯逻辑单测（注入假串口，不开真实串口） |
| `code/test/test_relay_runtime.py` | 11 项接入/关停单测（dry-run 用内存继电器） |
| `code/config/v2_models.py`、`v2_loader.py`、`v2_factory.py` | `[devices.relay]` 配置、校验与 `build_relay()` |
| `code/robocup_runtime.py` | 启动时开端口、退出时先 `all_off()` 再关端口 |
| `configs/robocup_diffdrive.example.toml` | 示例 `[devices.relay]`（默认 `enabled = false`） |

### 0.2 移植时的改动（仅仓库适配，协议未改）

1. 日志由 `loguru` 改为标准库 `logging`（本仓库统一约定），因此唯一第三方依赖是 `pyserial`；
   没有 `pyserial` 也能 import 本模块并使用 `build_channel_command()` / `parse_status_response()` 等纯函数。
2. `LCUSRelay(serial_factory=...)`：新增串口工厂注入点，供单测和离线联调使用；正式运行保持 `None`。
3. 新增 `FakeLCUSRelay`：dry-run / 回放使用的内存继电器，不打开任何串口。
4. 端口等参数正式运行时来自 TOML 的 `[devices.relay]`（组件本身不读 TOML）；
   单文件独立使用时仍可显式传 `port` 或设置环境变量 `D_TASK_RELAY_PORT`。

### 0.3 协议与硬件参数（不要改）

- 串口参数：**9600 8N1**，DTR/RTS 显式拉低（`rtscts=False`、`dsrdtr=False`）。
- 控制帧固定 4 字节：`A0 | 路号 | 状态 | 校验`
  - 路号 `01`~`08` = 第 1~8 路；
  - 状态 `01` = 开，`00` = 关；
  - 校验 = 前三字节按字节求和取低 8 位。
- 控制帧**没有应答**；要确认结果必须发查询帧 `FF` 回读。
- 查询返回每路 10 字节 ASCII 文本，开/关等长：
  - `CH1: ON \r\n`（`ON` 后有 1 个空格补齐 10 字节）、`CH1: OFF\r\n`
  - 4 路板返回 40 字节，8 路板返回 80 字节。
- 返回格式不唯一：同协议的 8 路板实测返回**二进制**（连续 `channel_count` 个字节，每字节一路，
  `0x01`=开 `0x00`=关）。`parse_status_response()` 两种都识别，本项目 4 路板实测为 ASCII；
  换回 8 路板时按板子把 `channel_count` 设成 8 即可，解析代码不用改。

指令帧速查表（可直接在 SSCOM32 / minicom 里手工核对）：

| 路号 | 打开 | 关闭 |
|---|---|---|
| CH1 | `A0 01 01 A2` | `A0 01 00 A1` |
| CH2 | `A0 02 01 A3` | `A0 02 00 A2` |
| CH3 | `A0 03 01 A4` | `A0 03 00 A3` |
| CH4 | `A0 04 01 A5` | `A0 04 00 A4` |
| CH5 | `A0 05 01 A6` | `A0 05 00 A5` |
| CH6 | `A0 06 01 A7` | `A0 06 00 A6` |
| CH7 | `A0 07 01 A8` | `A0 07 00 A7` |
| CH8 | `A0 08 01 A9` | `A0 08 00 A8` |
| 查询 | — | `FF` |

也可以直接打印这张表（不开串口、不动触点）：

```bash
python3 code/test/relay_selftest.py --print-frames --channels 4
```

### 0.4 配置（`configs/robocup_diffdrive.example.toml`）

```toml
[devices.relay]
enabled = false              # 默认关闭: 装上板子并固定端口后再打开
port = ""                    # 见下方"本机实测端口"
baudrate = 9600
channel_count = 4            # 本项目装的是 4 路板; 8 路板写 8
read_timeout_s = 0.2
query_timeout_s = 1.0
verify_writes = true         # 每次动作后 FF 回读确认(推荐保持 true)
disconnect_on_shutdown = true
```

**本机实测端口（ROCK 5A，2026-10-06 通过 SSH 只读确认）**

- 板子：**4 路 LCUS**，CH340（`1a86:7523`，`ID_USB_DRIVER=ch341`）；
- 节点：`/dev/ttyUSB0`（`lsusb` Bus 008 Device 002，USB 树 `platform-fc8c0000.usb` / `usb8/8-1`）；
- udev 稳定路径：`/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0`
  （**推荐用这个**，不要用会随插拔变号的 `/dev/ttyUSB0`）；
- 只发 `FF` 查询帧得到 40 字节 ASCII：`CH1: OFF\r\nCH2: OFF\r\nCH3: OFF\r\nCH4: OFF\r\n`
  → 确认这就是 LCUS 继电器、4 路、当前全部断开（未发送任何控制帧）；
- 该端口当前**没有**任何进程占用（C10B 在 `/dev/ttyACM0`，HC-15 电台在 `/dev/ttyS4`，两者都不是继电器）。

要把它固定成 `/dev/relay_lcus` 还是直接用 by-path，二者选一：

```toml
port = "/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0"
# 或者先按 §1.8 建 udev 软链接, 再写:
# port = "/dev/relay_lcus"
```

`[devices.relay]` 是**可选**表：老配置不写这一段仍能加载，等价于 `enabled = false`。
`enabled = true` 时 `port` 不能为空，`channel_count` 必须在 1~8。

### 0.5 本仓库运行时行为

- `build_relay()` / `build_runtime()` **不打开**任何串口；
- `RobocupRuntime.start()` 打开串口（失败会走既有的启动失败路径：停底盘、停传感器、释放继电器后上抛）；
- `RobocupRuntime.close()`（正常结束、异常、SIGINT 都会走到）先停底盘与传感器，再
  `all_off(verify=verify_writes)`，然后关端口，并写 `relay_shutdown` 事件；
  若 `all_off` 返回 False 或抛异常，只记录 ERROR/WARNING，**不会**掩盖主流程异常，但**触点可能仍然吸合**；
- dry-run / replay 使用 `FakeLCUSRelay`，只记录命令，不接触任何设备；
- 事件：`relay_ready`（启动）、`relay_shutdown`（关停，含 `released` 字段）。

### 0.6 两条已知实机行为（不要当成故障）

1. **首次回读滞后已被 `verify_settle` 覆盖**：4 路板实测，发控制帧后约 **50ms** 内 `FF` 回读仍返回旧状态。
   因此 `verify=True` 现在会**先等待 `DEFAULT_VERIFY_SETTLE`（0.1s）再回读**，正常情况下首次回读即一致，
   不再出现「第一次判定失败 → 同状态重发 → 第二次确认成功」。重发机制保留为兜底：
   板子更慢或首帧真的丢失时仍会重发，同路同状态重发**幂等且方向安全**，**不要把 `retries` 设成 0**。
   每次带校验的开关因此比"立即回读"多约 0.1s 耗时，这是刻意的稳定性取舍；需要旧行为可在构造时传
   `verify_settle=0`。若仍频繁看到重发，说明该块板子的状态刷新比 100ms 更慢。
2. **继电器断电保持**：`close()` 和进程退出都**不会**断开已吸合的触点。需要断开必须显式 `all_off()`。
   （USB 断电重插后板子会复位为全 OFF，但**不要把它当安全手段**。）

---

## 1. 实车检查清单（1.1 → 1.12）

标记约定：`$` 普通用户可执行；`#` 需要 root 或需要事先获得设备所有者授权（会改系统配置）。未标注的都是只读命令。

### 1.1 Python 版本

**检查**

```bash
python3 -V
```

**判定**：`Python 3.7.x` 或更高（驱动用了 f-string 和 `from __future__ import annotations`）。
本仓库的 v2 配置在 Python 3.11+ 用标准库 `tomllib`，3.10 需要 `tomli`。

**对应操作**：低于要求时安装新解释器并用新版本调用；**不要**为兼容旧版本改写驱动。

### 1.2 依赖是否齐

**检查**

```bash
python3 -c "import serial; print(serial.__version__)"
python3 -m compileall -q code/test/relay_selftest.py
```

**判定**：打印版本号（建议 `>=3.4`，3.5 实测可用），且自检脚本能编译。

**对应操作**

- 有 pip：`$ python3 -m pip install --user "pyserial>=3.4"`
- 离线板子：用离线 wheel（`pip download` 在有网机器上拉好后拷入）安装。
- 本仓库已不使用 `loguru`，无需安装。

### 1.3 CH340 内核驱动是否存在

**检查**（先插上继电器）

```bash
lsmod | grep -E 'ch341|usbserial'
dmesg | tail -20          # 无权限时用 sudo dmesg
```

**判定**：能看到 `ch341`；`dmesg` 中出现类似
`ch341-uart 1-1.2:1.0: ch341-uart converter now attached to ttyUSB0`。

**对应操作**：没有 `ch341`（ARM 厂商 BSP 内核常见）需打开 `CONFIG_USB_SERIAL_CH341`（`=m` 或 `=y`）后重编内核或换内核；有 `ch341` 但报错时检查供电与线材，换 USB 口重试。

### 1.4 设备节点是否出现

**检查**

```bash
ls -l /dev/ttyUSB* 2>/dev/null
ls -l /dev/serial/by-id/ /dev/serial/by-path/ 2>/dev/null
```

**判定**：插拔继电器时，`/dev/ttyUSBx` 会相应地出现/消失。

**对应操作**：没有新节点时回到 1.3，并检查 USB 口供电（部分开发板单口带不动继电器板）；出现多个 `ttyUSBx` 时记下插拔前后**新增的那一个**，并在 1.8 做稳定命名。

### 1.5 串口权限

**检查**

```bash
ls -l /dev/ttyUSB0
id -nG | tr ' ' '\n' | grep -E 'dialout|uucp'
```

**判定**：当前用户在 `dialout`（Debian/Ubuntu）或 `uucp`（Arch 等）组内，或设备权限允许当前用户读写。

**对应操作**（系统配置修改，需先获得授权）

```bash
# 一次性加入组（需要重新登录或 newgrp 生效）
# sudo usermod -aG dialout $USER
```

长期方案见 1.8 的 udev 规则（`GROUP="dialout", MODE="0660"`）。

### 1.6 端口是否被别的进程占用

**检查**

```bash
fuser -v /dev/ttyUSB0 2>/dev/null
lsof /dev/ttyUSB0 2>/dev/null
```

**判定**：没有其他进程占用（正在运行本驱动时看到自己的 pid 属正常）。

**对应操作**：被 `ser2net`/`socat`/采集程序/`ModemManager` 占用时停掉对应服务，或换一个 USB 口。

### 1.7 brltty / ModemManager 抢设备（Ubuntu 22.04+ 常见）

**检查**

```bash
dpkg -l | grep -i brltty
systemctl status brltty --no-pager
systemctl status ModemManager --no-pager
```

**判定**：`brltty` 未安装或未运行；`ModemManager` 没有反复探测该串口（`dmesg` 里没有周期性探测日志）。

**对应操作**（系统配置修改，需先获得授权）

```bash
# sudo apt remove brltty                     # brltty 抢 CH340 时
# udev 规则中给该设备加: ENV{ID_MM_DEVICE_IGNORE}="1"
# sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 1.8 稳定端口路径（重要，别跳过）

**检查**

```bash
ls -l /dev/serial/by-id/ /dev/serial/by-path/
udevadm info -a -n /dev/ttyUSB0 | grep -E 'KERNELS|idVendor|idProduct|serial'
```

**判定**

- 板子上**只有**一个 USB 串口设备时，`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` 这类路径可用。
- 板子上**还有别的 CH340**（例如 HC-14 电台，USB ID 同为 `1a86:7523`）时，`by-id` **不可靠**：无序列号的 CH340 可能同名或互相覆盖，`/dev/ttyUSB*` 编号也会随枚举顺序变化。

**本机实测（ROCK 5A，2026-10-06）**

```text
/dev/ttyUSB0                                          -> CH340, LCUS 4 路继电器
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0      -> ../../ttyUSB0   (无序列号, 换板/换口会变)
/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0 -> ../../ttyUSB0   ← 用这个
```

`ID_PATH=platform-fc8c0000.usb-usb-0:1:1.0`，USB 树 `platform/fc8c0000.usb/usb8/8-1/8-1:1.0`。
同一台机器上 C10B 是 `/dev/ttyACM0`（`usb-1a86_USB_Single_Serial_...`），HC-15 电台走 `/dev/ttyS4`，
都和继电器无关，**不要**把它们当成继电器端口。

**两种收尾方式，任选一种**

1. 直接用 by-path（不需要 root，不改系统配置）：

```toml
port = "/dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0"
```

2. 或者建一条 udev 软链接，把端口固定成 `/dev/relay_lcus`（需要 root 授权；`KERNELS` 用上面实测的 `8-1`）：

```udev
# /etc/udev/rules.d/99-lcus-relay.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", KERNELS=="8-1", \
  SYMLINK+="relay_lcus", GROUP="dialout", MODE="0660"
```

```bash
# sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/relay_lcus          # 应指向 /dev/ttyUSB0
```

之后把 `port = "/dev/relay_lcus"` 填进 `[devices.relay]`。

> 规则：**不要**按 VID/PID 自动猜端口（继电器和 HC-14 同 ID），也不要写死易变的 `/dev/ttyUSB0`。

### 1.9 只读自检（不发控制帧，不会吸合继电器）

**检查**（与驱动同仓库，需先装 `pyserial`）

```bash
# 先确认这个串口就是 LCUS(只发一次 FF, 打印原始返回):
python3 code/test/relay_selftest.py --port /dev/ttyUSB0 --identify
# 再读路数和状态:
python3 code/test/relay_selftest.py --port /dev/relay_lcus
# 也可以用配置里的端口（需要该 profile 里 [devices.relay] 已 enabled = true 且填了 port）:
python3 code/test/relay_selftest.py --config configs/<你的车辆 profile>.toml
```

**期望输出**

```
detected: 4
states  : CH1=OFF CH2=OFF CH3=OFF CH4=OFF
```

（`detected` 是板子实际路数：4 路板为 4，8 路板为 8。本机 2026-10-06 只读实测就是这组输出。）

**判定**：能识别路数且状态可读 → 波特率、协议、接线都正确。

**对应操作**

- **完全无返回**：先用 `--identify` 看原始返回；只有 LCUS 会对 `FF` 回 `CHn: ...` 文本或二进制快照，
  再回头查 1.3~1.6。
- **收到乱码或只解析到部分行**：确认波特率是 9600（不是 115200）；确认没有别的进程在读写同一端口。
- **识别到 4 路但手上是 8 路板**：确认是否插错板子；确认后把 `channel_count` 改成识别到的值（用 8 路配置查询 4 路板只会告警 `缺少通道 [5,6,7,8]`，属预期）。

### 1.10 单路动作验证（**会真实吸合**，先空载确认）

**检查**

```bash
python3 code/test/relay_selftest.py --port /dev/relay_lcus --channel 1 --on     # 吸合第 1 路
python3 code/test/relay_selftest.py --port /dev/relay_lcus --channel 1 --off    # 断开第 1 路
```

**判定**：输出 `set_channel -> OK`，且继电器有吸合声/指示灯变化、`states` 中对应路变 `True`。

**注意**：正常情况下**只发一帧并在 0.1s 后回读到新状态**（§0.6）。若先出现一次
`校验失败, 重发第 2 次` 再成功，说明这块板子的状态刷新比 `DEFAULT_VERIFY_SETTLE` 更慢，
属可接受但需留意的行为，不是接线故障。

**对应操作**

- **一直校验失败**：听/看有没有吸合动作。
  - 有动作但回读一直是旧值 → 把 `query_timeout` 调大（如 `--query-timeout 2`）后重试；
  - 完全无动作 → 核对路号映射（换一路试）、确认继电器板供电与负载电源。
- **动作后程序退出，触点仍保持** → 正常（§0.6），离开前务必执行 `--all-off` 并复核状态。
- **反复开/关同一路做联调**：用 `--blink <CH> --interval <s> --repeat <n>`（会反复吸合，注意负载与机械寿命）。

### 1.11 时序与阻塞

**检查**：确认调用点没有在**高频控制回环**里同步调用 `query_status()` / `set_channel(verify=True)`。
本仓库的差速控制回环（`robocup_runtime.step()`）**不调用**继电器，仅在启动/关停时访问，因此不占用控制周期。

**判定**

- `query_timeout_s`（默认 1.0s）小于调用侧可接受的阻塞时间；
- 参考耗时：9600 波特率下 4 字节控制帧约 4.2ms，4 路回包（40 字节）约 42ms、8 路回包（80 字节）约 84ms；
  每次 `set_channel(verify=True)` 还要加上 `DEFAULT_VERIFY_SETTLE`（0.1s）稳定等待，
  4 路 `all_off(verify=True)` 因此最坏约 4 ×（4.2ms + 0.1s + 42ms）≈ 0.6s；
- 查询期间驱动持有内部锁，不要在其他锁里嵌套调用。

**对应操作**：实时任务里把 `query_timeout_s` 调小（例如 0.3）；把查询/校验放到独立线程或低频步骤里。

### 1.12 供电与多路同时吸合

**检查**：确认 `all_on()` 的使用场合、USB 供电方式、负载侧接线。

**判定**：多路同时吸合时板子不掉线、不复位。

**对应操作**

- 用**带供电的 HUB** 或外部 5V 给继电器板供电；
- 用逐路分时动作代替 `all_on()`；
- 负载侧电流/电压按继电器触点容量接线。

---

## 2. 环境变量与自启动

- 端口来源优先级（正式运行时）：`[devices.relay] port` > `LCUSRelay(port=...)` 显式参数 > 环境变量 `D_TASK_RELAY_PORT`。
- **systemd 不读 shell 环境变量**，单文件独立使用时要在 unit 里显式写：

```ini
Environment=D_TASK_RELAY_PORT=/dev/relay_lcus
```

- 常驻/自启动程序必须在退出路径（含异常与信号处理）上显式 `all_off()`：继电器断电保持，进程退出不等于断开。
  本仓库的 `RobocupRuntime.close()` 已经覆盖正常结束、异常与 SIGINT；自定义脚本请自行处理。
- 只读 rootfs 上运行：用单文件方式（`LCUSRelay` 不写任何文件）。

---

## 3. 自检脚本

`code/test/relay_selftest.py`（默认只读）：

```bash
python3 code/test/relay_selftest.py --list                                     # 只读: 枚举串口, 不开继电器
python3 code/test/relay_selftest.py --print-frames --channels 4                # 只读: 打印指令帧
python3 code/test/relay_selftest.py --port /dev/ttyUSB0 --identify             # 只读: 确认端口是 LCUS
python3 code/test/relay_selftest.py --port /dev/relay_lcus                     # 只读: 识别路数 + 查询状态
python3 code/test/relay_selftest.py --port /dev/relay_lcus --channel 1 --on    # 会真实吸合!
python3 code/test/relay_selftest.py --port /dev/relay_lcus --channel 1 --off
python3 code/test/relay_selftest.py --port /dev/relay_lcus --all-off
python3 code/test/relay_selftest.py --port /dev/relay_lcus --blink 2 --interval 1 --repeat 3
python3 code/test/relay_selftest.py --config configs/<你的车辆 profile>.toml
```

（`configs/robocup_diffdrive.example.toml` 里 `[devices.relay] enabled = false`，直接用 `--config` 会提示未启用；
先按 §1.8 固定端口再打开它。）

退出码：`0` 成功；`1` 无有效返回或带校验的动作失败；`2` 参数错误。
带校验的开关动作输出示例（现已只需一帧，不再有"重发后确认"）：

```
detected: 4
states  : CH1=OFF CH2=OFF CH3=OFF CH4=OFF
set_channel -> OK
states  : CH1=ON CH2=OFF CH3=OFF CH4=OFF
```

---

## 4. 常见故障对照表

| 现象 | 可能原因 | 对应操作 |
|---|---|---|
| `ModuleNotFoundError: No module named 'serial'` | 未装 pyserial | 见 1.2 |
| `RuntimeError: 未安装 pyserial, 无法打开 LCUS 继电器串口` | 驱动惰性导入 pyserial 失败 | 见 1.2 |
| `SerialException: could not open port ... Permission denied` | 权限不足 | 见 1.5、1.6 |
| `SerialException: [Errno 2] could not open port /dev/relay_lcus` | 设备节点不存在或编号变了 | 见 1.3、1.4、1.8 |
| 端口存在但一会儿就消失 | brltty / ModemManager 抢占，或供电不足 | 见 1.7、1.12 |
| `FF` 查询完全无返回 | 端口不是继电器、波特率不对、板子没供电 | 见 1.9 |
| 查询返回 `缺少通道: [5, 6, 7, 8]` | 用 8 路配置查 4 路板 | 正常告警；改用 `channel_count = 4` |
| 每次带校验的开关都"失败一次再重发" | 板子状态刷新慢于 `DEFAULT_VERIFY_SETTLE`(0.1s) | 保持 `retries >= 1`；确需固定等待可构造时传更大的 `verify_settle` |
| 校验一直失败但继电器有吸合声 | 回读太早 / 板子固件差异 | 调大 `query_timeout_s` 与 `retries`，并用串口工具手工核对 `FF` 返回格式 |
| 多路同时吸合时板子掉线复位 | USB 供电不足 | 见 1.12 |
| 程序退出后负载仍带电 | 继电器断电保持（§0.6） | 退出前显式 `all_off()`；本仓库看 `relay_shutdown` 事件的 `released` 字段 |
| `disconnect_on_shutdown` 为 true 但事件里 `released=false` | 回读未确认或端口已失效 | 现场必须人工确认触点状态，不要只看程序是否退出 |
| 运行中 USB 串口掉线（写入报 `SerialException`/`PermissionError`） | 供电/线材/USB 口不稳（参考工程在另一块板上实测过） | 见 §5；写入失败会直接上抛，必须人工确认触点状态 |

---

## 5. 安全边界（移植后须知）

- 驱动会**真实吸合/断开触点**，且**没有任何硬件互锁**；不经过 C10B 底盘链路，因此不受底盘看门狗、
  ACK/重试/心跳保护。**不要把它当作安全关键回路**，急停与安全兜底由任务层负责。
- 端口**不得按 VID/PID 猜**：CH340 与其他设备（如 HC-14 电台）USB ID 相同（`1a86:7523`），
  必须显式指定 `port` 或 `D_TASK_RELAY_PORT`。
- 串口读写故障（`serial.SerialException` 等）会**直接上抛**，驱动不吞异常、也不做安全兜底；
  参数错误抛 `ValueError`，未 `open()` 抛 `RuntimeError`。
- `open()`/`close()`/`query_status()`/`get_channel_state()`/`read_raw_response()`/`detect_channel_count()`
  **只收发查询帧**，不动触点；
  只有 `set_channel()`/`turn_on()`/`turn_off()`/`set_all()`/`all_on()`/`all_off()` 会动触点。
- `set_channel(verify=False)` 只表示"控制帧已发出"，**不能**证明继电器真的动作了；
  需要确认必须用 `verify=True` 或事后 `query_status()`。
- `set_all()` 是逐路加锁，**跨路不原子**；中途失败**不回滚**已动作的通道，失败通道号在日志里。
- DTR/RTS 被显式拉低：个别继电器板可能把这两根线接到其他电路，副作用未验证。
- 已验证范围（本项目当前状态）：装着**一块 4 路 LCUS 板**（CH340，9600 8N1），**该板没有损坏通道**，
  因此不需要像参考工程那样避开某些路号；2026-10-06 在该 ROCK 5A 上只读实测确认了端口
  `/dev/ttyUSB0`（by-path `platform-fc8c0000.usb-usb-0:1:1.0-port0`）、路数识别与四路全 `OFF` 状态
  （§0.4、§1.9），**动作类验证（吸合/断开）尚未在本机执行**。
  仍**未验证**的项目：8 路板与第 5~8 路映射、多路同时吸合、长时间稳定性、运行中拔插 USB。
  本仓库侧的 dry-run、单元测试和关停路径已在软件层验证（见 §6），不代表实机验收。
- 参考工程在同协议的另一块 8 路板上记录过两类硬件风险，本板**没有**这些现象，但使用前应当知道：
  1. **触点/接线侧损坏**：有两路出现"软件开关帧与 FF 回读都正常、外接电路却不动作"，属板子/负载侧故障；
     用某一通道驱动实际负载前，先空载确认该路真的动作。
  2. **USB 串口运行中掉线**：曾出现写入时 `SerialException: WriteFile failed`、端口从系统消失。
     掉线时软件**无法补发关闭帧**，若板子仍带电，已吸合的通道会**保持吸合**；因此不要让通道长时间
     无人照看地保持吸合，出现写入失败要立即上报并转为人工处置（现场确认触点状态）。
- 部署到本仓库遵守既有规则：本地修改 → 提交 → 设备端 `git pull --ff-only`，不要在设备上直接改仓库文件。

---

## 6. 本仓库侧验证（不需要硬件）

```powershell
py -3 -m compileall -q code tools
py -3 -m unittest discover -s code/test -p "test_relay_lcus.py"
py -3 -m unittest discover -s code/test -p "test_relay_runtime.py"
py -3 -m unittest discover -s code/test -p "test_config_v2.py"
py -3 code\main_robocup.py --config configs\robocup_diffdrive.example.toml --mode dry-run --log-dir logs\dry-run
```

覆盖范围：

- 8 路开/关指令帧与校验和、默认 4 路配置下第 5~8 路被拒绝、非法路号/路数拒绝；
- 分片、噪声、大小写、缺通道返回的解析容忍度，ASCII 与二进制两种 FF 返回格式，
  以及"分片/噪声不会被误判成二进制快照"；
- `verify=True` 先等待 `verify_settle` 再回读（首次回读即一致、不再多发帧），`verify_settle=0` 时不等待，
  `verify=False` 时不等待也不回读；`verify_settle` 非法值被拒绝；
- `read_raw_response()` 只发 FF、返回板子原始字节，空返回/串口故障/未 `open()` 的行为都被覆盖；
- `relay_selftest.py --identify` 只读路径（只调用一次 `read_raw_response()`，失败时退出码 1）；
- 最坏情况下的"首次滞后 → 幂等重发 → 确认"路径、重发耗尽后返回 False（失败关闭）；
- 未 `open()` 抛 `RuntimeError`、缺 `pyserial` 抛 `RuntimeError`、环境变量端口解析；
- `[devices.relay]` 默认关闭、可选表、默认 `channel_count = 4`（显式 8 仍可）、非法 `channel_count`、未知键、未知设备表；
- `build_relay()` 传递配置的路数与驱动默认 `verify_settle`；
- dry-run/replay 用内存继电器（不开端口）、启动失败时释放继电器、关停时先停底盘再 `all_off` 再关端口、
  未确认断开不会破坏 `close()`。

---

## 7. 接口速查

```python
from components.relay_lcus import LCUSRelay, FakeLCUSRelay

relay = LCUSRelay(port="/dev/relay_lcus", baudrate=9600, channel_count=4,
                  timeout=0.2, query_timeout=1.0)   # channel_count 默认 4, 8 路板传 8

relay.open()                                # 只开串口, 不动触点
relay.set_channel(1, True, verify=True)     # 开第 1 路 + 先稳定 0.1s 再 FF 回读确认, 返回 bool
relay.turn_on(2, verify=True)               # 开第 2 路
relay.turn_off(2, verify=True)              # 关第 2 路
relay.all_off(verify=True)                  # 断开全部(安全方向)
relay.set_all(True, verify=True)            # 闭合全部(注意供电)
relay.query_status()                        # {路号: True/False}, 失败返回 None
relay.get_channel_state(3)                  # True / False / None
relay.read_raw_response()                   # 只发 FF 并返回原始字节, 用于确认端口是 LCUS 继电器
relay.detect_channel_count()                # 识别板子实际路数(只读), 失败返回 None
relay.verify_settle                         # 带校验动作的稳定等待秒数(默认 DEFAULT_VERIFY_SETTLE=0.1)
relay.close()                               # 只关串口, 不断开触点
```

支持 `with LCUSRelay(port=...) as relay:` 上下文管理（进入 `open()`、退出 `close()`）。
构造参数：`port`、`baudrate`、`channel_count`（默认 `DEFAULT_CHANNEL_COUNT = 4`）、`timeout`、
`query_timeout`、`verify_settle`（默认 `DEFAULT_VERIFY_SETTLE = 0.1`，传 0 关闭等待）、
`serial_factory`（仅测试注入）。
模块级纯函数：`build_channel_command(channel, on, channel_count=4)`、
`parse_status_response(data, channel_count=4)`、`format_states(states)`、
`list_serial_ports()`、`format_port_list()`、`resolve_relay_settings(port, baudrate)`。
配置构造：`from config.v2_factory import build_relay`（`fake=True` 返回 `FakeLCUSRelay`）。
环境变量：`D_TASK_RELAY_PORT`。日志：统一 `[RELAY]` 前缀，控制帧以十六进制打印，便于和串口工具对照。

---

## 8. 可选改造

1. **零第三方依赖**：`loguru` 已在移植时去掉，目前只依赖 `pyserial`（惰性导入）。
2. **独占打开串口**：在 `_new_serial()` 的 pyserial 分支加 `exclusive=True`（POSIX `TIOCEXCL`），
   可避免两个进程同时打开同一端口导致数据串台。
3. **降低 USB 延迟**（非正确性前提）：

```bash
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer      # 常见默认 16ms
# echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

4. **控制帧后的稳定等待（已实现）**：`set_channel(verify=True)` 现在写完控制帧后先等待
   `verify_settle`（默认 `DEFAULT_VERIFY_SETTLE = 0.1s`）再发 `FF` 回读，正常情况下不再出现
   "假失败 + 多余重发"；重发仍作为兜底保留。该值目前**不是** TOML 项：它是与硬件时序绑定的常量，
   不是每辆车都不同的部署参数，需要调整时改常量或构造时传 `verify_settle=`（测试里传 0 即为旧行为）。
5. **任务层动作**：比赛任务目前**没有**调用继电器。需要把某一路接入载荷动作时，在
   `robocup_runtime.py` 中通过 `self.relay` 调用（例如 `self.relay.turn_on(channel, verify=True)`），
   并确保动作失败（返回 False）不改变底盘的安全停策略。
