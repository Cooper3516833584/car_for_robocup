# ROCK 5A 当前车辆接线（当前实车验证）

本文档只记录**当前这辆车**（Cooper ROCK 5A + WHEELTEC L150）的验证过接线。
换主控板请参考 `docs/HARDWARE_PORTING.md`；这些信息**不会**再出现在业务代码
的报错提示里。

## 差速底盘

旧前轮转向舵机及 PWM 部署脚本已移除。现行几何和传感器外参见
`docs/HARDWARE_MEASUREMENTS.md`，由 schema-v2 profile 配置。

## PWM 舵机（MG90S）

单轴 PWM 舵机，用于载荷指向，**不是**旧的阿克曼前轮转向。详细接线、overlay
启用步骤和实车命令见 `docs/SERVO_PWM7_M0.md`。

| 项目 | 值 |
|---|---|
| 物理 Pin | 28 |
| 引脚功能 | `PWM7_IR_M0`（`GPIO0_D0`） |
| 设备树节点 | `febd0030.pwm` |
| 设备树 overlay | `rk3588-pwm7-m0`（板端默认未启用，需追加后重启） |
| 舵机 | Tower Pro MG90S，50 Hz，±90 度 ↔ 500~2500 us |
| 0 度 | 中位脉宽 1500 us |
| 供电 | 独立 5~6 V BEC + 共地；**不要**用排针 5V |
| 权限 | sysfs 通道属 `pwm` 组，组内用户无需 sudo |
| 静止位 | 默认保持使能（舵机松手后没有保持力矩） |
| 配置节 | `[devices.servo]` |

`rk3588-pwm0-m2.dtbo` 是为 D500 的 `UART6-M1` 一起加载的旧 overlay，与舵机无关：
Pin 28 需要**额外**追加 `rk3588-pwm7-m0.dtbo`，不要移除已有 overlay。

## D500 雷达

| 项目 | 值 |
|---|---|
| D500 `TX` | 物理 Pin 21（`UART6_RX_M1`） |
| D500 `P5V` | 物理 Pin 2（5V） |
| D500 `GND` | 物理 Pin 20（GND） |
| D500 `PWM` | 物理 Pin 25（GND，该线按接线图接地，不是 ROCK 5A PWM 输出） |
| 设备树 overlay | `rk3588-uart6-m1.dtbo` |
| 设备 | `/dev/ttyS6`，属组 `dialout` |
| 波特率 | `230400 8N1`，只读 |

`rsetup` 启用 UART overlay 后重启。本次仅修改仓库源码，没有修改板端
`/boot/extlinux/extlinux.conf`；不要为了软件清理移除 D500 UART overlay。

## Alarm（声光报警）

| 项目 | 值 |
|---|---|
| 物理 Pin | 11 |
| 引脚功能 | `GPIO4_B3`（bank `gpio4`，line offset 11，全局约 139） |
| 极性 | active low（拉低=报警开） |

开机服务 `sound-light-alarm.service` 执行 `--off --grant-group gpio`，让
`gpio` 组成员可写 `direction`/`value`。

## C10B 驱动板

- USB 设备 `/dev/ttyACM0`，`115200 8N1`。
- 电机使能由驱动板 `KEY2` 控制；串口遥测帧第 2 字节 `00`=已使能。
- 左后轮接左侧带编码器电机接口，右后轮接右侧带编码器电机接口。

## HC-15 无线串口（2026-10-05 新接线）

用户提供的接线如下，Pin 均为 ROCK 5A 40Pin 排针的物理编号：

| HC-15 | ROCK 5A |
|---|---|
| VCC | Pin 1，3.3V |
| GND | Pin 9，GND |
| RXD | Pin 7，UART4_TX_M2（主板 TX → 模块 RX） |
| TXD | Pin 29，UART4_RX_M2（模块 TX → 主板 RX） |

Pin 7/29 和 `/dev/ttyS4` 的对应关系见
[Radxa 引脚表](https://docs.radxa.com/rock5/rock5a/hardware-design/hardware-interface)与
[UART4-M2 使用说明](https://docs.radxa.com/en/rock5/rock5a/getting-started/interface-usage/pin-40-test)。
板端通过 `rsetup` 启用 UART4-M2 overlay（`rk3588-uart4-m2.dtbo`）并重启后，
确认 `/dev/ttyS4` 存在且运行用户属于 `dialout`。保留已有 UART6-M1 雷达 overlay；
仅推送仓库代码不会自动修改设备树或重启小车。

串口组件默认 `/dev/ttyS4`，沿用 `115200 8N1`、无软件/硬件流控。
接线图未说明新的 UART 波特率，本次不修改电台存储的波特率、信道、空速或功率，
也不假设旧 HC-14 的 AT 指令适用于 HC-15。
原生 UART 没有连接 DTR/RTS；内核不支持该 ioctl 时允许打开，实际 I/O 错误仍失败。
串口独占、重连和地面站 `BB 33` 封装保持原有行为。

新接口是 `HC15SerialDriver` / `DEFAULT_HC15_PORT`；旧 HC14 名称和
`SerialCommunicationDriver` 保留为别名，旧调用也使用新端口。若需要旧 USB 硬件，
必须显式传入它的端口，不能自动猜测 CH340 或回退到 `/dev/ttyUSB0`。
`code/test/hc14_*.py` 仍是旧 HC-14 工具，只适用于原 USB 配置。

新接线的无发送串口检查（不发 AT、不开底盘；会独占串口并消费接收数据）：

```bash
python3 tools/hc15_probe.py --duration-s 3
```

需要合成位置应答时，显式运行 `code/test/fleet_car_pose_simulator.py --connect-hc15`；
它会向地面站发送模拟位置，不能与真实任务同时运行。
以上接线依据用户提供的图片，尚未执行真机双向通信验证。
