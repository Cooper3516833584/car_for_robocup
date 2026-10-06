# PWM 舵机（MG90S）在 ROCK 5A `PWM7_IR_M0`（物理 Pin 28）

本页记录本车上**单轴 PWM 舵机**的接线、板端使能步骤、软件接入点和实车验证命令。
组件位置：`code/components/servo_axis.py`；HAL：`code/hal/pwm.py`；
联调工具：`tools/servo_pwm7_probe.py`。

---

## 0. 结论摘要

| 项目 | 值 |
|---|---|
| 引脚名 | `PWM7_IR_M0`（`GPIO0_D0`） |
| 物理针脚 | 40-pin 排针 **Pin 28** |
| 设备树节点 | `pwm@febd0030` → sysfs 类名 **`febd0030.pwm`** |
| 设备树 overlay | **`rk3588-pwm7-m0`**（板端默认**未启用**） |
| 舵机 | Tower Pro **MG90S**（4.8~6.0 V 模拟舵机，50 Hz） |
| 0 度定义 | **中位脉宽 1500 us**（不是脉宽端点） |
| 行程（软件限位） | ±90 度 ↔ 500~2500 us，20 ms 帧周期 |
| 配置节 | `[devices.servo]`（`configs/robocup_diffdrive.toml`，默认 `enabled = false`） |
| 组件入口 | `build_servo(config, fake=False)` → `components.servo_axis.ServoAxis` |

> `PWM7_IR_M0` 这个名字里的 `IR` 表示该引脚在别的产品上是红外接收输入；
> 在本车它是 **PWM 输出**，功能由 overlay 的 mux 决定，与红外无关。

---

## 1. 为什么必须先启用 overlay

ROCK 5A 的 Pin 28 默认不做 PWM 输出。`pwm@febd0030` 节点的 `status` 是 `disabled`，
所以 `start()` 时芯片选择会失败并明确报错：

```
PWMBackendError: cannot find a PWM chip matching ('febd0030.pwm',); the pin has no
exported PWM channel - check that the device-tree/pinmux overlay for your board is
enabled and that the board has rebooted since
```

启用步骤（**需要重启，会中断板端所有程序**）。仓库提供了脚本化的做法，
避免手改引导配置；`tools/board_overlay.py` 会先备份 `extlinux.conf`：

```bash
# 1) 板端自带的 overlay 默认被重命名禁用（文件内容本身是合法的已编译 blob）
ls /boot/dtbo/rk3588-pwm7-m0.dtbo.disabled

# 2) 加入引导参数（自动备份 + 幂等，可重复执行）
sudo python3 tools/board_overlay.py enable rk3588-pwm7-m0
sudo python3 tools/board_overlay.py status     # 确认 fdtoverlays 行

# 3) 重启
sudo reboot
```

重启后确认（这一步只读，不会让舵机动）：

```bash
for c in /sys/class/pwm/pwmchip*; do readlink -f $c; done
# 期望出现 .../febd0030.pwm/pwm/pwmchipN；本车实测为 pwmchip2
python3 tools/servo_pwm7_status.py
```

回滚（脚本已自动备份，路径会打印出来）：

```bash
sudo python3 tools/board_overlay.py disable rk3588-pwm7-m0 && sudo reboot
```

> **不要**用未列出的 overlay 猜测：`rk3588-pwm7-m2` / `-m3` 对应别的针脚，
> 本车验证过的是 **`-m0` = Pin 28**。
>
> `fdtoverlays` 行里的文件后缀不影响加载：U-Boot 直接按路径读 blob，
> `.disabled` 只是 Radxa 用来"停用"的命名约定，所以脚本不需要改名或重新编译。

---

## 2. 接线

| 舵机线 | 接到 | 说明 |
|---|---|---|
| 橙/黄（信号） | Pin 28（`PWM7_IR_M0`） | 3.3 V PWM，实测可用 |
| 红（电源+） | **独立 5~6 V BEC** | 不要接排针 5V：MG90S 堵转电流 1.8~2.5 A，会把板子拉挂 |
| 棕/黑（地） | BEC 地 + 排针任意 GND | 信号必须有共地回流，否则抖动/不动 |

要点：

- **供电必须独立**。排针 5 V 是给逻辑用的，不是给舵机用的。
- 3.3 V 高电平在实车上能驱动 MG90S，但处在部分舵机输入阈值边缘；
  如果出现抖动或不吃指令，加一个 3.3 V→5 V 电平转换。
- 舵机盘装配后，**中位（1500 us）应对应机械零位**，这样 `0` 度不会顶限位。

---

## 3. 角度与脉宽的约定

`code/components/servo_axis.py` 刻意采用"**0 度 = 中位**"的约定：

| 角度 | 脉宽 |
|---|---|
| -90° | 500 us |
| -45° | 1000 us |
| **0°** | **1500 us** |
| +45° | 2000 us |
| +90° | 2500 us |

即 `pulse = pulse_mid + angle * (pulse_max - pulse_min) / (2 * half_range)`。

这样约定的原因：未标定的舵机如果上电就被写成脉宽端点（1000 或 2000 us），
输出臂可能直接顶到机械限位并堵转。中位是最安全的已知位置，也是本项目
"转到 0 度"的含义。

规则与边界：

- 超出标定行程的角度**直接报错**（`ServoRangeError`），不会悄悄截断成端点；
- 脉宽必须满足 `0 <= pulse_min_us < pulse_max_us < period_us`。
  等于整帧周期会被拒绝：那样没有低电平间隔，舵机收不到合法帧；
- 若你的舵机实际可用行程不是对称 ±90°，用 `travel_from_endpoints()` 或
  配置里的 `pulse_min_us`/`pulse_max_us` 标定，`0` 度仍然落在两个端点的中点；
- **连续旋转舵机（"360° 舵机"）没有位置反馈**：同样的脉宽含义是"速度"，
  1500 us 是停转，其它值是转速。此时"转到某个角度"没有物理意义，不要用本组件做位置控制。

MG90S 规格参考：堵转扭矩 ~1.8 kg·cm、速度 ~0.1 s/60°、死区 ~5 us。
按 500~2500 us / ±90° 换算，1 度 ≈ 11.1 us，所以脉宽按 1 us 取整带来的
角度分辨率约 0.09 度，远优于舵机自身精度。

---

## 4. 软件接入点

| 文件 | 作用 |
|---|---|
| `code/components/servo_axis.py` | 组件本体：角度↔脉宽换算、行程校验、限速、回中、释放 |
| `code/hal/pwm.py` | `LinuxSysfsPWMOutput` + `resolve_chip()`：sysfs PWM 输出与芯片选择 |
| `code/config/v2_models.py` | `ServoConfig` 校验（`[devices.servo]`） |
| `code/config/v2_loader.py` | 可选表：没有 `[devices.servo]` 的老档案照旧加载 |
| `code/config/v2_factory.py` | `build_servo(config, fake=...)` |
| `code/test/test_servo_axis.py` | 纯逻辑单测（内存轴，不开 PWM） |
| `code/test/test_hal_pwm.py` | HAL 单测（假 sysfs 树，含芯片选择回退） |
| `tools/servo_pwm7_probe.py` | 实车联调 CLI（需 `--confirm`） |
| `tools/servo_pwm7_status.py` | 只读自检：模块、配置、脉宽映射、通道是否可用 |
| `tools/servo_pwm7_hold.py` | 停在某角度并保持（detach，`--stop` 干净退出） |
| `tools/servo_backdrive_probe.py` | 诊断回弹：空载/保持/松开三段的母线电压对比 |
| `tools/board_overlay.py` | 板端 overlay 启停/查询（改 `extlinux.conf`，自动备份） |
| `tools/car_ssh.py` / `car_sync.py` | 本机辅助脚本，**不在仓库里**（见 §7.2） |

组件不读 TOML：生产路径由 `build_servo()` 把校验过的配置传进来。构造**不会**打开硬件，
只有 `ServoAxis.start()` 才写 sysfs——与仓库其它组件"构造不动作"的约定一致。

典型用法：

```python
from components.servo_axis import MG90S_TRAVEL, ServoAxis
from hal.pwm import LinuxSysfsPWMOutput

output = LinuxSysfsPWMOutput(chip_device_match="febd0030.pwm", channel=0)
axis = ServoAxis(MG90S_TRAVEL, pwm=output, settle_s=0.35)
axis.start()                            # 回中到 0 度（1500 us）
pulse = axis.set_angle(-45.0, settle=True)   # 转到 -45 度并等它到位
print(axis.angle_deg, pulse)            # -45.0 1000
axis.release()                          # 关掉脉冲串，舵机卸力
```

### 4.0 按速率扫描（`sweep_to` / `sweep_by` / `return_to_start`）

`set_angle()` 是"给一个目标角度"，`sweep_to()` 是"以某个角速度移过去"：

```python
axis.start()                                            # 记录起始角 0 度
axis.set_angle(-90.0, settle=True)                      # 先到 -90 度
axis.sweep_to(90.0, rate_deg_s=30.0)                    # 30 度/秒扫到 +90 度（6 s）
axis.sweep_to(-90.0, rate_deg_s=30.0)                   # 再 30 度/秒扫回来（6 s）
axis.sweep_by(15.0, rate_deg_s=30.0)                    # 相对当前位置
axis.return_to_start(rate_deg_s=30.0)                   # 回到 start() 记录的起始角
```

实现与边界：

- 舵机固件**没有轨迹发生器**，速率是软件插值出来的：按 `step_deg`（默认 1 度）
  切分，每步写一次脉宽，再补上 `step/rate` 的等待。所以运动是阶梯状的，
  阶梯粗细由 `step_deg` 决定，越小越平滑但主机调度压力越大；
- 每步耗时**不会短于一帧 PWM 周期**（20 ms）。因此 `rate_deg_s` 过高时
  实际会慢于请求值，而不是超速刷写通道（1000 度/秒 + 1 度步进实测退化为
  每步 20 ms）；
- `sweep_to` **故意绕过** `min_command_interval_s`：速率本身已经限速，
  两者叠加会让扫描卡顿或速率失真。但下一条普通 `set_angle` 仍受该限速约束；
- 目标角一定**精确落在**请求值上，即使步长除不尽行程（`step_deg=7` 走 47 度也对）；
- 目标角越界抛 `ServoRangeError`，**不会**静默截断到端点；
- `return_to_start()` 用的是 `start()` 记录的起始角，不是"当前位置"，
  所以中途丢步也仍能回到操作员上电时看到的位置；用 `start(home=False)` 启用
  而未记录起始角时，该调用会抛 `ServoNotStartedError` 而不是瞎猜。
  `start(home=True)` 时起始角就是 `home_angle_deg`（默认 0 度）；
  `start(home=False, sweep_start=True)` 只**记录**起始角而不写脉冲，
  便于调用方自己发第一条命令（CLI 就是这样，否则第一条命令会被限速拦掉）。

`tools/servo_pwm7_probe.py` 对应参数：`--sweep <角度...> --rate-deg-s 30
--step-deg 1 --return`。其中 `--home` 同时作为舵机的起始角/返回目标，
所以 `--home -90 --sweep 90 --return` 的含义是：
**到 -90 度 → 30 度/秒扫到 +90 度 → 30 度/秒回到 -90 度**。

### 4.1 芯片选择为什么不用 `pwmchipN`

`/sys/class/pwm` 的**类名**会被截断：`febd0000.pwm` 与 `fd8b0000.pwm` 都显示成 `fd8b0000.pwm`。
所以按 `pwmchip0` 这种序号选芯片会随 probe 顺序变化而选错针脚。
`resolve_chip()` 因而支持三级选择器，按顺序尝试：

1. `pwmchipN`：sysfs 字面名，简单但依赖 probe 顺序；
2. **`febd0030.pwm`（设备树节点名）**：读取 `device` 符号链接后**精确比较**节点名，
   不受 probe 顺序和类名截断影响 —— 本车用这个；
3. 其它字符串：解析后路径的**子串**匹配，给未知板卡兜底。

约束（都有单测覆盖）：

- 设备树节点名的比较是**精确**匹配，不会退化成子串匹配。否则 `febd0030.pwm`
  会匹配到任何设备路径里含该名字的控制器（实测就曾错误命中 `pwmchip0`）；
- 配置里**不要**加 `pwmchipN` 兜底项。本车 `pwmchip0` 是 D500 的 `fd8b0000.pwm`，
  数字兜底会静默驱动错误针脚，而不是报错；
- 符号链接用 `os.readlink` 手工展开，不用 `Path.resolve()`：板端是 Python 3.11，
  当路径最后一个分量本身是符号链接时，`resolve()` 不保证继续展开嵌套的 `device`
  链接（Python 3.13 行为不同）。这是实测发现并修掉的问题，属于必须保持的写法。

### 4.2 权限（实测：需要 root）

**结论：写这个通道需要 root。** 实车验证：

| 事实 | 值 |
|---|---|
| 通道属主/权限 | `root:pwm`，模式 `775`（`period`/`duty_cycle`/`enable` 都是） |
| `radxa` 的组 | 含 `108(pwm)`、`106(gpio)` |
| 以 `radxa` 运行 `servo_pwm7_hold.py` | **`Permission denied: .../pwm0/period`** |
| 以 root 运行同一命令 | 正常写入并保持 |

也就是说，尽管文件系统权限位对 `pwm` 组开放，sysfs 属性写入仍被拒绝——
**不要依赖组权限**，那些位看起来够用但实际不行。首次联调时它曾短暂成功过，
所以不要把它当成稳定行为。

推荐做法（按优先级）：

1. **开机服务以 root 运行保持器**。仓库里已有用 systemd 跑 root 级硬件服务的
   先例（声光报警、电池监控），与现有部署方式一致，也避免把密码放进脚本：

   ```ini
   [Unit]
   Description=Park PWM servo at 0 degrees and hold
   After=multi-user.target

   [Service]
   Type=simple
   Environment=PYTHONPATH=/home/radxa/car/code
   ExecStart=/usr/bin/python3 /home/radxa/car/tools/servo_pwm7_hold.py --foreground
   Restart=on-failure

   [Install]
   WantedBy=multi-user.target
   ```

   注意 `--foreground`：systemd 自己就是 supervisor，不需要再 fork。

2. **联调时直接 `sudo`**（最简单）。

> 不要按旧文档把 `pwmchip*` 改到 `gpio` 组：实测那既不是正确的组，实际也不生效。
> 旧的 `rock5a-pwm0-permissions.service` 也不要再引入（见 `docs/LEGACY_CLEANUP.md`）。

### 4.3 为什么"松手后位置会变"，以及怎么让它真的停在某个角度

**现象**：命令到 0 度、程序退出后，舵机有时会自己弹回 -30 度左右；有时又停在 0 度不动。

**结论：这不是软件 bug，但 `--release` 是错误的选择。** 排查过程与证据：

| 检查 | 结果 | 说明 |
|---|---|---|
| 保持 0 度、上电 45 s，期间读 sysfs | `duty_cycle=1500000`、`enable=1` 全程不变 | 脉宽与通道状态稳定，没有软件在改它 |
| 上电保持期间目视 | 一直停在 0 度，无回弹 | 舵机+控制环正常，能顶住负载 |
| 命令到 +90 度并保持 | 正常顶到 +90 度并保持 | **供电、共地、信号、行程权限全部正常** |
| 松开后（`enable=0`）| 位置不再可复现：曾回弹到 -30 度，也曾停在 0 度 | 无保持力矩时位置只由机械力平衡决定 |
| C10B 母线电压对比（空载 / 保持 / 松开）| 12.440 / 12.450 / 12.440 V | 分辨率不够，**结论不可用**（见下） |

根本原因：**舵机只在脉冲串使能（`enable=1`）时才有保持力矩。** 松开后
（`enable=0`）舵机内部没有主动力矩，只剩减速齿轮的静摩擦；此时旋转面上的
相机线束会像一个弹簧一样把它往回拉，是否拉得动取决于线束回弹力与静摩擦谁大。
所以"松手后的角度"是一个自由平衡结果，不可能稳定复现——这正是"有时回弹、
有时不回弹"的原因。相机线束（整圈旋转面水平、上面放摄像头）是唯一会随角度
变化的力源，与观察吻合。

`tools/servo_pwm7_probe.py` 和 `ServoAxis.close()` **默认就是松开**，所以默认
行为会让这种轴丢位置。

正确做法：**舵机默认保持使能**（本车采用的策略，也是本仓库的默认行为）。
理由：MG90S 这类舵机在脉冲串停止后**完全没有保持力矩**，带相机的水平旋转面会被
线束回弹力拉走；而保持的代价只是舵机静态电流（小舵机带轻载约 10~20 mA 量级），
远小于转动电流，这点功耗在整车上可以接受。

```bash
# 调到 0 度并一直保持 —— 不需要任何额外参数
python3 tools/servo_pwm7_hold.py

# 保持 30 分钟后自动松开
python3 tools/servo_pwm7_hold.py --minutes 30

# 停止并松开（--stop 一定会释放，避免停在一个持续耗电的状态）
python3 tools/servo_pwm7_hold.py --stop

# 只在确实想让轴自由时才显式松开
python3 tools/servo_pwm7_hold.py --park -30 --release
```

**不需要 `sudo`**：本车 `radxa` 用户在 `pwm` 组里，而 sysfs 通道是
`root:pwm` + `rwxrwxr-x`，所以组内用户可以直接写。早期的
`rock5a-pwm0-permissions.service` 之所以存在是因为按 `gpio` 组授权；
现在用 `pwm` 组就够，不要再引入开机服务。

组件层同样默认保持：

- `ServoAxis.close()` **默认保持**（`hold=True`）；要松开必须显式
  `close(hold=False)` 或调 `release()`；
- `ServoAxis.__exit__`（`with` 退出）同样默认保持，避免 `with` 块结束就丢位置；
- 配置里**没有**单独的"松开"开关：默认就是保持，只有一个选择时多一个字段
  只会变成误导（早期版本的 `release_on_close` 字段已删除）。

如果确实要靠机械解决（松开也不动），改线束走向是最彻底的办法：把相机线束
固定在旋转件上、留一段服务环、让线束沿转轴走向，并在 0 度附近处于松弛区。
另一个省电的折中是把静止位设成线束的松弛角（例如 -30 度），松开即可。

**两个容易踩的板端行为**（已实测）：

- **进程死后通道仍然输出**：内核会继续按最后写入的 `duty_cycle` 发波。
  用 `timeout`/`kill` 杀掉保持进程时，`finally` 不会执行，舵机会**保持在
  最后那个角度**（我们实测到杀进程后 `enable=1`、停在 +90 度）。这比"啪一下
  掉下来"安全，但不能当成请求成功——要确认状态请读
  `/sys/class/pwm/pwmchip2/pwm0/enable`，或直接用 `tools/servo_pwm7_status.py`
  （它会打印 `ENABLED (holding torque)` 和当前角度）。
- **不带 `setsid` 的后台进程会随 SSH 会话被杀**：前台跑保持工具，SSH 一断
  就结束。`tools/servo_pwm7_hold.py` 因此自己 `fork + setsid`，并把运行时文件
  （pid/log/stop）放到**当前用户可写**的目录：root 用 `/run`，普通用户用
  `/run/user/<uid>`。这一点是踩过的坑——早期固定写 `/tmp`，root 先跑过一次后，
  普通用户再启动就会因为 root 拥有的 `/tmp/servo_pwm7_hold.pid` 拿到
  `PermissionError` 而直接退出（此时**通道还停在之前的状态**，很容易误判成
  "舵机没动"）。

如果要用外部工具量回弹力矩，`tools/servo_backdrive_probe.py` 会按
"空载 → 保持 → 松开"三段采样母线电压，但它**测不出小舵机带轻载的差别**
（实测 0.01 V），所以它只适合判断"舵机是否在明显堵转"，不能用来给小回弹力定级。

---

## 5. 实车验证

```bash
# 0) 干跑：只报告芯片、行程和将要写入的脉宽，不写硬件
python3 tools/servo_pwm7_probe.py

# 1) 真正转到 0 度（中位 1500 us），保持 0.5 s 后释放
sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_pwm7_probe.py --confirm --home 0

# 2) 到 -90 度，再以 30 度/秒扫到 +90 度，最后 30 度/秒回到 -90 度
sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_pwm7_probe.py --confirm \
    --home -90 --sweep 90 --rate-deg-s 30 --return

# 3) 扫多个角度（每点停留 0.5 s，最后释放）
sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_pwm7_probe.py --confirm \
    --home 0 --sweep -90 -45 45 90 --rate-deg-s 30

# 4) 保持上电（不释放，用于目视确认）
sudo PYTHONPATH=/home/radxa/car/code python3 tools/servo_pwm7_probe.py --confirm \
    --home 0 --keep-enabled
```

工具在**所有退出路径**（含异常与 Ctrl-C）都会 `release()` 关掉脉冲串，
除非显式给了 `--keep-enabled`。释放后舵机没有保持力矩，负载会让它掉下来。
每个扫描点都会打印实际耗时，可以据此核对速率是否符合预期
（180 度 @ 30 度/秒 应约为 6.0 s）。

组件自带 CLI（等价、参数更细）：

```bash
sudo PYTHONPATH=/home/radxa/car/code python3 -m components.servo_axis --angle 0
```

---

## 6. 已实测 / 未实测

- **已实测（2026-10-06，本车 Cooper ROCK 5A）**：
  - Pin 28 由 `rk3588-pwm7-m0` overlay 驱动；重启后 `pwm@febd0030` 的 `status`
    变为 `okay`，`/sys/class/pwm/pwmchip2` → `febd0030.pwm`；
  - `--home 0` 写入 1500 us：sysfs 读到 `period=20000000`、`duty_cycle=1500000`
    （20 ms 帧 / 1.5 ms 脉冲）；释放后 `enable=0`；
  - **速率扫描实测**：`--home -90 --sweep 90 --rate-deg-s 30 --return` 序列执行成功，
    两个 180 度行程各耗时 6.40 s（= 6.00 s 运动 + 0.35 s settle + ~0.05 s 调度），
    即速率与请求的 30 度/秒一致（误差约 0.1%，步长 1 度）；结束停在 -90 度
    （`duty_cycle=500000`），随后释放脉冲串；
  - `code/test/test_servo_axis.py`（61 项）与 `code/test/test_hal_pwm.py`（15 项）
    在板端 Python 3.11.2 上全部通过；`code`、`tools` 在板端 `compileall` 通过；
  - `main_robocup.py` 与 `components`/`config` 的惰性导出在板端可正常导入，
    `config/v2_runtime.py` 的三个模式约束仍可构建（HARDWARE_MISSION 仍有 1 项
    与舵机无关的待测项）。
- **未实测**：行程端点 500/2500 us 是否对应本颗 MG90S 的真实机械 ±90°
  （本次只证明软件按标定值输出，不证明机械角度）；持续扫描的温升与长期重复性；
  3.3 V 电平在低温或长线下的余量；舵机回路与 `robocup_runtime.py` 的集成
  （`build_servo()` 已就绪但没有任务代码调用它）。
- 因此 `pulse_min_us`/`pulse_max_us` 目前是**厂商规格默认值**，不是实测标定值。
  需要绝对精度时，按 `docs/CALIBRATION.md` 的方法记录实测脉宽再改配置。

---

## 7. 板端部署方式（现状：`git pull --ff-only`）

**现在板端就是普通 Git 部署，不需要任何专用部署脚本。** 舵机改动已在
`origin/main` 里，板端直接拉取即可：

```bash
ssh ROCK-5A                      # ~/.ssh/config 里的别名，用 key 登录
cd /home/radxa/car
git rev-parse --short HEAD       # 确认当前提交
git status --porcelain           # 必须是干净的（gitignore 的实车配置除外）
git fetch origin
git pull --ff-only origin main
python3 -m compileall -q code tools
python3 tools/servo_pwm7_status.py
```

### 7.1 曾经为什么要"外科式同步"（历史，已结束）

提交 `2abbac3` 合并之前，板端 checkout 与开发工作树的**分支谱系不同**：板端
`v2_loader.py` 是宽容版（容忍已停用的 `navigation.map`/`navigation.footprint` 键），
`v2_models.py` 没有 `CompetitionMapConfig`/`FootprintConfig`。那时直接全量覆盖会让
板端 profile 加载失败，所以曾经用两个临时脚本把舵机代码"补丁"进板端版本：

- `tools/board_servo_patch.py`（在板端版本上生成最小补丁）
- `tools/board_servo_deploy.py`（逐文件备份 + 回滚脚本 + 板端自检）

**这两个脚本已经删除**，原因不是它们不好用，而是它们的输入前提消失了：

1. 舵机代码现在是 `origin/main` 的原生内容，板端 `git pull` 就能拿到；
2. 在"已经含 ServoConfig"的文件上再跑一次补丁，只会重复插入或需要
   `--refetch` 去撤销自己的改动——这正是本次踩到的坑：
   `--refetch` 从已打过补丁的板端取回文件后，那块**过期的 `ServoConfig`
   会被当成基线保留**，于是板端跑的默认值和仓库不一致（旧版含已删除的
   `release_on_close`）。保留一个只能用来自我撤销的工具没有意义。

历史产物仍可查：`git log 2abbac3`，以及板端
`/home/radxa/car_servo_backups/<时间戳>/`（含每次部署的备份与 `rollback.sh`）。

### 7.2 部署前必须知道的两件事

- **写 PWM 通道需要 root**（见 §4.2），所以板端跑 `servo_pwm7_hold.py` 要 `sudo`；
  单纯 `git pull` 和跑测试都不需要。
- **`tools/car_ssh.py` / `tools/car_sync.py` 不是仓库文件**：它们是本机辅助脚本，
  `origin/main` 里没有。`car_ssh.py` 走 `~/.ssh/config` 的 key 认证（不再内置密码），
  `car_sync.py` 是整树快照通道，谱系一致时才用。日常部署按上面的 `git pull` 即可。
