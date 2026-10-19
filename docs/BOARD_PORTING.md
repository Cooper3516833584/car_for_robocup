# 开发板迁移与复现（ROCK 5A）

本文档描述如何把本仓库部署到一块**新的 ROCK 5A** 上，使它在本车相同的接线与
服务配置下直接运行。它补充（不替代）`docs/HARDWARE_PORTING.md`、
`docs/platforms/rock5a.md`、`docs/PAYLOAD_SG90.md` 和 `deploy/board/README.md`。

记录时间：2026-10-19。对应实车：`radxa@10.202.217.163`，仓库 `main`，
`b6ab4df`，Debian 12 + 内核 `6.1.43-20-rk2312`。

## 1. 仓库现在包含哪些“板端配置”

2026-10-19 之前只存在于 SD 卡、没有进 Git 的内容，现在按类别进入仓库：

| 内容 | 位置 |
|---|---|
| 实车运行配置（原被 `.gitignore` 忽略） | `configs/robocup_diffdrive.toml` |
| 任务服务单元、T265 自愈单元 | `deploy/*.service`、`deploy/*.timer.in` |
| PWM0 权限脚本与 oneshot 单元 | `deploy/board/rock5a-pwm0-permissions*` |
| 用户组、udev、设备节点的一键配置 | `deploy/board/install_board_setup.sh` |
| ROS 2 / 视觉环境重建配方 | `deploy/ros_env/` |
| 引脚、串口、GPIO、PWM 的完整映射 | 本文第 3 节与 `docs/PAYLOAD_SG90.md` |
| 代码里的硬编码常量清单 | 本文第 10 节 |

**刻意不进入公开仓库**的内容：WiFi 密码（NetworkManager 连接文件）、SSH
私钥、4.6 GB 的 `robocup_ros` 环境、运行日志、历史备份。这些要么是秘密，要么
可由本文档的配方重建。仓库是 public，任何情况下都不要把密码或私钥提交进去。

## 2. 新板落地步骤

以下命令默认仓库在 `/home/radxa/car`、服务用户为 `radxa`。路径不同也可以，
`deploy/install_car_services.sh` 会把单元里的路径改写成实际位置；但
`deploy/ros_env` 默认把环境放在 `/home/radxa/robocup_ros`（与仓库同级）。

1. 刷 Radxa Debian 12 镜像，启动，创建可 `sudo` 的 `radxa` 用户，接入网络。
2. 克隆仓库：`git clone https://github.com/Cooper3516833584/car_for_robocup.git /home/radxa/car`。
3. 板级配置（用户组、PWM0 权限、udev、T265 单元、节点检查）：
   `sudo sh /home/radxa/car/deploy/board/install_board_setup.sh`
4. 若 T265 没有 `pyrealsense2`：`sudo bash deploy/ros_env/build_pyrealsense2.sh`
   （约 20–40 分钟，需要联网）。
5. 以 `radxa` 用户重建 ROS/视觉环境：
   `bash /home/radxa/car/deploy/ros_env/create_ros_env.sh`（约 6 GiB，首次较慢）。
6. 如果第 3 步报告 PWM/UART 节点缺失：
   `sudo sh deploy/board/install_board_setup.sh --enable-overlays && sudo reboot`，
   重启后再跑一次第 3 步确认全部 `ok`。
7. 安装任务服务：`sudo sh /home/radxa/car/deploy/install_car_services.sh`。
8. 按本板实际接线修改 `configs/robocup_diffdrive.toml`（见第 7 节），并在新板
   重新标定几何/外参；不要直接沿用另一辆车的标定值。
9. 无硬件自检：
   ```sh
   py -3 -m compileall -q code tools
   py -3 -m unittest discover -s code/test -p "test_*.py"
   python3 code/main_robocup.py --config configs/robocup_diffdrive.toml --mode dry-run
   ```
10. 确认传感器和底盘都接好后，再 `sudo systemctl enable --now competition-boot.service`。
    注意：拉取代码不等于授权跑电机或自主任务。

## 3. 引脚与设备映射

物理引脚编号均为 ROCK 5A 40Pin 排针。设备路径是本车当前值，换板时按实际枚举修改。

| 功能 | 物理 Pin | 复用/节点 | 设备路径 | 配置/代码 |
|---|---|---|---|---|
| C10B 驱动板 USB | — | USB CDC | `/dev/ttyACM0`，115200 8N1 | `[devices.c10b] port` |
| D500 雷达 TX | 21 | `UART6_RX_M1` / `feb90000.serial` | `/dev/ttyS6`，230400 8N1，只读 | `[devices.d500] port` |
| D500 5V / GND | 2 / 20 | — | — | — |
| HC-15 VCC / GND | 1 / 9 | 3.3V / GND | — | — |
| HC-15 RXD（板 TX） | 7 | `UART4_TX_M2` / `feb70000.serial` | `/dev/ttyS4`，115200 8N1 | `[devices]`/`code/components/serial_communication.py` |
| HC-15 TXD（板 RX） | 29 | `UART4_RX_M2` | `/dev/ttyS4` | 同上 |
| 物资舵机 0（黄） | 23 | `PWM0_M2` / `fd8b0000.pwm` ch0 | `/sys/class/pwm/pwmchipN` | `payload_servos.py` |
| 物资舵机 1（单件） | 24 | `PWM1_M2` / `fd8b0010.pwm` ch0 | 同上 | 同上 |
| 物资舵机 2（双件 1） | 15 | `PWM11_M1` / `febe0030.pwm` ch0 | 同上 | 同上 |
| 物资舵机 3（双件 2） | 13 | `PWM14_M1` / `febf0020.pwm` ch0 | 同上 | 同上 |
| 物资舵机 4（右安装） | 27 | `PWM6_M0` / `febd0020.pwm` ch0 | 同上 | 同上 |
| 相机舵机 | 28 | `PWM7_IR_M0` / `febd0030.pwm` ch0 | 同上 | `[devices.servo]` |
| 声光报警 | 11 | `GPIO4_B3`，active low | `/dev/gpiochip*` | `code/components/sound_light_alarm.py` |

PWM 槽位一律按**设备树节点名**匹配，不能按 `pwmchipN` 序号猜测：

- `code/hal/pwm.py::resolve_chip()` 精确匹配 `device` 链接。
- `code/components/payload_servos.py` 写死五个节点名。
- `code/config/v2_models.py` 的默认 `chip_device_match=("febd0030.pwm",)` 是相机轴。

## 4. 设备树 overlay

当前实车的 `fdtoverlays` 行（2026-10-19）：

```text
/boot/dtbo/rk3588-pwm0-m2.dtbo /boot/dtbo/rk3588-uart6-m1.dtbo
/boot/dtbo/rk3588-pwm7-m0.dtbo.disabled ... 其余为 .disabled
```

但**当前镜像的 base DT 已经把** PWM1/6/7/11/14 和 UART4-M2 路由好了（实测
`/proc/device-tree` 中这些节点 `status=okay` 且带 `pinctrl-0`），所以实车在
这些 overlay 显示为 disabled 时仍然工作。判断标准只能是内核实际暴露的节点，
不是 extlinux 行。

完整能力所需的 overlay 名单：

| overlay | 用途 |
|---|---|
| `rk3588-pwm0-m2` | Pin 23 / 物资舵机 0 |
| `rk3588-pwm1-m2` | Pin 24 / 物资舵机 1 |
| `rk3588-pwm11-m1` | Pin 15 / 物资舵机 2 |
| `rk3588-pwm14-m1` | Pin 13 / 物资舵机 3 |
| `rk3588-pwm6-m0` | Pin 27 / 物资舵机 4 |
| `rk3588-pwm7-m0` | Pin 28 / 相机轴，**禁止禁用** |
| `rk3588-uart6-m1` | D500 雷达 |
| `rk3588-uart4-m2` | HC-15 电台 |

用仓库脚本查看/启用（会先备份 `extlinux.conf`，需重启）：

```sh
python3 tools/board_overlay.py status
sudo python3 tools/board_overlay.py enable rk3588-pwm0-m2
sudo reboot
```

`deploy/board/install_board_setup.sh --enable-overlays` 会对缺失项批量执行上述命令。

## 5. 用户组、权限与 udev

服务用户必须属于：

```text
dialout gpio pwm i2c spidev video plugdev
```

- 串口：`dialout`
- 声光报警：`gpio`（`sound-light-alarm.service` 以 root 执行 `--grant-group gpio`）
- 五路物资 + 相机 PWM：`pwm`
- T265/D500 USB 节点：`plugdev`
- 摄像头：`video`

`install_board_setup.sh` 会 `usermod -aG` 并创建缺失的组。组变更后要重新登录
（或重启）才对已有 shell 生效。

PWM 节点本身还有一层权限：

- `/etc/udev/rules.d/99-realsense-libusb.rules`：RealSense USB 节点 0666/plugdev
  （来自 `tools/99-realsense-libusb.rules`）。
- `/etc/udev/rules.d/99-t265-boot-init.rules`：T265 插入即触发自愈单元。
- `/usr/local/sbin/rock5a-pwm0-permissions` + `rock5a-pwm0-permissions.service`：
  导出 pwm0 并把 export/period/duty/enable/polarity 交给 `pwm` 组。
- 五路物资和相机通道由 `tools/setup_payload_pwm_permissions.py` 在
  `competition-boot.service` 的 `ExecStartPre` 中以 root 导出并 chmod 0777；
  该脚本对**不存在**的通道只告警不失败，所以 overlay 缺失时任务仍能启动，但
  对应轴不可用。

## 6. systemd 服务

| 单元 | 安装者 | 实车状态 | 作用 |
|---|---|---|---|
| `sound-light-alarm.service` | `install_car_services.sh` | enabled | 开机把 GPIO4_B3 置安全电平 |
| `battery-voltage-monitor.service` | 同上 | enabled | C10B 低压声光报警 |
| `servo-pwm7-home.service` | 同上 | enabled | 开机把相机轴回 0° |
| `robocup-slam.service` | 同上 | 已安装，手动/按需 | slam_toolbox sidecar |
| `competition-boot.service` | 同上 | enabled | 监听电台、视觉、任务主流程 |
| `t265-boot-init.service` / `.timer` | `install_t265_boot_init_service.sh` | enabled | T265 启动/热插拔自愈 |
| `rock5a-pwm0-permissions.service` | `install_board_setup.sh` | enabled | PWM0 组权限 |

`install_car_services.sh` 用 `sed` 把单元里的 `/home/radxa/car` 和
`/home/radxa/robocup_ros` 替换成实际路径，因此仓库可以放在别处（但
`robocup_ros` 仍按仓库同级推导）。

实车上还残留一个 disabled 的 `mission-screen-launcher.service`，它引用
`configs/cooper_rock5a_l150.toml`（该文件不存在）。不要启用它；需要串口屏
启动器时按 `code/mission_screen_launcher.py --help` 重新配置。

## 7. 运行时配置

`configs/robocup_diffdrive.toml` 现在**受版本管理**，是实车实测值；示例文件
`configs/robocup_diffdrive.example.toml` 仍是占位模板。

换板时必须逐项确认：

| 配置项 | 本车值 | 说明 |
|---|---|---|
| `[devices.c10b] port` | `/dev/ttyACM0` | USB 枚举顺序可能变化，用 `ls -l /dev/serial/by-id` 固定 |
| `[devices.d500] port` | `/dev/ttyS6` | 必须是 UART6-M1 |
| `[devices.t265] serial` | `230322111433` | **另一台 T265 的序列号不同**；留空则取第一台 |
| `[devices.servo] chip_device_match` | `["febd0030.pwm"]` | 相机轴，不能改成 pwmchipN |
| `[vehicle.geometry]` | 198 mm 轮距、67 mm 轮径 | 必须按新车重测 |
| `[sensors.*.mount]` | D500/T265 外参 | 必须按新车重测，roll/pitch 本车按 0 处理 |
| `[calibration]`、`[safety]` | 多项 `false` | 未标定前 `hardware-mission` 会 fail closed，这是有意的 |

修改后至少跑一遍 dry-run 和单元测试；不要用示例值覆盖实车标定。

## 8. ROS 2 / 视觉环境

实车布局（共约 4.6 GB，未入库）：

```text
/home/radxa/robocup_ros/
  bin/micromamba              2.9.0
  root/                       MAMBA_ROOT_PREFIX
  env/                        python 3.11.15 + ROS 2 Humble + slam_toolbox
  vision_env/                 venv --system-site-packages（torch/ultralytics）
  python_ext/                 pyrealsense2 的 PYTHONPATH 入口
```

重建顺序与固定版本见 `deploy/ros_env/README.md`：

1. `sudo bash deploy/ros_env/build_pyrealsense2.sh`（v2.53.1 + Python 3.11 补丁）
2. `bash deploy/ros_env/create_ros_env.sh`
3. `deploy/install_car_services.sh` 会检查
   `competition_boot`、`main_robocup`、`rclpy`、`pyrealsense2`、`cv2`、
   `rapidocr`、`onnxruntime`、`ultralytics` 是否都能导入。

## 9. 网络

实车用 DHCP + WiFi，当前地址 `10.202.217.163/24`，网关 `10.202.217.28`。
WiFi 连接文件含 PSK，**不提交**。新板用：

```sh
nmcli device wifi connect "<SSID>" password "<PASSWORD>"
```

需要固定地址时用 `nmcli connection modify ... ipv4.addresses ...`，不要把
NetworkManager 的 `.nmconnection`（含 `psk=`）复制进仓库。

## 10. 代码里的硬编码常量

这些值已在代码中固定，换板时按表修改；本次同步没有改动运行逻辑。

| 位置 | 值 | 换板时 |
|---|---|---|
| `code/competition_boot.py:100` | `port='/dev/ttyS4'` | 电台不在 ttyS4 时改这里 |
| `code/competition_task.py:112` | `HC_PORT = "/dev/ttyS4"` | 同上 |
| `code/components/radar_driver.py:38` | `DEFAULT_D500_PORT = "/dev/ttyS6"` | D500 换 UART 时改 |
| `code/components/serial_communication.py:29` | `DEFAULT_HC15_PORT = "/dev/ttyS4"` | 同上 |
| `code/components/payload_servos.py:10-18` | `fd8b0000/fd8b0010/febe0030/febf0020/febd0020.pwm` | 引脚改接时改节点名 |
| `code/components/servo_axis.py:113` | `ROCK5A_PWM7_M0_CHIP = "febd0030.pwm"` | 相机轴换脚时改 |
| `code/config/v2_models.py:135` | 默认 `chip_device_match=("febd0030.pwm",)` | 与配置一致 |
| `code/components/sound_light_alarm.py:31` | `GPIO_BANK_LABEL="gpio4"`，line 11 | 换 GPIO 时改 bank/offset |
| `code/hal/gpio.py:78` | `line_offset: int = 11` | 同上 |
| `code/test/battery-voltage-monitor.service:15` | `--device /dev/ttyACM0` | C10B 换名时改 |
| `tools/hc15_link_test.py:105` | `192.168.31.107` / `192.168.31.224` | 仅测试脚本，替换为现场地址 |
| `deploy/*.service` | `/home/radxa/...` | 由安装脚本自动改写，不要手改板端 |

## 11. 新板验收命令

```sh
# 服务与单元
systemctl is-enabled rock5a-pwm0-permissions t265-boot-init.timer competition-boot
systemctl status competition-boot --no-pager

# 设备树/PWM/串口
python3 tools/board_overlay.py status
ls -l /sys/class/pwm/
ls -l /dev/ttyS4 /dev/ttyS6 /dev/ttyACM0

# 环境
sudo sh deploy/board/install_board_setup.sh    # 再跑一次，应全部 ok

# 软件
py -3 -m unittest discover -s code/test -p "test_*.py"
python3 code/main_robocup.py --config configs/robocup_diffdrive.toml --mode dry-run
```

## 12. 本板转为其他用途时

如果这块板不再跑小车，先停用并禁用服务，避免开机占用串口/PWM：

```sh
sudo systemctl disable --now competition-boot.service robocup-slam.service \
  servo-pwm7-home.service t265-boot-init.timer t265-boot-init.service \
  battery-voltage-monitor.service sound-light-alarm.service \
  rock5a-pwm0-permissions.service
```

仓库和本文档已包含恢复所需的全部配置；不要删除 `/home/radxa/car` 之外的
`robocup_ros`，除非确认新用途不需要它。
