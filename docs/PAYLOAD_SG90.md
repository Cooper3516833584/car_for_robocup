# 五 SG90 物资投放

小车物资只使用 `code/components/payload_servos.py`，旧 LCUS、电磁铁驱动和配置已移除。
摄像头 MG90S 沿用独立 PWM7 和原校准，不能作为物资槽位使用。

| 槽位 | 物资 | 物理 Pin | 设备树节点 | channel | overlay |
|---|---|---|---|---|---|
| 0 | 黄色 | 23 | fd8b0000.pwm | 0 | rk3588-pwm0-m2 |
| 1 | 单件选投目标 | 24 | fd8b0010.pwm | 0 | rk3588-pwm1-m2 |
| 2 | 双件选投目标第 1 件 | 15 | febe0030.pwm | 0 | rk3588-pwm11-m1 |
| 3 | 双件选投目标第 2 件 | 13 | febf0020.pwm | 0 | rk3588-pwm14-m1 |
| 4 | 后一个单件颜色（右侧安装） | 27 | febd0020.pwm | 0 | rk3588-pwm6-m0 |
| 摄像头 | MG90S | 28 | febd0030.pwm | 0 | rk3588-pwm7-m0 |

物资 OPEN=0°=1000 μs；CLOSE=90°=1500 μs；周期 20000 μs。
仅在物资组件内将公共角度减 90，再交给原 `ServoAxis`。不改变相机角度换算。
节点选择复用 `resolve_chip()`，不依赖 pwmchip 序号。

监听启动立即 OPEN 五只；有效 RUN 先 CLOSE 五只，再执行原倒计时。
正常投放只 OPEN 对应 slot。STOP、定位失败、任务异常、SIGINT/SIGTERM、任务完成、
服务重启重新监听都 OPEN 五只。打开后持续保持 PWM，不调用 release/disable。
重复 OPEN 允许；一路失败仍尝试其余各路，最后报告错误。
`payload_configured()` 只读检查实际节点与写权限，真正写入结果仍以命令返回/异常为准。
PWM 写入成功只代表软件命令完成，没有机械反馈，不能保证物资实际落下。

## 调用链

`car_full_flow_test.main → CompetitionBoot(car_only_test=True) → main_robocup.main
→ competition_task.run_competition_stage('full') → run_full_mission
→ patrol_leg_with_pending_targets → perform_payload_detour
→ payload_detour.run_payload_detour → open_payload(slot)`。

正式入口同一流程，保留无人机协议；单机入口只屏蔽无人机命令，默认真实底盘、视觉和 SG90。
任务板红蓝绿必须是 2/1/1 排列。小车承担红蓝绿全部四件，外加黄色一件；两个单件颜色按固定红、蓝、绿优先级分配，不随出现顺序改变。
槽位 yellow:0、首个单件颜色:1、双件颜色:(2,3)、后一个单件颜色:4，总计四个目标/五件。四个目标任意先后，双件目标只接近一次，在同一位置依次打开槽位 2、3，每件保持 0.5 秒并检查停止条件。
无人机仅投隐含黄色一件，RGB 配额全零。任务/START 消息版本升级为 3，双方必须同步更新；CLOSE/REMAP 的运行编号消息仍为版本 1。
释放 False 或异常将停车并传播失败，不生成 release_done/成功 payload_drop。
槽位 0～3 保持原左90 → 43 cm → 放 → 原轴倒退 → 右90。
槽位 4 在同一个 `run_payload_detour()` 内使用 `right_mount`：
左90 → 融合前进32 cm → 左90 → 停车放slot4 → 右90 → 沿保存侧轴投影倒退 → 右90。
第二次左转后不再前进；回到侧轴朝向后才倒车；绝对航向均来自触发时融合 road_yaw。
转向期间临时解除侧轴直线约束，回到 side_yaw 后恢复；回程使用最新融合位姿，
不声称转向平移误差被消除。`payload_drop_pose.target_yaw_rad` 在 slot4 为 road_yaw+π。
完整接近、0.5 秒保持、原轴倒退、恢复航向和继续原线段均复用既有实现。
只有单元测试内部显式 `simulate_payload=True` / `simulate_release=True` 才跳过物资 PWM，
此参数默认 False，不由 car_only_test 推导。模拟模式故障收尾也不打开硬件。
非比赛 dry-run/replay/hardware-probe 不构造物资 PWM。

## 现场部署

PC 修改、离线测试、提交并推送后，板端核对分支、HEAD、干净状态及快进关系，
使用 `git pull --ff-only`。板端是部署副本，不能直接开发或强制覆盖。
旧本地 profile 如保留 `[devices.relay]` 会被明确拒绝。经操作者授权后，在板端使用
PC 开发、测试并提交的部署迁移工具清理该段：

```sh
python3 tools/retire_payload_relay_config.py
```

该工具先在 `logs/config-backups/` 保存并核验原文件，比较解析结果确认仅删除 relay。
其余实测参数保留，不能直接用示例覆盖实车标定；不在板端开发修改工具代码。

新增 Pin27 前须核对 SPI0_MISO_M0 / I2C6_SDA_M0 及现有接线没有占用。
相机始终保留 Pin28 / febd0030.pwm / PWM7，禁止禁用或迁移其 overlay。
重启后用真实设备地址分别解析 PWM6 和 PWM7，不能按 pwmchipN 序号猜测。
先只读查看板端 overlay：

```sh
python3 tools/board_overlay.py list pwm0-m2
python3 tools/board_overlay.py list pwm1-m2
python3 tools/board_overlay.py list pwm11-m1
python3 tools/board_overlay.py list pwm14-m1
python3 tools/board_overlay.py list pwm6-m0
python3 tools/board_overlay.py status
```

仅在确认未启用且操作者同意部署后启用：

```sh
sudo python3 tools/board_overlay.py enable rk3588-pwm0-m2
sudo python3 tools/board_overlay.py enable rk3588-pwm1-m2
sudo python3 tools/board_overlay.py enable rk3588-pwm11-m1
sudo python3 tools/board_overlay.py enable rk3588-pwm14-m1
sudo python3 tools/board_overlay.py enable rk3588-pwm6-m0
```

overlay 重启生效；重启须另经操作者同意。不要在 PC 开发阶段远程改系统配置。

`competition-boot.service` 保留正式 `competition_boot.py` 入口，增加 root `ExecStartPre=+`
运行 `tools/setup_payload_pwm_permissions.py`。它精确导出五物资加一相机共六路 pwm0，
对各自 export/unexport、通道目录及 period/duty_cycle/enable/polarity 设置 0777，
打印实际路径和最终权限。缺节点或 chmod 失败均非零退出；仅准备权限，不写位置或启用 PWM。
本车 Radxa udev 规则会在 PWM add/change 事件中统一重设为 0775；脚本先完成六路导出，
再有界等待 udev 事件处理完毕，最后统一设置并核验六路 0777，避免启动事件覆盖结果。
安装器仍替换仓库路径、安装模板并保留比赛服务启用状态。

```sh
sudo sh deploy/install_car_services.sh
sudo systemctl restart competition-boot.service
journalctl -u competition-boot.service -f
```

重启监听会立即驱动五只物资舵机 OPEN，须由操作者确认现场机构可操作后执行。
没有发送 RUN 就不会自行行驶。若内核拒绝 chmod，需在现场验证后调整服务运行身份；
不能将权限准备失败当成功，也不能在未测试 ROS/T265/相机环境时声称 root 方案已验收。

## 静止舵机测试

先由操作者确认机构可操作，再准备六路权限；无需启动完整任务：

```sh
sudo python3 tools/setup_payload_pwm_permissions.py
python3 tools/payload_servo_test.py open all
python3 tools/payload_servo_test.py close all
python3 tools/payload_servo_test.py open 0
python3 tools/payload_servo_test.py open 1
python3 tools/payload_servo_test.py open 2
python3 tools/payload_servo_test.py open 3
python3 tools/payload_servo_test.py open 4
python3 code/main_robocup.py --competition-stage drop
```

close 仅支持 all；正常退出保持最后 PWM。直接 drop 默认 OPEN 五只，False 非零退出。
相机/融合运动测试见 `CENTER_TARGET_ROUTE_TEST.md`，不再提供旧继电器选项。

资料：[Radxa ROCK 5A 引脚表](https://docs.radxa.com/en/rock5/rock5a/hardware-design/hardware-interface)、
[40-PIN 功能测试](https://docs.radxa.com/en/rock5/rock5a/getting-started/interface-usage/pin-40-test)、
[Radxa overlays](https://github.com/radxa-pkg/radxa-overlays)。

新增 PWM6 的节点为 febd0020.pwm；PWM11/PWM14 的设备树地址来自 Linux rk3588-base.dtsi；Pin 13/15 按 ROCK 5A x1.2/v1.1 引脚表和本次操作者确认配置。
本次仅完成本地代码及无硬件验证，未启用 overlay、部署或驱动新增舵机；现场须核验引脚复用、PWM 权限、独立装载和实际释放。

当前五路/v3验证结果见 [PAYLOAD_V3_ACCEPTANCE_20261016.md](PAYLOAD_V3_ACCEPTANCE_20261016.md)。

## 历史阶段记录：2026-10-16 四路物资版本（已被五路版本取代）

- `py -3 -m compileall -q code tools`：通过。
- `py -3 -m unittest discover -s code/test -p 'test_*.py'`：979 项，10 项按环境跳过，其余通过。
- 最后收紧测试断言后，`test_competition_task.py` 68 项与 `test_payload_servos.py` 10 项再次通过。
- 与无人机实际 FleetBus 编解码、任务接收器进行 9 组模拟 TASK/START/ACK 联测，覆盖三种任务牌排列和任务编号边界，均通过。
- `git diff --check`：通过。仅本地修改，未提交、推送、部署或启动硬件。

本次修改文件（相对本仓库）：

- `AGENTS.md`
- `code/competition_task.py`
- `code/components/drop_task_protocol.py`
- `code/components/payload_servos.py`
- `code/payload_detour.py`
- `code/test/test_competition_direct_stages.py`
- `code/test/test_competition_task.py`
- `code/test/test_fleet_task_sender.py`
- `code/test/test_main_patrol_integration.py`
- `code/test/test_payload_detour.py`
- `code/test/test_payload_pwm_tools.py`
- `code/test/test_payload_servos.py`
- `docs/PAYLOAD_SG90.md`
- `tools/payload_servo_test.py`
- `tools/setup_payload_pwm_permissions.py`
