# 双 SG90 物资投放

小车物资只使用 `code/components/payload_servos.py`，旧 LCUS、电磁铁驱动和配置已移除。
摄像头 MG90S 沿用独立 PWM7 和原校准，不能作为物资槽位使用。

| 槽位 | 物资 | 物理 Pin | 设备树节点 | channel | overlay |
|---|---|---|---|---|---|
| 0 | 黄色 | 23 | fd8b0000.pwm | 0 | rk3588-pwm0-m2 |
| 1 | 任务板分配颜色 | 24 | fd8b0010.pwm | 0 | rk3588-pwm1-m2 |
| 摄像头 | MG90S | 28 | febd0030.pwm | 0 | rk3588-pwm7-m0 |

物资 OPEN=0°=1000 μs；CLOSE=90°=1500 μs；周期 20000 μs。
仅在物资组件内将公共角度减 90，再交给原 `ServoAxis`。不改变相机角度换算。
节点选择复用 `resolve_chip()`，不依赖 pwmchip 序号。

监听启动立即 OPEN 两只；有效 RUN 先 CLOSE 两只，再执行原倒计时。
正常投放只 OPEN 对应 slot。STOP、定位失败、任务异常、SIGINT/SIGTERM、任务完成、
服务重启重新监听都 OPEN 两只。打开后持续保持 PWM，不调用 release/disable。
重复 OPEN 允许；一路失败仍尝试另一路，最后报告错误。
`payload_configured()` 只读检查实际节点与写权限，真正写入结果仍以命令返回/异常为准。
PWM 写入成功只代表软件命令完成，没有机械反馈，不能保证物资实际落下。

## 调用链

`car_full_flow_test.main → CompetitionBoot(car_only_test=True) → main_robocup.main
→ competition_task.run_competition_stage('full') → run_full_mission
→ patrol_leg_with_pending_targets → perform_payload_detour
→ payload_detour.run_payload_detour → open_payload(slot)`。

正式入口同一流程，保留无人机协议；单机入口只屏蔽无人机命令，默认真实底盘、视觉和 SG90。
两份物资为 yellow:0、任务分配色:1，任意先后，每种正常释放一次。
释放 False 或异常将停车并传播失败，不生成 release_done/成功 payload_drop。
完整接近、0.5 秒保持、原轴倒退、恢复航向和继续原线段均复用既有实现。
只有单元测试内部显式 `simulate_payload=True` / `simulate_release=True` 才跳过物资 PWM，
此参数默认 False，不由 car_only_test 推导。模拟模式故障收尾也不打开硬件。
非比赛 dry-run/replay/hardware-probe 不构造物资 PWM。

## 现场部署

PC 修改、离线测试、提交并推送后，板端核对分支、HEAD、干净状态及快进关系，
使用 `git pull --ff-only`。板端是部署副本，不能直接开发或强制覆盖。
旧本地 profile 如保留 `[devices.relay]` 会被明确拒绝；部署前在 PC 对该 profile 删除
旧段并保留其余实测参数，按仓库部署约定同步配置，不能直接用示例覆盖实车标定。

先只读查看板端 overlay：

```sh
python3 tools/board_overlay.py list pwm0-m2
python3 tools/board_overlay.py list pwm1-m2
python3 tools/board_overlay.py status
```

仅在确认未启用且操作者同意部署后启用：

```sh
sudo python3 tools/board_overlay.py enable rk3588-pwm0-m2
sudo python3 tools/board_overlay.py enable rk3588-pwm1-m2
```

overlay 重启生效；重启须另经操作者同意。不要在 PC 开发阶段远程改系统配置。

`competition-boot.service` 保留 `car_full_flow_test.py` 入口，增加 root `ExecStartPre=+`
运行 `tools/setup_payload_pwm_permissions.py`。它精确导出上述三路 pwm0，
对各自 export/unexport、通道目录及 period/duty_cycle/enable/polarity 设置 0777，
打印实际路径和最终权限。缺节点或 chmod 失败均非零退出；仅准备权限，不写位置或启用 PWM。
安装器仍替换仓库路径、安装模板并保留比赛服务启用状态。

```sh
sudo sh deploy/install_car_services.sh
sudo systemctl restart competition-boot.service
journalctl -u competition-boot.service -f
```

重启监听会立即驱动两只物资舵机 OPEN，须由操作者确认现场机构可操作后执行。
没有发送 RUN 就不会自行行驶。若内核拒绝 chmod，需在现场验证后调整服务运行身份；
不能将权限准备失败当成功，也不能在未测试 ROS/T265/相机环境时声称 root 方案已验收。

## 静止舵机测试

先由操作者确认机构可操作，再准备三路权限；无需启动完整任务：

```sh
sudo python3 tools/setup_payload_pwm_permissions.py
python3 tools/payload_servo_test.py open all
python3 tools/payload_servo_test.py close all
python3 tools/payload_servo_test.py open 0
python3 tools/payload_servo_test.py open 1
python3 code/main_robocup.py --competition-stage drop
```

close 仅支持 all；正常退出保持最后 PWM。直接 drop 默认 OPEN 两只，False 非零退出。
相机/融合运动测试见 `CENTER_TARGET_ROUTE_TEST.md`，不再提供旧继电器选项。

资料：[Radxa ROCK 5A 引脚表](https://docs.radxa.com/en/rock5/rock5a/hardware-design/hardware-interface)、
[40-PIN 功能测试](https://docs.radxa.com/en/rock5/rock5a/getting-started/interface-usage/pin-40-test)、
[Radxa overlays](https://github.com/radxa-pkg/radxa-overlays)。
