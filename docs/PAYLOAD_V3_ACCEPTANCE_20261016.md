# 五份小车 / 一份无人机：离线验收（2026-10-16）

## 实际变更

小车基线 `d99f22113920305e31a04a4eda7de8256014433f`，已重新 fetch 核对远端 main。

- 相机仍为 Pin28 / PWM7 / febd0030.pwm / channel0，原配置、ServoAxis、build_servo、homing 工具和服务没有修改。
- slot4 独占 Pin27 / PWM6_M0 / febd0020.pwm / channel0。五路物资保持20ms周期、OPEN=1000μs、CLOSE=1500μs、payload_angle-90映射。
- 同一 payload_detour.py 的 right_mount：左90 → 融合前进32cm → 左90 → 放slot4并保持 → 右90 → 按最新融合位姿的侧轴投影倒退 → 右90。第二次左转后不前进；先恢复侧轴航向才倒退。释放航向日志为 road_yaw+π。
- standard 槽位0～3仍走43cm，双份槽2/3同一次往返。普通模式保留 safe_distance_m 覆盖；slot4固定32cm。
- 车载RGB全部四件及黄色，共四颜色目标/五槽位。两种单份颜色按固定red/blue/green优先级分配slot1和slot4；出现顺序不改变分配，缺失目标记skipped。
- 中央10%一帧触发、减速、EntryLatch、三边路线、融合定位、PP、蜂鸣和ACK顺序保持。单机仍默认真实物资输出。
- 运动/保持周期显式核对融合位姿，覆盖转向首周期丢失位姿的边界；STOP、定位失效及PWM失败沿用停车、异常传播和全OPEN，没有自动回位动作。
- TASK/START v3，两端RGB只接受全零；START独立校验task_id。FleetBus外层和CLOSE/REMAP运行编号仍V1。
- 权限准备精确枚举五物资+原相机共六路，仅export/settle/chmod/核验，不写duty或enable。静止工具支持open4，open5拒绝，close仍只支持all。

## 执行验证

Windows本机使用 Python3.13 的 `python`，对应任务书python3命令。

| 命令（小车仓库根目录） | 结果 |
|---|---|
| `python -m compileall -q code tools` | 退出0，无编译错误 |
| `python -m unittest discover -s code/test -p 'test_*.py'` | 1027项，0失败/0错误，10项沿用环境跳过 |
| `python tools/verify_payload_handoff.py --drone-repo ../drone` | 9组通过：3种任务板×3种ID（1/42/u32上界） |
| `git diff --check` | 通过 |

跳过沿用Windows Unix socket和设备符号链接限制等原条件，没有新增跳过或放宽核心断言。
新增回归覆盖真实控制器融合几何、两个左转/两个右转绝对航向、32cm、转向平移残差、释放朝向、STOP/位姿失效/PWM6故障；原standard用例继续通过。
三种任务板使用不同四色出现顺序，验证固定槽位分配、五次释放及三段原路线续航。集成测试确认相机只有+90°拍摄。
联测使用双方实际帧解析、FleetTaskSender、RescueTaskReceiver/RescueFleetNode及AST提取的机端DropLedger/释放方法；串口、FC、导航、舵机均为假对象。
确认CLOSE/REMAP → TASK COMPLETED → START ACCEPTED，之后仅一次黄色STM32通道3释放，重复尝试被拒绝，v2拒绝。

## 交付边界

仅本地代码、测试、文档和提交；未推送GitHub、未连接板端、未启用overlay、未重启服务/设备、未动车/实际投放/起飞，未声称两端实机联机验收。
用户原有未跟踪 `tools/verify_startup_retries.py` 保留且不纳入本次提交；已有日志不清理。
无人机黄色暂定slot2/STM32通道3，未核实实际装填。

部署前由操作者确认Pin27未被SPI/I2C或其他接线占用，并核对PWM6 overlay可用；启用和重启需现场授权。必须双端同步协议v3后再比赛。
重启后按真实地址分别解析febd0020与febd0030，不按pwmchipN猜测；保留被Git忽略的实测TOML，禁止以example覆盖。
空载静止验证五路独立与相机隔离后，用既有CH3入口 `--payload-slot 4 --simulate-release` 验证融合动作，再经操作者确认装填执行真实投放。模拟仍会动车，也需要现场明确授权。

## 修改文件

- `AGENTS.md`
- `code/competition_task.py`
- `code/components/drop_task_protocol.py`
- `code/components/payload_drop_log.py`
- `code/components/payload_servos.py`
- `code/payload_detour.py`
- `code/test/test_ch3_drop_test.py`
- `code/test/test_competition_boot.py`
- `code/test/test_competition_direct_stages.py`
- `code/test/test_competition_task.py`
- `code/test/test_fleet_task_sender.py`
- `code/test/test_main_patrol_integration.py`
- `code/test/test_payload_detour.py`
- `code/test/test_payload_drop_log.py`
- `code/test/test_payload_pwm_tools.py`
- `code/test/test_payload_servos.py`
- `docs/CAR_FULL_FLOW_TEST.md`
- `docs/COMPETITION_TASK.md`
- `docs/PAYLOAD_SG90.md`
- `docs/PAYLOAD_V3_ACCEPTANCE_20261016.md`
- `tools/payload_servo_test.py`
- `tools/run_ch3_drop_test.py`
- `tools/setup_payload_pwm_permissions.py`
- `tools/verify_payload_handoff.py`

## 资料

[Radxa引脚表](https://docs.radxa.com/en/rock5/rock5a/hardware-design/hardware-interface)列出Pin27 PWM6与Pin28 PWM7；[Radxa overlay清单](https://github.com/radxa-pkg/radxa-overlays/blob/main/arch/arm64/boot/dts/rockchip/overlays/Makefile)包含rk3588-pwm6-m0。
[Linux RK3588 DTS](https://raw.githubusercontent.com/torvalds/linux/master/arch/arm64/boot/dts/rockchip/rk3588-base.dtsi)用于节点对照，实际以板端内核和枚举为准。
