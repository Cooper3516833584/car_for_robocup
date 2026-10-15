# 双 SG90 改造离线验收（2026-10-15）

基线：main `23339871e73f1ad05fe8b0df485fb3a2567dacb3`；GitHub main 复核为同一版本。
本报告记录 PC 软件验证，不代表真机舵机或 sysfs 权限验收。

## 已落实行为与关键调用链

- 物资组件精确绑定 fd8b0000.pwm/0、fd8b0010.pwm/0；物资角度减 90 后交给原 ServoAxis。
  Fake PWM 实际收到 OPEN=1000 μs、CLOSE=1500 μs，周期 20000 μs。
  相机 ServoAxis/PWM7 的映射和控制未改。
- `CompetitionBoot.start → OPEN both → listen`；有效 RUN 在原 payload_action_lock 内 CLOSE both。
  原倒计时、蜂鸣、SLAM 和无人机协议/STOP 转发机制保留。
- `car_full_flow_test → CompetitionBoot(car_only_test=True) → main_robocup.main →
  run_competition_stage('full') → run_full_mission → patrol_leg_with_pending_targets →
  perform_payload_detour → run_payload_detour → open_payload(slot)`。
  car_only_test 仅屏蔽无人机。显式内部 simulate_payload/simulate_release 默认 False，与单机模式独立。
- yellow:0、assigned:1；任意检测顺序，每种正常释放一次。成功输出仅针对当前槽位；STOP、
  定位失败、异常、信号退出、任务完成、重新监听都全开并保持 PWM。
- `_attempt` 明确 False 为失败，None 仍兼容 alarm/stop。RUN CLOSE 失败不启动任务；
  detour OPEN False/异常停车并传播失败，不记录 release_done 或成功 payload_drop；
  两种 drop stage 也检查实际返回结果。日志只确认 PWM 命令写入，没有机械反馈结论。
- 缓存轴失效时重新构造；重复 OPEN 强制输出。一路错误仍尝试另一路并汇总；不 release/disable。
- 旧 LCUS、电磁铁槽位映射、runtime relay 生命周期、配置/工厂、导出和继电器自测彻底删除。
  相机/融合导航测试保留；历史 run_ch3_drop_test.py 改用真实 SG90 槽位 0/1，支持显式模拟。
- service 仍指向 car_full_flow_test.py；root ExecStartPre 只导出项目三路 PWM 并设 0777，
  不写角度/脉宽/enable。缺 overlay、导出失败、chmod 失败/未生效均报错；安装器原路径替换继续覆盖新行。
- Pure Pursuit、三段路线坐标、雷达/T265/SLAM、OCR/YOLO 算法、配额和 HC-15 协议未修改。

## 验证结果

在 PC 仓库根目录执行：

```sh
python -m compileall -q code tools
python -m unittest discover -s code/test -p 'test_*.py'
git diff --check
```

- compileall：通过。
- 全量 956 项：949 通过，7 跳过，0 失败/错误；日志 `logs/sg90_full_suite.txt`。
  跳过项为 Windows 无创建符号链接权限的原 HAL PWM 测试；不是 SG90 逻辑跳过。
- 最后补充返回注解/失败日志后重跑物资 PWM 10 项全通过，日志 `logs/sg90_pwm_pulses.txt`。
- diff --check：通过。
- 活动 code/tools/configs 搜索 payload_task、relay_lcus、prepare_payload、drop_payload、
  build_relay、relay_port、release_mode：无引用。旧 devices.relay 只在拒绝过期配置的测试中出现。
  有日期的历史现场报告/日志保留原始事实，不作为当前操作入口。
- 日志包含故意注入的 PWM/定位/后端异常，是故障路径验收；最终 unittest 状态为 OK。

覆盖：实际脉宽与持续输出、单槽/双槽、重复 OPEN、首路失败仍尝试次路、并发不交错、
关闭缓存轴恢复；监听启动/RUN/STOP/worker 失败/完成全开及下一次 RUN；单机无无人机且真实 PWM；
两色先后顺序、三边实长路线、显式模拟不碰硬件；释放 False 不倒退不报成功；SIGTERM 收尾；
权限三节点精确解析/导出等待/0777/失败传播（临时目录与 mock）；独立 CLI 返回值与保持输出。
所有单元测试为 fake/mock/临时目录；未操作真实 PWM、串口、底盘或无人机。

## 待现场验证和部署状态

只读 SSH 连接 10.202.217.163:22 超时，未能读取 PWM overlay 或板端当前 Git 状态。
本次未修改板端配置、未启用 overlay、未重启服务/板卡、未操纵真机。
现场仍需核对 dtbo 是否存在/启用，实际 sysfs 能否 chmod 0777，三路 PWM 是否均可写，
实测 50 Hz 下 1000/1500 μs，以及真实机构动作、STOP/失败/完赛全开与定位/路线精度。
旧本地 profile 如含 devices.relay 需在 PC 删除旧段并保留实测标定，不能直接覆盖成示例。
部署与独立测试步骤见 [PAYLOAD_SG90.md](PAYLOAD_SG90.md)。
重启新监听会立即使两个 SG90 OPEN，应在操作者明确确认机构可操作后执行；重启板卡另需同意。

## 修改文件清单

状态 M=修改，A=新增，D=删除。配置本地忽略副本不纳入 Git；示例配置已更新。

```text
M	AGENTS.md
M	README.md
M	code/car_full_flow_test.py
M	code/competition_boot.py
M	code/competition_task.py
M	code/components/__init__.py
M	code/components/payload_servos.py
D	code/components/payload_task.py
D	code/components/relay_lcus.py
M	code/config/mission_profile.py
M	code/config/v2_factory.py
M	code/config/v2_loader.py
M	code/config/v2_models.py
M	code/main_robocup.py
M	code/payload_detour.py
M	code/robocup_runtime.py
M	code/target_patrol.py
D	code/test/relay_selftest.py
M	code/test/test_ch3_drop_test.py
M	code/test/test_competition_boot.py
M	code/test/test_competition_direct_stages.py
M	code/test/test_competition_task.py
M	code/test/test_config_v2.py
M	code/test/test_drop_target_vision.py
M	code/test/test_main_patrol_integration.py
M	code/test/test_payload_demo.py
M	code/test/test_payload_detour.py
M	code/test/test_payload_servos.py
D	code/test/test_payload_task.py
M	code/test/test_relative_slam_task.py
D	code/test/test_relay_lcus.py
D	code/test/test_relay_runtime.py
D	code/test/test_route_startup.py
M	code/test/test_slam_manual_accuracy.py
M	configs/robocup_diffdrive.example.toml
M	deploy/competition-boot.service
M	docs/CAR_FULL_FLOW_TEST.md
M	docs/CENTER_TARGET_ROUTE_TEST.md
M	docs/CH3_LANE_SAFE_20261011.md
M	docs/COMPETITION_TASK.md
M	docs/HARDWARE_PORTING.md
M	docs/INTEGRATION_ACCEPTANCE.md
M	docs/LEGACY_CLEANUP.md
D	docs/RELAY_LCUS.md
M	tools/closed_loop_motion.py
M	tools/run_center_target_route.py
M	tools/run_ch3_drop_test.py
M	tools/run_yellow_payload_demo.py
M	tools/slam_manual_accuracy.py
M	tools/turn_forward_diag.py
A	code/test/test_payload_pwm_tools.py
A	docs/PAYLOAD_SG90.md
A	tools/payload_servo_test.py
A	tools/setup_payload_pwm_permissions.py
A	docs/SG90_MIGRATION_20261015.md
```
