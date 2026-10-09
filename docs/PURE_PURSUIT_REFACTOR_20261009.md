# Pure Pursuit 统一运动控制改造记录（2026-10-09）

已按任务包 IMPLEMENTATION.md 的 Phase 0–8（含 6.5）完成本地实施和离线验收。未连接实车、未刷固件、未部署到 ROCK 5A、未推送远端。实车精度与 C10B 制动/换向能力仍待验证。

## 基线与配置

- 仓库 `car`，分支 `main`；初始工作区干净，HEAD 为 `d0931855f15c73869403178d05053d026bf4c89f`；`git pull --ff-only` 返回 Already up to date。
- 实际本地 profile 使用 `differential_vx_vz`；物理主动轮距 **0.198 m**，协议换算轮距 **0.164 m**，未混用或修改。
- 唯一内核使用现有 `navigation.lookahead_m`；示例与本地忽略的实测配置改为 **0.20 m**。主线速度最多 0.15 m/s，可见目标沿用 0.08 m/s；侧向投放恢复 travel_drive 上限。
- `configs/robocup_diffdrive.toml` 原本被 Git 忽略，其 lookahead 改动仅存在本机；部署时须核对实际生效 profile。示例配置仍保留未实测占位尺寸及原协议，未将其冒充实车配置。

## 分阶段落实

| 阶段 | 行为与验收 | 提交 |
|---|---|---|
| 0 | 隔离 stop 异常；原始失败不丢失，断磁/蜂鸣器/资源清理继续；基线 888 项，投放故障回归 22 项通过 | `07ccdda` |
| 1 | differential_navigation.compute_line 唯一平移数学内核；参考包 10 项及实际项目内核 10 项通过 | `ed326bc` |
| 2 | 新增 global/local line；删除旧 distance/segment/recovery/追终点公式；动作回归 9 项通过 | `3da8b44` |
| 3 | 局部直线/局部转向/相对转向直接使用经 freshness、confidence、continuity 校验的 T265；无 fused 回退；运行时 18 项通过 | `9f83964` |
| 4 | 280/420/250 原始几何端点及绝对 90°/180° 朝向保持；恢复按最新投影初始化；路线 10 项通过 | `8b40342` |
| 5 | 投放全程同一 odom frame，固定 road_yaw、名义 side_yaw，回程严格复用 B2/A2；测试迁移修正后 22 项通过 | `62d5b69`, `7a0b226` |
| 6 | 正式比赛 lane/search/alignment 迁移；8/2 cm 使用 fresh T265 构造局部线，负距离明确 reverse；比赛 53 项通过 | `e7cfd48` |
| 6.5 | 删除 payload_demo 执行器、factory 与参数；录像仍默认黄色、slot=2/CH3，可显式覆盖；入口初始 3 项通过 | `c633a5b` |
| 7 | 统一 TurnController：刹停角速度约束，角误差≤3°且真实 T265 yaw_rate≤3°/s 连续 4 个新样本才完成；重复样本不计数，越界纠正固定目标；新增 6 项通过 | `b77cac8` |
| 8 | 释放前位置和线/角速度复核；5 mm 沿线、3 cm 终点横向检查，连续四帧≤0.02 m/s、≤3°/s；停后漂移禁止释放；投放 24 项通过 | `0f0384b` |
| 收尾 | CLI 原始错误落盘、清理失败不得写成功；诊断调用迁移，旧独立运动执行器删除停用；补齐限速和终点日志、50 ms 仿真 | `5ce4567`, `62ff411`, `4bec121` |

端点横向 3 cm 是本轮停止/释放检查边界，**不是赛道宽度或实际投放精度合格声明**。全局直线导航沿用现有相对目标语义，没有增加避障；坐标仍是未实测占位值，调用者必须确认直线可行驶。

## 删除和停用

- 删除 MotionActionType.FOLLOW_SEGMENT / DRIVE_DISTANCE / DRIVE_TO / FACE_POINT 及对应方法、旧 atan 横向补偿、强纠偏/recovery、二维终点回追、固定 startup yaw bias。
- navigate_to / navigate_to_pose 保留语义，平移仍调用 compute_line；末端航向调用同一 TurnController。
- 删除 `_distance_control_T_t265` 冻结对齐及每段 fused->local 重建。保留 D500_GLOBAL_PENDING、定位等待、任务中止、底层轮速/加速度/看门狗保护。
- 删除 payload_demo.py、build_payload_demo_session 和 `--demo-no-position-checks` 开环运行分支。
- `closed_loop_motion.py`、`turn_forward_diag.py` 已改为新动作；旧 basic_motion / square_run / drive_calibration pulse / drive_actuation_probe / auto_motion_diag live / yaw_wall_crosscheck direct-serial 运动执行器删除并停用，入口在打开硬件前返回失败。历史数学与日志分析保留；请使用正常闭环工具开展新运动测试。
- 历史日志不变。关键词检查仅命中删除断言、历史日志测试及 PoseFusion 历史注释，没有旧可执行调用。

## 最终离线验证

运行命令与实际结果见 `PP_OFFLINE_TEST_RESULTS_20261009.txt`：

- `py -3 -m compileall -q code tools`：通过。
- `py -3 -m unittest discover -s code/test -p "test_*.py"`：**870 项，OK，跳过 7 项**。
- `git diff --check`：通过。
- 参考包数学测试：10 项通过。
- `py -3 tools/simulate_pure_pursuit.py`：四组 50 ms 仿真通过，摘要见 `PP_SOFTWARE_SIMULATION_20261009.json`。

2.8 m、初始 y=±8 cm，两组各 411 步（20.55 s），全程正向；47 cm 注入前 0.5 s 共 5° yaw 扰动，72 步到达；B2->A2 倒车复用原端点，66 步到达。仿真同时断言平移时双轮同向和速度方向。理想运动模型的横向收敛数值不能代替地面测量。

旧实现及停用诊断的行为断言已换成新语义回归，测试数量与基线不能简单比较。完整测试日志中的部分 ERROR 是预期故障注入输出，最终 unittest 结果为 OK。

## 待上车（未验收）

1. 0.5–1 m 直线，以尺量/俯视视频核对真实轨迹。
2. 左90/右90停稳后前进，各多次记录左右轮目标/反馈与 T265 yaw；重点验证换向迟起和约 9.6° 停车余转。
3. 3/5 cm 初始侧偏，确认只向前圆弧入线，不追身后历史点。
4. 完整 47 cm 往返：名义侧向、释放前静止、原线返回、固定道路朝向；测量投放口几何误差。
5. 280/420/250 全路线与 YOLO 触发恢复；按赛题地面尺度核对终点/投放精度。
6. 先明确 C10B 硬件失联保护，再设计故障试验；不得直接拔 USB 假定上位机还能发送 STOP。

Pure Pursuit 不会修复 C10B 单轮迟起、USB 写失败或真实制动力；低速 0.045 m/s 下限、终点边界及停稳阈值需实测。内部定位和日志均不是独立地面真值。

## 变更文件

- `code/center_target_route.py`
- `code/competition_task.py`
- `code/components/basic_motion_controller.py`
- `code/components/differential_navigation.py`
- `code/config/v2_factory.py`
- `code/payload_demo.py`
- `code/payload_detour.py`
- `code/robocup_runtime.py`
- `code/test/test_basic_motion.py`
- `code/test/test_basic_motion_controller.py`
- `code/test/test_basic_motion_startup.py`
- `code/test/test_center_target_route.py`
- `code/test/test_closed_loop_motion.py`
- `code/test/test_competition_task.py`
- `code/test/test_differential_navigation.py`
- `code/test/test_drive_calibration.py`
- `code/test/test_payload_demo.py`
- `code/test/test_payload_detour.py`
- `code/test/test_pure_pursuit_line.py`
- `code/test/test_relative_slam_task.py`
- `code/test/test_robocup_runtime.py`
- `code/test/test_square_run.py`
- `code/test/test_task_board_startup.py`
- `code/test/test_turn_forward_diag.py`
- `configs/robocup_diffdrive.example.toml`
- `tools/auto_motion_diag.py`
- `tools/basic_motion.py`
- `tools/closed_loop_motion.py`
- `tools/drive_actuation_probe.py`
- `tools/drive_calibration.py`
- `tools/run_center_target_route.py`
- `tools/run_yellow_payload_demo.py`
- `tools/simulate_pure_pursuit.py`
- `tools/square_run.py`
- `tools/turn_forward_diag.py`
- `tools/yaw_wall_crosscheck.py`

数学核心适配自用户任务包独立编写的 reference/pp_line_core.py；未直接复制 Nav2/PythonRobotics 的实质源码，未新增 ROS/Nav2/MPC 依赖。
