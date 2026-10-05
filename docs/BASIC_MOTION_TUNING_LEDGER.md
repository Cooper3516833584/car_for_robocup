# BasicMotionController 闭环调参台账

依据 `docs/BASIC_MOTION_CLOSED_LOOP_TUNING_PLAN.md` 执行。本文件是该计划要求的调参台账：
每次改动一行，保留旧值、新值、依据和复测结果。未复测的候选值不写成已验收值。

## 会话信息

| 项目 | 值 |
| --- | --- |
| 日期 | 2026-10-04 |
| 目标主机 | `radxa@10.82.117.163`（rock-5a, aarch64） |
| 板端仓库 | `/home/radxa/car`，`main` @ `792d909`（与本机 HEAD 一致） |
| 板端保留改动 | `code/test/battery-voltage-monitor.service`、`code/test/sound-light-alarm.service`（已修改）、`tools/battery_alarm_chain_test.py`（未跟踪） |
| 板端实车 TOML | `configs/robocup_diffdrive.toml`，sha256 `f8c27a23c6da5d12126017b6a7bee1b3cb125a217118cc88615b4e63c3b06984` |
| 日志根目录 | `/home/radxa/car_test_logs/closed_loop_motion_20261004_03/` |
| 运动入口 | `tools/closed_loop_motion.py`（板端已存在，17229 B，与仓库一致） |
| 新增分析工具 | `tools/closed_loop_analyze.py`（阶段 2 离线指标 + 有效性判定）、`tools/c10b_telemetry_probe.py`（只读遥测探针） |
| 新增无硬件测试 | `code/test/test_closed_loop_analyze.py`（13 项）、`code/test/test_closed_loop_motion.py` 补全至 18 项 |

## 生效配置（未改动前的基线）

```
drive_track_width_m      = 0.247
firmware_track_width_m   = 0.164
protocol_mode            = differential_vx_vz
max_linear_speed_m_s     = 0.20     (闭环入口再降到 0.10)
max_angular_speed_rad_s  = 0.30     (闭环入口再降到 0.15)
max_linear_accel_m_s2    = 0.15
position_tolerance_m     = 0.03
yaw_tolerance_rad        = 0.0523598776   (3°)
slowdown_distance_m      = 0.50
path_yaw_gain            = 1.5
final_yaw_gain           = 1.5
```

相对 SLAM profile 由 `config/relative_slam_profile.py` 在内存中强制：`backend=slam_toolbox`、
`enable_wall_absolute=false`、`require_field_anchor=false`、`relative_goals_only=true`。
D500 固定墙/比赛地图占位值不参与定位，`navigation.map`、`navigation.footprint` 只被忽略地加载。

## 阶段 0 结果

1. 只读核对通过。板端实车 TOML、协议模式、限速、外参已读取，见上。
2. 无硬件回归：板端 `python3 -m compileall -q code tools` 通过；
   `python3 -m unittest discover -s code/test -p "test_*.py"` → **Ran 752 tests, OK (skipped=8)**。
   加入 `test_closed_loop_analyze.py` 与补全后的 `test_closed_loop_motion.py` 后本机为
   **Ran 778 tests, OK (skipped=8)**。
3. `tools/closed_loop_motion.py` 参数守卫实测通过：拒绝示例配置、拒绝缺少三项确认、
   拒绝 `--cm 0` 与 `--cm 60`。`code/test/test_closed_loop_motion.py` 在回归内。
   本轮补全计划要求的无硬件终止/安全路径覆盖（该文件 5 → 18 项）：

   | 计划条目 | 覆盖 |
   | --- | --- |
   | 参数解析、一次性动作、目标坐标计算 | 原有 5 项 |
   | 状态终止 | `BLOCKED`/`POSE_LOST`/`SAFE_STOPPED`/`ERROR` 四态均抛错 |
   | 定位失效 | 动作中融合位姿丢失、`mission ERROR` 在提交前抛错 |
   | 超时 | 动作到点超时、停稳窗口超时 |
   | 操作者中断 | `abort()` 在提交动作前生效 |
   | 边界保护 | 单步位姿跳变、离开 0.75 m 有界区域 |
   | 重复启动拒绝 | 单次运行只提交一条动作 |
   | 异常停车 | 动作失败时 `request_safe_stop` + `close`；启动失败（`build_runtime` 抛错）时关闭日志器 |
   | 日志计算 | `code/test/test_closed_loop_analyze.py` 13 项 + 丢事件判为无效 |
4. 静止融合预检（`tools/test_live_relative_pose.py --seconds 32`，无电机）：
   位姿漂移 ≤ `x 0.01 cm / y 0.01 cm / yaw 0.01°`，T265 置信度 0.67，
   D500 扫描 4.0–5.1 Hz，`dropped_events=0`，`log_write_error=null`。
   日志：`.../closed_loop_motion_20261004_03/preflight_static/`。

> 说明：`preflight_static` 末尾打印过一次 `INVALID T265/scan TF alignment p95 >= 30 ms`。
> 这是 `test_live_relative_pose.py` 自带的显示健康启发式，`closed_loop_motion.py` 用的
> `drive_calibration.accepted_fused_sample()` 并不包含该 TF 门限，因此不构成本轮动作的阻断条件。

## 阶段 1/2：基线动作与诊断

### 动作 1 —— `drive-distance +20 cm`（基线配置，实测超时）

命令：
```sh
python tools/closed_loop_motion.py --config configs/robocup_diffdrive.toml \
  --output .../s1_drive_fwd_20cm \
  --confirm-motor-test --confirm-area-clear --confirm-estop-ready drive-distance --cm 20
```

| 量 | 值 |
| --- | --- |
| 结果 | `TimeoutError: single action exceeded its deadline`（30 s 上限），`valid=false` |
| 阶段 | 全程停留在 `initializing`，未进入 `aligning` |
| 指令积分 | 线性 **1.1834 m**，角 **0.1993 rad** |
| 限幅后命令 | `max_limited_v = 0.0419 m/s`，`max_limited_ω = 0.1229 rad/s` |
| 融合位姿位移 | 纵向 **+0.0102 m**，横向 −0.0092 m，偏航 +0.549° |
| T265 原始位移 | **0.0136 m**，偏航 +0.577° |
| 融合跳变 | 最大单样本步长 **0.0026 m**（无跳变），>2 cm 步数 0 |
| SLAM 锚点新息 | 最大 **0.0 m**，非零更新 0 次 |

诊断（阶段 2）：本轮**没有定位问题**——融合与 T265 一致，无跳变、无锚点新息。
真正的原因是**驱动没有输出直线运动**：指令积分 1.1834 m，实际推进约 0.01 m（约 0.8%）。

补充：`slowdown_distance_m = 0.50` 大于 20 cm 目标，`_line_command` 从
`0.10 × (0.20/0.50) = 0.04 m/s` 起，到 3 cm 容差时降到 `0.006 m/s`。
这一条会让短距离动作在低速区爬行，是次要因素，但**不是**本轮零位移的主因：
本轮 1.18 m 的指令积分里绝大部分都落在 0.02–0.04 m/s 区间，车仍几乎不动。

### 对照：上一轮（07:32–07:51，同一入口，配置旧哈希）同类动作

| 量 | `..._02/drive_distance_forward_20cm_tape` |
| --- | --- |
| 结果 | 同样 `TimeoutError`，但已进入 `aligning` |
| 指令积分 | 线性 0.5709 m，角 2.0128 rad |
| 融合位移 | 纵向 +0.3746 m，偏航 +122.978° |
| 融合跳变 | 最大单样本 **0.0741 m**（t=27.91 s，等效 1.387 m/s） |
| SLAM 锚点新息 | 最大 **0.21 m**，非零更新 233 次 |

诊断：该轮在 t=27.91 s 接受了一个 **0.21 m 的 SLAM 锚点新息**，一步把融合位姿前拉 7.4 cm
（`ANCHOR_BLEND≈0.35`），随后按 0.65 的几何衰减收敛。控制器因此误判“已过冲 18 cm”，
进入 `aligning` 以 0.15 rad/s 原地掉头，30 s 内无法收敛。
**这是定位侧的跳变问题，与本轮（03）的驱动问题不同，两者必须分开处理。**

### 动作 2 —— 开环直线 `distance 50 cm @ 0.10 m/s`（交叉验证）

| 量 | 值 |
| --- | --- |
| 结果 | `TimeoutError: no measured progress for 3 seconds`（工具主动中止） |
| 限幅后命令 | 0.1000 m/s 稳定保持 |
| 实测推进 | ≈ **0.000 m**（`measured` 在 −0.007…+0.004 m 之间抖动） |
| 日志 | `.../s3_openloop_fwd50_v010.csv`、`.../s3b_openloop_fwd50_v010.csv` |

### 动作 3 —— 开环原地旋转 `rotate 30° @ 0.15 rad/s`（对照）

| 量 | 值 |
| --- | --- |
| 目标 / 预测 / 实测 | +30.000° / +30.029° / **+20.297°** |
| 带符号误差 | **−9.703°** |
| 旋转中心漂移 | **0.152 m**（远超开环 5 cm 筛查值） |
| 日志 | `.../s3c_openloop_rot30.csv` |

**结论：角速度（Vz）通道生效，线速度（Vx）通道不产生位移。**

### 硬件侧交叉验证：C10B 串口遥测

用只读探针 `tools/c10b_telemetry_probe.py` 在发指令的同时抓 24 字节遥测帧：

| 观察 | 结果 |
| --- | --- |
| 帧率 | 25–29 Hz，校验全部通过 |
| `motor_disabled` | 465/465 帧均为 **False**（固件认为电机使能） |
| 电池 | **11.01 – 11.13 V**（现场服务阈值 `--threshold-v 11.0`） |
| 静止时遥测 | 线速度字段 = 0 |
| 发 0.10 m/s 指令时 | 线速度字段按加速度斜坡 13 → 26 → 53 → 67 → 80 → 107 mm/s 跟随 |

**结论：串口链路、11 字节 Vx/Vz 帧编码、固件收帧与加速度整形都正常。**
问题在固件之后（供电 / 电机 / 机械 / 接地）。

### 与 2026-10-03 开环数据的历史对比

`/home/radxa/car_test_logs/drive_calibration_20261003_codex/` 的停稳读数：

| 文件 | 目标 | 速度 | `k_v` |
| --- | --- | --- | --- |
| `drive_fwd_50_features_2.csv` | +0.500 m | 0.100 m/s | **+0.952** |
| `fwd_50cm_after_rotation.csv` | +0.500 m | 0.100 m/s | **+0.876** |
| `drive_fwd_10_formal_100mm_b.csv` | +0.100 m | 0.100 m/s | +0.004 |
| `drive_fwd_10_formal_3.csv` | +0.100 m | 0.050 m/s | −0.001 |
| `fwd_startup_10cm_after_timestampfix.csv` | +0.100 m | 0.050 m/s | −0.029 |
| `rotate_left_90_after_startup.csv` | +90° | 0.200 rad/s | +0.826 |

2026-10-03 时 50 cm @ 0.10 m/s 能交付 88–95%，说明直线通道当时是通的；
10 cm 短脉冲当时就已经交付≈0。**因此今天是相对 10-03 的退化，不是一直如此。**

注意：`ackermann_firmware_compat` 与 `differential_vx_vz` 只差**软件侧的**
最小转弯半径检查。纯直线（Vz=0）命令在两种模式下发给 C10B 的 11 字节帧**完全相同**，
所以本次退化不能用“改了协议模式”解释。

### 定位侧缺陷：SLAM 锚点新息跳变（与驱动问题分开处理）

代码事实（`code/components/pose_fusion.py`）：

| 常量 | 值 | 含义 |
| --- | --- | --- |
| `D500_MAX_INNOVATION_M` | 0.30 | 锚点新息门限；**不超过 30 cm 就被接受** |
| `D500_MAX_INNOVATION_YAW_RAD` | 12° | 锚点偏航新息门限 |
| `ANCHOR_BLEND` | 0.35 | 每次接受的锚点按 35% 融合 |
| `SLAM_LOOP_MAX_M` | 1.0 | 回环迁移门限 |

因此单帧 slam_toolbox 位姿只要与 T265 航迹推算差 ≤30 cm，就会通过门限，
并以 `0.30 × 0.35 = 0.105 m` 的幅度**在单个 50 ms 采样内**拉动融合位姿。
`..._02/drive_distance_forward_20cm_tape` 实测到 0.21 m 新息、7.4 cm 单步跳变，
正是这条路径。在 ±1 cm 停稳验收尺度下，这是**单次样本级**的致命扰动：
一旦发生在动作末段或停稳窗口内，该次动作无效。

`slam_loop_toolbox` 的 `LoopClosureEvent` 在板端不可用（启动日志
`slam_toolbox LoopClosureEvent unavailable; large anchors remain gated`），
所以回环一致性通道也不作为兜底。

处理原则（按计划）：

1. 这类动作一律标为无效，**不得**计入通过，也不得用平均掩盖。
2. 先按“融合位姿 → 控制命令 → 驱动限幅 → 车轮响应”顺序查因，不在驱动调参里夹带定位改动。
3. 收紧 `D500_MAX_INNOVATION_M` / `ANCHOR_BLEND` 属于**代码常量**改动，不是 TOML 参数，
   须单独一轮、带回归测试并记录旧值/新值，不能与增益调参混在同一轮。

`tools/closed_loop_analyze.py` 现在会为每次动作输出 `validity.action_valid` 与
`invalid_reasons`，把上面的“标为无效”规则自动化：融合单步位移 ≥10 cm、单步偏航 ≥20°、
融合状态离开健康集合、`source_flags` 缺少 `t265/slam/fused`、日志丢事件或写失败
都会判为无效；新息 >3 cm、单步位移 >1 cm 会记为警告。

### 当前阻断

**外部阻断（硬件/网络）**：`10.82.117.163` 自换电池断电后一直不可达——连续 3 轮
（2026-10-04 目标轮次 1/2/3）ICMP 100% 丢包、SSH 在协议 banner 阶段即失败。
换电池前的最后一次板端读数：电池 11.01–11.13 V、`motor_disabled=False`、
遥测帧率 25–32 Hz。板端一旦恢复网络即可按下方“恢复执行步骤”继续。

**技术阻断（驱动）**：C10B 已确认收到并整形了线速度指令，但车体不产生直线位移；
角速度通道正常。在补上“架空车轮轮向检查”和电量恢复之前，阶段 1 的八种动作与阶段 3
的参数迭代都无法产出有效样本——按计划要求，这类动作应标为无效，不得计入通过。

## 调参台账

| # | 日期 | 参数 | 旧值 | 新值 | 依据 | 复测结果 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 2026-10-04 | `pose_fusion.SLAM_ANCHOR_MAX_INNOVATION_M`（**新增代码常量**，仅用于 slam_toolbox 锚点路径） | 沿用 `D500_MAX_INNOVATION_M = 0.30 m` | **0.10 m** | 实测后退动作中 0.297 m 位置新息刚好通过 0.30 m 门限，单样本注入 0.106 m 跳变（`s1b_drive_back_20cm` t=14.81 s） | **跳变消除**：同动作新息 0.297 → **0.0105 m**，单步位移 0.1056 → **0.0047 m**，>2 cm 步数 0。但动作仍超时（车载 T265 证实车体真实偏转 48.9°，而指令角积分 8.69 rad），**故跳变只是症状，发散根因在下游**。 |

保留 `D500_MAX_INNOVATION_M = 0.30`，legacy `update_d500_absolute`/`update_field_wall` 路径行为**未变**。
回归：`test_pose_fusion.py` 新增 3 项（门限内接受、实测 0.297 m 离群被拒且位姿不跳、边界含等号），
全套 **781 tests OK**。文件已部署板端，md5 `01a73873c9503a7e4d65b8f247eb3402`。

### 门限修改后的关键结论

跳变修掉后，后退动作**仍然**失败，且车载 T265 独立证实车体确实偏转了 48.9°，说明：

- 定位侧：**已修复**（新息与跳变都回到正常量级）。
- 驱动侧：后退过程中车体真实偏转 −/+ 数十度，而指令角速度几乎未被有效执行
  （指令角积分 8.69 rad ≈ 498°，实测仅 49°，约 **10%** 角速度效率），
  对照纯 `rotate +90°` 在同一次会话中 4.9 s 内达到 90.056°（效率 ≈100%）。
- 即：**“纯旋转”角速度通道正常，但“直线运动（尤其后退）期间/之后的旋转”几乎失效。**

这已不是增益或门限问题，按计划属于“数据与地面观察矛盾 → 暂停参数迭代，核对轮向、有效轮径、
打滑、外参和融合位姿”。轮向已核对正确、融合已核对一致，剩余可疑项为**打滑/脚轮拖滞/地面**，
需要现场目视确认。

> 按计划“只改一个参数、增益单轮变化 ≤10%”的规则，本表在驱动直线通道恢复后才会写入第一条。

## 恢复执行步骤（板端重新在线后）

按顺序执行；每条实车动作单独提交、单独观察，不自动串联。

```sh
# 0. 上线确认与占用清点
ssh radxa@10.82.117.163 'uptime; ls -l /dev/ttyACM0 /dev/ttyS6; \
  pgrep -af "slam_toolbox|robocup|closed_loop"'

# 1. 环境（每次新终端）
export MAMBA_ROOT_PREFIX=/home/radxa/robocup_ros/root
export PYTHONPATH=/home/radxa/robocup_ros/python_ext:/home/radxa/car/code
cd /home/radxa/car
RUN=/home/radxa/car_test_logs/closed_loop_motion_20261004_04

# 2. 架空主动轮：轮向检查（不需要 T265 / SLAM；操作者观察左右轮方向）
python3 tools/basic_motion.py --config configs/robocup_diffdrive.toml \
  --confirm-motor-test jog --direction forward --seconds 1
#   再依次 backward / left / right

# 3. 落地后启动 SLAM sidecar（独立进程，保持运行）
/home/radxa/robocup_ros/bin/micromamba run -p /home/radxa/robocup_ros/env \
  ros2 launch /home/radxa/car/launch/robocup_slam.launch.py

# 4. 静止融合预检（无电机，需连续 10 s 稳定）
/home/radxa/robocup_ros/bin/micromamba run -p /home/radxa/robocup_ros/env \
  python tools/test_live_relative_pose.py --config configs/robocup_diffdrive.toml \
  --output $RUN/preflight_static --seconds 30

# 5. 阶段 1 单动作（一次进程只一条动作；三项确认必须现场成立）
/home/radxa/robocup_ros/bin/micromamba run -p /home/radxa/robocup_ros/env \
  python tools/closed_loop_motion.py --config configs/robocup_diffdrive.toml \
  --output $RUN/s1_drive_fwd_20cm \
  --confirm-motor-test --confirm-area-clear --confirm-estop-ready \
  drive-distance --cm 20

# 6. 阶段 2 离线分析（含有效性判定；超时/中断的运行同样可分析）
python3 tools/closed_loop_analyze.py $RUN/s1_drive_fwd_20cm
```

判定纪律：`validity.action_valid=false` 的动作直接作废重做，不得计入通过、不得取平均。
直线通道恢复前不进入阶段 3，不改任何增益。

## 2026-10-04 第二轮现场会话（电池恢复后）

### 环境与阶段 0 收尾

| 项目 | 结果 |
| --- | --- |
| 板端地址 | 网络重连后 IP 变为 `192.168.31.224`（原 `10.82.117.163`） |
| 电池 | **12.48–12.58 V**（换电池前 11.02 V），遥测 39 Hz（原 25 Hz） |
| 架空轮向检查 | **四条全部正确**：前进/后退两轮同向，左转左轮后+右轮前，右转反之 |
| 直线通道 | 开环 50 cm @ 0.10 m/s → 实测 **0.479 m，k_v = 0.958**（换电池前 ≈ 0.01） |
| 日志根目录 | `/home/radxa/car_test_logs/closed_loop_motion_20261004_04/` |

**结论：换电池前的“直线不走”确认为 11.0 V 欠压**，不是接线或固件问题。

### 阶段 1 已取得的有效样本

| 动作 | 目标 | 停稳实测 | 误差 | 判定 |
| --- | --- | --- | --- | --- |
| `drive_distance` | +20 cm | +17.21 cm | **−2.79 cm** | 成功，超 1 cm 目标 |
| `rotate` | +30° | +30.412° | **+0.412°** | 成功，达标 |
| `rotate` | −30° | −28.565° | **+1.435°** | 成功，略超 1° |
| `rotate` | +90° | **+90.056°** | **+0.056°** | 成功，达标，无跳变 |
| 开环直线 | +50 cm | +47.90 cm | −2.10 cm | k_v=0.958 |
| 开环直线 | −50 cm | −47.28 cm | +2.72 cm | k_v=0.954 |

`drive_distance +20 cm` 停在离目标 2.79 cm 处，**原因是 `position_tolerance_m = 0.03`**：
控制器判定 `remaining ≤ 3 cm` 即成功。这正是计划表格中“状态已 SUCCEEDED 但停稳误差
大于目标”一行，对应的候选动作是收紧 `position_tolerance_m`（阶段 3，尚未执行）。

### 阻断：后退（`drive_distance -20 cm`）可复现失败 —— 3/3

| 次数 | 结果 | 融合纵向 | 融合横向 | 融合偏航 | 指令角积分 |
| --- | --- | --- | --- | --- | --- |
| 1 | 超时 30 s | −0.2044 m | +0.0715 m | +3.7°（过程中摆到 −42.7°） | +8.09 rad |
| 2 | 超时 30 s | −0.2140 m | +0.1013 m | **+58.0°** | +8.70 rad |
| 3 | 超时 30 s | −0.2062 m | −0.0786 m | **−58.1°** | −8.67 rad |
| 4（重启 sidecar 后） | 被入口守卫中止 | −0.1472 m | −0.1032 m | +36.4° | −0.95 rad |

两次发散到**相反**的偏航，说明这是**发散型不稳定**（放大初始扰动），不是固定偏置。

**机理（t=14.81 s 的单样本证据）**：

| 量 | 值 |
| --- | --- |
| 融合单步位移 | **0.1056 m**（触发入口 `MAX_POSE_STEP_M = 0.10` 守卫） |
| 融合偏航 / T265 偏航 | +36.43° / +36.36°（**两者一致**） |
| SLAM 位置新息 | **0.2969 m**（恰好压在 `D500_MAX_INNOVATION_M = 0.30` 门限之下） |
| SLAM 偏航新息 | +0.200° |

即：**偏航是真实的**（T265 与融合一致），但 SLAM 在**位置**上与 T265 航迹推算差 29.7 cm，
该新息刚好通过门限被接受，按 `ANCHOR_BLEND=0.35` 在单个 50 ms 采样内注入约 10.4 cm 位移跳变。
控制器随后追逐这个虚假位移，进入发散循环。

对照：纯直线前进无跳变；纯旋转 ±30°、+90° 均无跳变且精度 0.06–0.41°。
**跳变只出现在“后退 + 转向修正”组合中。**

### 操作发现：SLAM sidecar 会卡死

连续几次发散动作后，sidecar 停在 `SLAM_STARTING`：`scan_publish_count=0`、
`scan_drop_count=290`（291 帧只放行 1 帧）、`slam.pose_received_count=0`。
`closed_loop_motion.py` 的 10 s 静止预检**正确地拒绝启动**（
`stationary T265+SLAM fused pose did not stay healthy`）。
按 PID 重启 sidecar 后预检立即恢复正常（漂移 ≤0.03 cm，扫描 4.0–4.5 Hz）。
**结论：每次 heavy 动作序列后应重启 sidecar，预检门限必须保留。**

### 当前阻断

**外部阻断（硬件/网络）**：无（板端已在 `192.168.31.224` 恢复）。

**技术阻断（后退方向）**：`drive_distance` 负方向在 SLAM 位置新息 ≈0.30 m 时被注入
10 cm 级位移跳变并进入发散循环，3/3 可复现。阶段 4 要求后退 50 cm 三次均 ≤1 cm，
在该问题解决前无法达成；正方向和旋转方向已达标或接近达标。
**未改动任何参数**——按计划要求，定位/驱动异常先查因，不夹带增益调参。

### 待定决策（需操作者/用户确认）

1. **收紧 SLAM 锚点位置新息门限**（`pose_fusion.D500_MAX_INNOVATION_M`，当前 0.30 m）
   使 29.7 cm 这类新息被拒绝而非接受。这是**代码常量**改动，不是 TOML 参数，
   须单独一轮 + 回归测试 + 记录旧值/新值；风险是全局依赖 SLAM 修正减少。
2. 先做地面观察：后退时车体是否明显自旋、是否有脚轮拖滞/打滑、地面是否平整。
3. 或先按计划收紧 `position_tolerance_m`，把已通过的正方向与旋转方向推到 1 cm／1°，
   后退方向单独作为遗留限制记录。

| 2 | 2026-10-04 | `navigation.position_tolerance_m`（TOML） | 0.03 | **0.01** | 前进 20 cm 停稳误差 2.79 cm，正好等于旧容差——控制器判定 `remaining ≤ 3 cm` 即成功（计划表格“状态已 SUCCEEDED 但停稳误差大于目标”一行） | **达标**：前进 20 cm 误差 **−0.47 cm**（横向 1.3 mm，偏航 0.74°）；前进 50 cm 误差 **−0.89 cm**（横向 0.3 mm，偏航 0.27°）；两者均无位姿跳变（单步 ≤8.3 mm，SLAM 新息 ≤0.010 m）。旧配置备份 `config_before_tol001.toml`。 |

### 阶段 4 进度（`position_tolerance_m = 0.01`）

| 动作 | 第 1 次 | 第 2 次 | 第 3 次 | 目标 ≤1 cm／1° |
| --- | --- | --- | --- | --- |
| 前进 50 cm | **−0.89 cm** | — | — | 第 1 次达标 |
| 前进 20 cm | −0.47 cm | — | — | 达标 |
| 左转 +90° | **+90.056°**（+0.056°） | **+89.900°**（−0.100°） | — | 2/3 达标 |
| 右转 −90° | **−89.768°**（+0.232°） | — | — | 1/3 达标 |
| 后退 50 cm | 未达成 | — | — | 阻断 |

### 后退问题：地面已清理仍失败，排除碎屑因素

操作者清理地面后复测 `s7_back20_cleanground`：仍超时，纵向 −0.2151 m、横向 +0.0661 m、
融合偏航 −12.3°，而指令角积分 **+8.0094 rad（459°）**。
**定位侧干净**（单步跳变 ≤5.4 mm、SLAM 新息 ≤13 mm，改动 #1 持续生效）。

紧接其后测得**纯旋转完全正常**：+90° → +89.900°（−0.100°）、−90° → −89.768°（+0.232°），
两轮轮速目标 ±0.037 m/s。

**关键矛盾**：后退动作的 `aligning` 阶段轮速指令与纯 `rotate` **完全相同**（v=0、ω=+0.3），
但前者 26 秒只转了 −0.2 rad，后者 5 秒转 90°。即**相同的轮速指令，结果相差约 60 倍**。
因此排除控制律、定位、增益、门限；问题在**后退运动之后的机械/传动状态**（打滑、脚轮拖滞、
或车体被卡），需要现场目视确认。

### 现场目视确认：左轮间歇性无力（硬件）

操作者在车旁观察后退动作后的原话：

> 「起初是顺时针旋转，**左轮几乎没有动**，转到较大幅度时开始逆时针顺时针小幅度交替摆动
> （似乎也是只有单个轮子在运动），最后逆时针大幅度转动至停止状态」

随后做两组各 6 条低速点动脉冲（3 组 `left` 原地转 + 前后各 1~2 条）交叉验证：

| 组 | 指令 | 操作者观察 |
| --- | --- | --- |
| A1–A6（2 s） | forward / left×3 / backward / forward | 未及细看，要求重做 |
| B1 | forward 3 s | 正常 |
| B2 | left 3 s | **左轮相对右轮明显无力** |
| B3 | left 3 s | **左轮相对右轮明显无力** |
| B4 | left 3 s | 正常 |
| B5 | backward 3 s | 正常（本次正后方直退） |
| B6 | forward 3 s | 正常 |

**结论**：左驱动通道（电机/减速箱/驱动板通道）**间歇性输出不足**，且集中出现在
**两轮反向的原地转工况**；直线前进、直线后退时两轮同向，未观察到异常。

这一条解释了之前所有"矛盾"数据：

- 纯 `rotate ±90°` 有时很好（0.06°/0.23°）——左轮当时正常。
- 后退 `drive_distance` 的 `aligning` 阶段需要长时间原地转，一旦左轮变弱，
  车体就绕右轮偏转、`cross_track` 单调增大、控制器持续加角速度，形成发散；
  发散方向取决于哪一侧当时更强，所以出现了 +58°/−58°/+57.7°/−12.3° 的随机方向。
- 定位侧始终干净（跳变 ≤5.4 mm、新息 ≤13 mm），与观测一致。

**处置**：按计划"左右明显不对称 → 暂停控制参数迭代，核对轮向、有效轮径、打滑、外参"
——轮向已确认正确、外参与融合已确认一致，剩余项为**电机/驱动硬件**。
阶段 4 的后退方向在该硬件问题排除前无法达成；**前进与旋转方向已达标**。

### 现场复测：对调左右电机线后左轮恢复正常

操作者把左右电机线对调后重做原地转点动（C1–C5、D1–D6、E1–E5 共三组）：
**左右两轮均正常，先前的左轮无力现象消失。**

解读：拔插动作本身修复了故障 → 原症状最可能是**左电机插头/接线接触不良**（间歇性），
而不是电机本体或 C10B 驱动板通道损坏。

**副作用（必须处理）**：对调电机输出线会**镜像转向极性** —— 直线方向不变，
但"左转"指令会让车顺时针转。任何闭环旋转/航向修正都会因此反向，必须先恢复正确极性
（把线对调回去，同时完成一次重新插拔）再继续闭环验收。

操作者另把车放到**圆盘**上（雷达停在圆心）作为旋转专用台架，旋转测试可用，
但**平移类测试（前后 50 cm）不能在圆盘上做**，需回到地面。

### 决定性证据：左轮"启动慢、先停"

电机线回调并重启开发板后复测，操作者在旁目视：

> 「前进时**左轮的启动较右轮慢**，后退时起初没有什么问题，但是**左轮较右轮先停**」

即左驱动通道存在**启动延迟 + 提前掉力**的不对称，与下述交叉实验合并：

| 接线状态 | 左轮表现 |
| --- | --- |
| 原始接线 | 间歇无力（后退发散方向随机 +58°/−58°/+57.7°/−12.3°） |
| **对调左右电机线**（=一次重新插拔） | **恢复正常**（C/D/E 三组共 16 条点动均正常） |
| **回调**（=又一次重新插拔） | **故障复现**（启动慢、先停） |

**结论：左电机回路的电气连接处于临界状态**（插头端子松动/氧化/变形，或线缆在插头根部
内部断股）。"重新插拔一次好、再插拔一次坏"正是接触不良而非绕组或驱动板损坏的特征。
修复方式：更换插头端子或直接焊接该路电机线，并加应力释放；不要靠反复插拔碰运气。

### 回调后的前后对比测试（`closed_loop_motion_20261004_05`）

| 动作 | 时间 | 结果 | 误差 | 判定 |
| --- | --- | --- | --- | --- |
| 前进 20 cm | 14:11:26 | 成功 | **−0.24 cm**（横向 4 mm） | ✅ |
| 前进 40 cm | 14:14:08 | 成功 | **−0.70 cm**（横向 2 mm，∫ω −0.204 rad 持续修偏） | ✅ |
| 后退 20 cm | 14:12:00 | 超时 | 横向 +5.6 cm、偏航 −28.2° | ❌ |
| 后退 40 cm | 14:14:45 | 3.3 s 中止 | `healthy fused pose lost during action` | ❌ |

**规律稳定：前进各距离全部达标，后退各距离全部失败。** 后退的失败形态随左轮当时状态
在"发散超时"与"定位失效中止"之间变化，与接触不良的间歇性一致。

### 融合位姿失效排查结论：T265 USB 链路

1. `back40` 中止时的融合事件：`state=d500_degraded`、`source_flags=["t265","dead_reckoning"]`、
   `d500_innovation_m=0.1616`、`d500_innovation_yaw_rad=0.0`、`rejection_reason=slam_innovation_gate`
   （258 样本中 253 个 `T265_SLAM` 正常，仅 5 个被拒；T265 置信度 258/258 = 0.667）。
2. 锚点来源确认为 TF `slam_map → t265_odom`（`code/components/slam_bridge.py:255`），本应是常量；
   它间歇性平移 16–30 cm 而偏航不变，指向 T265 侧位姿/时间戳不连续。
3. 手工转车窗口 `manualrot02` **完全无数据**，stdout 决定性报错：
   `T265UnavailableError: cannot start T265 pose stream: No device connected`。
4. `lsusb` + pyrealsense 枚举：**T265 完全不在 USB 上**（`T265_NOT_FOUND`，`count 0`）。

**统一解释**：T265 USB 链路不可靠，可同时解释此前两类看似不同的故障 ——
动作中途 `T265UnavailableError: Frame didn't arrive within 1000`（`s3_drive_back_20cm`），
以及间歇性 16–30 cm **纯平移、零偏航**的 SLAM 伪新息（T265 帧停顿/抖动时，
`map_T_t265_odom` 用了与扫描时刻不匹配的 T265 位姿合成）。

**结论：融合位姿失效的首要嫌疑是 T265 USB 连接（线缆/接口/供电），不是门限值。**
在该链路修好之前不要再改 `SLAM_ANCHOR_MAX_INNOVATION_M`。

## 待办

1. 电量恢复后复测 50 cm @ 0.10 m/s 开环，确认直线通道恢复。
2. 架空主动轮做轮向检查：前进/后退、原地左转/右转，记录左右轮实际方向。
3. 若轮向正确但落地仍不直行，按“融合位姿 → 控制命令 → 驱动限幅 → 车轮响应”继续查因。
4. 直线通道恢复后再进入阶段 1 的八动作覆盖，并对 `slowdown_distance_m` 评估
   短距离（20 cm）候选值。
5. ~~单独处理 `..._02` 出现的 0.21 m SLAM 锚点新息跳变（定位侧，不牵连驱动调参）。~~
   → **2026-10-05 已处理**，见下方附录。

## 附录：2026-10-05 定位链自愈改动台账

本轮不再动驱动/门限：`SLAM_ANCHOR_MAX_INNOVATION_M` 保持 `0.10`
（上面"在该链路修好之前不要再改"的约束仍然遵守），改的是**它之后没有恢复路径**这件事。
完整失效模式、数据流与实车证据见 `docs/D500_SLAM_DATA_FLOW.md` §4 / §7.6。
日志根目录 `/home/radxa/car_test_logs/slam_deadlock_fix_20261005/`。

| 项 | 旧值 | 新值 | 依据 | 复测结果 |
| --- | --- | --- | --- | --- |
| `PoseFusion.update_t265` 连续性恢复 | `_t265_pose is None` 时永久闩锁（每帧早退） | 无参照物时把当前帧当新原点 | 开机首帧 `tracker_confidence=0` 即永久 LOST | run03/run05 开机首帧确为 0，均在 **0.13–0.15 s** 内恢复 |
| 喂给 SLAM 的位姿 | `continuous_t265_pose`（受导航门控） | 新增 `slam_feed_t265_pose` | 循环依赖死锁 | run02/03/05：桥接 `SLAM_OK` 59/60、148/149、148/149 帧 |
| 桥接 TF 饿死看门狗 | 无 | 3 s 无 TF → 保持最后位姿补发一帧 + `slam.tf_starved_force_count` | 上游断流即永久空窗 | 实测 `tf_starved_force_count=0`（未触发，保留兜底） |
| 窗口上界容差 / 释放 pending scan | `<= last_tf_stamp_s`、无 `else` 分支 | `<= last_tf_stamp_s + max(tf_period_s, 实测 T265 周期) + tf_period_s`、释放并计数 | 251/291 帧误丢、桥接永久卡死；run06 在 T265 原生 19 Hz 下丢 155/292 | `scan_drop_no_tf_window_count` 291/294 → 0–1 / 40–150 s |
| SLAM anchor 持久新息 | `not loop_closure` 时永久拒绝 | 3 帧共识 + 0.5 s 渐变迁移；孤立离群仍拒绝 | run04：0.330979 m 新息被拒 74 s / 3507 样本 | run05 同一遮镜头实验：0.1456 m 新息 1 s 内迁完并回到 `ok` |
| `tools/test_live_relative_pose.py` | `--seconds` 不约束 preflight / shutdown | 整进程硬预算 + `BoundedPreflight` + `run_bounded` | 实测 `--seconds 22` 跑了 3 min 51 s | 本次实车 40/60/120/150 s 运行全部准时退出 |

**前进 20 cm 复测（`closed_loop_motion.py`，操作者监护 + 急停就位）**：
`valid=true`、动作 4.81 s、融合纵向 **+0.1960 m**（误差 4 mm）、横向 −0.8 mm、偏航 +0.354°、
动作中 `slam_innov max = 0.0003 m`、单样本最大步长 9.1 mm。
上一轮同一条动作 30 s 超时失败（车卡在 `initializing`），本轮通过。
（日志 `logs/after_run08_fwd20cm/`；run07 是同一动作的失败尝试，
原因是 sidecar 连续多轮后退化，按约定只杀节点 PID 重启后一次通过。）

**与"T265 USB 嫌疑"的关系**：本轮开跑前 `lsusb` 确认 T265 在 USB 上
（`Bus 002 Device 003: ID 8087:0b37`），run02/03/05 的 `tracker_confidence` 全程为 2，
手工转 92° 时 `d500_innovation_m` 全程 0.000000 m —— 链路目前是好的。
但本轮证明：**即使链路完全健康，一次 tracker 重定位仍可让融合位姿永久卡死**
（run04：0.331 m 持久锚点新息被永久拒绝），所以这两个根因相互独立，
不能再只归因于 USB 连接。

无硬件回归：本机与板端均 `825 tests OK (skipped=8)`（基线 781），`git diff --check` 干净。
