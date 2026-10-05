# D500 雷达数据流与融合定位通路

本文整理 `D500 雷达 → slam_toolbox → 融合位姿` 的完整数据通路、各环节的时间基准要求，
以及现场排查中确认的真实故障点。用于后续判因，避免再把"传感器坏"与"链路对齐坏"混为一谈。

## 1. 数据通路

```
/dev/ttyS6 (230400 8N1, 只读)
  │  47 字节 54 2C 帧 + CRC；完整一圈组装成 720 束 / 0.5° 分箱
  ▼
D500 驱动 (components/radar_driver.py)
  │  · 每包带 16 位毫秒计数器
  │  · D500_TIMESTAMP_MODULUS_MS = 30000  ← 实测值，不是 uint16 的 65536
  ▼
DeviceClockMapper (components/sensor_clock.py)
  │  · map_milliseconds(device_ms, received_monotonic_s) -> measurement_s
  │  · _offset_s 只在首次/复位时抓取一次，之后保持不变（避免抖动进入时间戳）
  │  · 回绕判据：delta < -modulus/2 → 加 modulus；delta < 0 或 > modulus/2 → 复位
  ▼
T265 (USB, pyrealsense2)                        ← 上游位姿流的唯一来源
  │  · T265PoseAdapter：外参/轴系/置信度/新鲜度
  ▼
PoseFusion.update_t265()
  │  · _t265_rebase 吸收 tracker 跳变，产出连续里程计
  │  · continuous_t265_pose  ← 导航门控：连续性中断时返回 None
  │  · slam_feed_t265_pose   ← SLAM 专用，不受连续性门控（见 §4）
  ▼
slam_bridge (components/slam_bridge.py)
  │  · 组装 pending_scan
  │  · 发布 TF  t265_odom → base_link  ← 来自 T265，更新 first/last_tf_stamp_s
  │  · 发布静态 TF base_link → d500_link
  │  · TF 饿死看门狗：连续 3 s 无 TF → 用最后已知位姿补发一帧（见 §4）
  ▼  放行四条件（SlamBridge._release_scan）
  │    ① measurement_s - last_scan_pub_s >= scan_period_s        （限速）
  │    ② first_tf_stamp_s <= measurement_s <= last_tf_stamp_s + tf_period_s
  │                                                              （★ T265 TF 时间窗，含 1 个 TF 周期容差）
  │    ③ time.monotonic() - measurement_s <= 0.25 s              （不能太旧）
  │    ④ 无论 ② 是否成立都必须释放 pending_scan                  （否则桥接永久卡死）
  ▼
/d500/scan (LaserScan, frame=d500_link)
  ▼
slam_toolbox (Ceres + Huber + SCHUR_JACOBI)
  │  · scan matching + 位姿图优化
  ▼
TF slam_map → t265_odom        ← 物理上应为【常量】
  ▼
PoseFusion.update_slam_anchor() (components/pose_fusion.py)
  │  · 前置条件包含连续 T265 可用（continuous_t265_pose 非 None）
  │  · 门限 SLAM_ANCHOR_MAX_INNOVATION_M = 0.10 m
  │  · 融合 ANCHOR_BLEND = 0.35
  │  · 维护 map_T_t265_odom
  ▼
融合位姿 = map_T_t265_odom ∘ T265位姿   (source_flags = t265+slam+fused, state = ok)
  ▼
BasicMotionController → DifferentialDrive → C10B (/dev/ttyACM0)
```

## 2. 关键不变量

| 不变量 | 含义 | 破坏后的表现 |
| --- | --- | --- |
| `map_T_t265_odom` 应为常量 | SLAM 地图系与 T265 里程计系是刚体关系 | 缓慢变化=正常纠偏；**突变=配准离群或 T265 时间不连续** |
| D500 与 T265 必须同一单调时间基准 | 条件 ② 直接比较两者时间戳 | 不匹配 → 扫描全被覆盖丢弃 → `scan_publish_count=0` |
| `scan_publish_count` 必须 > 0 | slam_toolbox 的唯一输入 | 为 0 → SLAM 永远 `SLAM_STARTING`/`SLAM_FAILED` |
| 喂给 SLAM 的位姿流不得被导航门控掐断 | 只有 SLAM anchor 能重建 `map_T_t265_odom` | 掐断 → 循环依赖死锁（§4） |
| `slam.first_tf_stamp_s` 一旦非 null 就不应再回 null | 窗口下界代表"曾经有过 TF" | 全程 null = 上游断流复现 |

## 3. 已确认的真实故障点

| # | 现象 | 根因 | 证据 |
| --- | --- | --- | --- |
| 1 | 直线完全不动 | **电池 11.0 V 欠压**（正好压在 11.0 V 报警阈值） | 换电池后 k_v = 0.958；此前 ≈ 0.01 |
| 2 | 融合伪新息 16–30 cm（纯平移、零偏航） | **T265 USB 未插好/链路不良** | 插好后手工转 90°，`d500_innovation_m` 全程 **0.0000** |
| 3 | 融合位姿全程 `lost`，扫描 100% 丢弃 | **`slam_bridge` 条件 ② 未满足**（D500 与 T265 时间窗不匹配） | `scan_input_count=294` / `scan_publish_count=0` / `scan_drop_count=293` / `bridge_queue_overwrite_count=293` / `tf_lookup_fail_count=2589` |
| 4 | 融合位姿**间歇性**全程 `lost`，重启进程才能恢复 | **连续性门控与 SLAM 之间的循环依赖死锁**（§4） | `first_tf_stamp_s`/`last_tf_stamp_s` 全程 null → 窗口恒为 `[inf, -inf]` → `scan_drop_no_tf_window_count=291/294`、`scan_publish_count=0`；对照正常运行 `first_tf_stamp_s=3046.9`、`last_tf_stamp_s=3075.6`、`scan_publish_count=19` |
| 5 | 融合位姿**在一次运行内**从 `ok` 掉到 `d500_degraded` 后再不恢复，只有重启进程才行 | **SLAM anchor 一次持久的新息被永久拒绝**（§4.5） | 2026-10-05 实车 run04：t=46.0 s 车未动，`slam_map→t265_odom` 一步跳 **0.330979 m**（`d500_innovation_m` 从 0.0000 变 0.330979、`rejection_reason=slam_innovation_gate`），此后 74 s 内 3507 个样本全部被拒，状态永久 `d500_degraded` |

故障 3 排查中已排除的假设：

- ❌ "D500 模数被错当 uint16" —— `radar_driver.py:52` 已正确设为 30000，`sensor_clock.py`
  的 docstring 也专门警告过这个陷阱。
- ❌ "T265 坏" —— 560 帧 / 29.4 s = 19.05 Hz，`tracker_confidence` 全为 2，
  接收−测量时差 ≤0.9 ms，新鲜度 0.0007 s。
- ❌ "D500 坏" —— 5 秒 49 帧完整扫描，每帧 478–490 点，范围 49–654 cm。

## 4. 故障 4：连续性门控 / SLAM 循环依赖死锁

### 4.1 依赖链

```
T265 出现一次连续性中断（首个采样置信度不足 / 丢帧 ≥200 ms）
  → PoseFusion._t265_continuity_broken = True
  → continuous_t265_pose 返回 None
  → robocup_runtime 不再调用 slam_bridge.push_t265
  → 桥接不发 T265 TF → 扫描时间窗恒为空 [inf, -inf] → 100% 扫描被丢
  → slam_toolbox 永远拿不到扫描 → 永远没有 slam_map
  → 融合位姿永远 LOST
  → _t265_continuity_broken 永远无法清除（死锁）
```

### 4.2 为什么"永远无法清除"

`update_t265()` 的恢复分支原本写作
`if self.backend == "slam_toolbox" and self._t265_pose is not None:`，
`elif` 分支之后是裸 `else: return`。

因此当**第一个**有效采样之前就发生过一次中断（开机时 tracker 尚未收敛、
`tracker_confidence` 为 0，或更早的 `T265UnavailableError: Frame didn't arrive within 1000`），
`_t265_pose` 一直是 `None`：

1. 中断把 `_t265_continuity_broken` 置真，而 `_t265_pose is None`；
2. 之后每一次有效采样都走 `else: return`，**既不清标志也不写 `_t265_pose`**；
3. 于是门控永久闩锁，融合位姿永久 LOST，只有重启进程才能恢复。

这解释了为什么是"间歇性"：只要开机后第一帧就有效（运行 win02），一切正常；
只要开机先来一帧低置信度/超时（运行 verify01），就永久闩锁。

### 4.3 修复（三道，互相独立）

| 修改 | 位置 | 作用 |
| --- | --- | --- |
| 1a 启动闩锁修复 | `PoseFusion.update_t265` 的 `_t265_continuity_broken` 分支 | `_t265_pose is None` 时把当前采样当作新的里程计原点（此时"不连续"没有参照物），清标志、写 `_t265_pose` |
| 1b SLAM 专用喂给 | 新增 `PoseFusion.slam_feed_t265_pose`；`robocup_runtime._consume_t265` 改用它 | 导航侧仍然门控（`continuous_t265_pose` 语义不变），但 SLAM 侧永远拿得到连续位姿，可以 bootstrap |
| 2 TF 饿死看门狗 | `SlamBridge._force_tf_if_starved` | 连续 3 s 无 TF 发布时，用桥接内最后已知 T265 位姿**按当前时间**补发一帧 TF，并累加 `slam.tf_starved_force_count` |
| 3 窗口容差 + 释放分支 | `SlamBridge._release_scan` | 窗口上界加 1 个 TF 周期容差；无 TF 窗口时**也必须释放 pending_scan** 并单独计数 |

安全性说明（**未放宽任何安全门**）：

- `estimate()` 的 state 判定、readiness、限速、C10B 协议、看门狗均未改动。
- 导航仍然只接受 `state=ok` 的融合位姿；`slam_feed_t265_pose` 只喂给 slam_bridge。
- 看门狗只在**上游断流**时保持数据流动，且把"最后已知位姿按当前时间保持"这件事
  记进 `slam.tf_starved_force_count` + WARNING 日志，不静默。
- 1a 只在"从未建立过连续里程计"时生效；一旦有参照物，恢复仍然走原有的重基准路径。

### 4.4 修复后的自愈时序

```
T265 中断 → _t265_continuity_broken=True（导航继续拒绝）
  → 下一个有效采样：重基准 → 标志清除、_t265_pose 更新（1a）
  → slam_feed_t265_pose 非 None → push_t265（1b）
  → T265 TF 发布 → 窗口打开 → 扫描放行（3）
  → slam_toolbox 出 map→odom TF → 桥接出 anchor
  → update_slam_anchor 接受 → map_T_t265_odom 建立、_slam_received_s 更新
  → estimate(): t265_fresh ∧ slam anchor 新鲜 → state=ok, flags=(t265, slam, fused)
```

预期 ≤10 s（T265 19 Hz + 雷达 5 Hz + slam_toolbox 建图）。实车实测见 §7.6。

### 4.5 故障 5：SLAM anchor 持久新息被永久拒绝（第二个闩锁）

2026-10-05 实车复现（run04，人为遮挡 T265 10 s 触发 tracker 重定位）：

```
t=45.978 s  d500_innovation_m: 0.000000 -> 0.330979（车没动，融合位姿平滑）
            rejection_reason = slam_innovation_gate, d500_accepted = False
t=46.269 s  d500_age > 0.5 s（_slam_received_s 不再刷新）-> state = d500_degraded
t=46.3..120 s  3507 个样本全部同一个 0.330979，全部被拒；状态永不回到 ok
```

原因：`SLAM_ANCHOR_MAX_INNOVATION_M = 0.10`（上一轮为修 0.297 m 单帧离群而收紧）
之后的**地图迁移路径被 `not loop_closure` 门住**：

```python
if not loop_closure or delta_m > SLAM_LOOP_MAX_M or ...:
    self._slam_candidates.clear()
    self._last_d500_rejection = "slam_innovation_gate"
    return
```

而板端 slam_toolbox 根本不发 `LoopClosureEvent`（桥接启动即打印
`slam_toolbox LoopClosureEvent unavailable; large anchors remain gated`），
所以 `loop_closure` 恒为 False，**任何**超过 0.10 m 的持久 disagreement 都
永远无法进入已有的"3 帧共识 + 0.5 s 渐变迁移"路径 —— 与故障 4 同类：
一次运行内锁死，只能重启。

修复：把 `loop_closure` 从**前置条件**降级为**仅影响日志措辞**。仍然保留全部护栏：

| 护栏 | 值 | 作用 |
| --- | --- | --- |
| 单帧新息上限 | `SLAM_ANCHOR_MAX_INNOVATION_M = 0.10` | 单帧超过即拒绝（上一轮 0.297 m 回归不变） |
| 迁移总量上限 | `SLAM_LOOP_MAX_M = 1.0 m` / `SLAM_LOOP_MAX_YAW_RAD = 35°` | 超过就仍是离群值，直接丢 |
| 共识帧数 | `SLAM_ANCHOR_CONSENSUS_SAMPLES = 3` | 必须连续 3 帧互相一致（`SLAM_LOOP_CONSISTENCY_*`）才算真迁移 |
| 渐变 | `SLAM_LOOP_MIGRATION_S = 0.5 s` | 迁移是斜坡，不是单帧跳变 |
| 孤立离群 | 下一帧回到门内时 `_slam_candidates.clear()` | 单次坏变换仍被丢弃，不启动迁移 |

实车验证（run05，同样遮挡 10 s）：`t=48.09–48.17` 出现 5 帧
`tracker_confidence_too_low`（连续性中断，`state=lost` 5 帧）→ `t=48.19` 回到 `ok`；
随后 anchor 跳 0.1456 m → 被拒 42 帧 → 共识达成 → `t=50.27` `d500_accepted=True`
→ `t=50.04` 起 `ok` 并保持到 150 s 结束。**全程无进程重启，
最长连续 `ok` = 100.3 s（修复前 run04 只有 45.6 s 就永久卡住）。**

## 5. 可观测字段（`slam_status` 事件）

| 字段 | 判读 |
| --- | --- |
| `slam.first_tf_stamp_s` | 全程 null = 循环依赖复现（上游从未喂到桥接） |
| `slam.last_tf_stamp_s` | 冻结在过去 = TF 停发 |
| `slam.no_window_measurement_s` | 最近一次"新鲜但不在窗口内"的扫描时间 |
| `slam.scan_drop_no_tf_window_count` | >0 且持续增长 = 窗口不匹配；**不应**再因为"扫描比最新 TF 晚几十毫秒"而增长 |
| `slam.tf_starved_force_count` | >0 = 看门狗补发过 TF（上游曾断流 ≥3 s），必须当成告警看 |
| `slam.scan_publish_count` | 稳定增长；桥接限速上限 `scan_hz=5`（见 §7.6 注），实测 4.2–4.5/s。长期为 0 = upstream 断流 |
| `slam.tf_lookup_fail_count` | 持续增长 = slam_toolbox 未发布 `slam_map → t265_odom` |
| `slam.scan_age_ms` | 过大 = D500 设备时钟偏移错了，不是雷达坏 |

融合侧：`fused_pose` 事件里的 `state`、`source_flags`、`t265_continuity_broken`、
`d500_innovation_m`、`rejection_reason`。

## 6. 变更台账

| 日期 | 项 | 旧值 | 新值 | 依据 | 复测结果 |
| --- | --- | --- | --- | --- | --- |
| 2026-10-05 | `PoseFusion.update_t265` 连续性恢复分支 | `if backend=="slam_toolbox" and _t265_pose is not None: … else: return`（`_t265_pose is None` 时永久闩锁） | 新增 `if _t265_pose is None:` 启动引导分支 | 运行 verify01：`first_tf_stamp_s` 全程 null、`scan_publish_count=0`、`fusion_source=LOST`；错误日志 `T265UnavailableError: Frame didn't arrive within 1000` 之后再不恢复 | 无硬件：`test_first_break_before_any_pose_does_not_latch_the_gate_forever`、`test_slam_anchor_rebuilds_after_a_startup_break_and_reaches_ok`、`test_slam_bridge_is_fed_across_a_startup_continuity_break`；把该分支还原成旧行为后三例均失败（已实测） |
| 2026-10-05 | 喂给 SLAM 的位姿源 | `fusion.continuous_t265_pose`（受连续性门控） | 新增 `fusion.slam_feed_t265_pose`，`robocup_runtime._consume_t265` 改用它 | 循环依赖：门控 `None` → 桥接无 TF → 无扫描 → 无 SLAM → 门控永不清除 | `test_continuity_break_gates_navigation_but_not_the_slam_feed`、`test_slam_feed_advances_while_navigation_stays_gated`、`test_navigation_stays_gated_during_the_slam_backend_break` |
| 2026-10-05 | 桥接 TF 饿死看门狗 | 无（TF 停发即永久空窗） | 连续 3 s 无 TF → 用最后已知位姿按当前时间补发一帧；新增 `slam.tf_starved_force_count` | 上游任一次断流都足以让扫描窗永久为空；需要可观测的兜底而非静默等待 | `test_watchdog_holds_the_last_known_pose_after_three_seconds_without_a_tf`、`test_forced_tf_reopens_the_scan_window_after_starvation`、`test_watchdog_does_not_fire_while_tf_keeps_flowing` |
| 2026-10-05 | 窗口上界容差 | `first_tf_stamp_s <= measurement_s <= last_tf_stamp_s` | `… <= last_tf_stamp_s + self.tf_period_s` | 实测 251/291 帧在 T265 健康时仍被丢弃 | `test_a_batch_of_scans_after_the_newest_tf_is_never_counted_as_no_window`（10/10 放行，`scan_drop_no_tf_window_count=0`） |
| 2026-10-05 | 无窗口时释放 pending scan | 无 `else` 分支 → pending 永不释放 → 桥接永久卡死 | 释放并累加 `slam.scan_drop_no_tf_window_count` | 运行 verify01：`scan_drop_no_tf_window_count=291/294` | `test_scan_before_the_first_tf_is_counted_as_no_window_not_silently_held` |
| 2026-10-05 | `tools/test_live_relative_pose.py` 总时限 | `--seconds` 只约束主循环；preflight / shutdown 可无限阻塞（实测 `--seconds 22` 跑了 3 min 51 s） | `--seconds` 作为整进程硬预算；preflight 走 `BoundedPreflight` 守护线程；shutdown 走 `run_bounded`；新增 `--shutdown-timeout` | 现场实测挂死 | `code/test/test_live_relative_pose_tool.py` 8 项（含"preflight 永不返回"与"shutdown 阻塞 30 s"两个硬预算用例） |
| 2026-10-05 | 计数器取证工具 | 无（判读靠人工读 events.jsonl） | 新增 `tools/localization_chain_report.py`：单次运行体检 + `--compare BEFORE AFTER` 修改前后对比 | 验收项 1/2/5 需要可复算的判据 | `code/test/test_localization_chain_report.py` 9 项（含 verify01 闩锁签名、僵尸桥接与 before/after 用例） |
| 2026-10-05 | SLAM anchor 持久新息 | `if not loop_closure or delta_m > SLAM_LOOP_MAX_M or …` → 无 loop closure 时超过 0.10 m 的新息永久被拒 | 删除 `not loop_closure` 前置条件；新增 `SLAM_ANCHOR_CONSENSUS_SAMPLES = 3`；`loop_closure` 只影响日志措辞 | 实车 run04：0.330979 m 持久新息被拒 74 s / 3507 样本，状态永久 `d500_degraded`；板端 slam_toolbox 不发 `LoopClosureEvent` | 无硬件：`test_persistent_anchor_disagreement_migrates_instead_of_latching`（还原旧门后报 `D500_DEGRADED != OK`，已实测）、`test_isolated_large_anchor_outlier_does_not_start_a_migration`、`test_anchor_far_beyond_the_migration_bound_stays_gated`；实车 run05 同一遮挡实验自愈（见 §4.5） |
| 2026-10-05 | 桥接 ROS 适配器 `_RosSlamSink` | 全部 ROS 调用内联在 `_run_ros`，无法离线测试 | 抽出 `_RosSlamSink`（唯一持有 ROS 消息类型的地方）；扫描/TF 判定逻辑移入 ROS 无关的 `process_iteration` | 实车 run01 暴露 `AttributeError: '_RosSlamSink' object has no attribute '_LaserScan'`，纯 fake-sink 测试看不见 | `code/test/test_slam_bridge_ros_sink.py` 5 项（注入伪 ROS 模块，删掉 `self._LaserScan = LaserScan` 即失败，已实测） |
| 2026-10-05 | 窗口上界容差（按实测 T265 周期） | 固定 `+ tf_period_s`（50 Hz → 20 ms） | `+ max(tf_period_s, 实测 TF 发布间隔) + tf_period_s`（上限 `TF_INTERVAL_CAP_S = 0.25`） | run06：T265 按原生 19 Hz 被读取时最新 TF 时间戳天然落后新扫描 ~52 ms，固定 20 ms 容差丢掉 155/292 帧（53 %），进而把 slam_toolbox 饿成 >0.5 s 锚点空档、融合状态反复掉出 `ok` | `test_scans_survive_a_t265_read_at_its_native_nineteen_hertz`（还原固定 20 ms 后 `no_window=40`，已实测）；实车 run07/run09：`no_window` 增量 1 与 **0** |
| 2026-10-05 | 桥接 `state()` 启动语义 | 线程已 `start()` 但 `_ready` 尚未置真时误报 `SLAM_FAILED` | 新增 `_started`；已启动但未就绪报 `SLAM_STARTING`，`_failed` 仍优先报 `SLAM_FAILED` | run08 日志第一条 `slam_status` 就是 `SLAM_FAILED`，把健康启动误判成桥接死亡 | `test_a_started_but_not_ready_bridge_reports_starting_not_failed`；实车 run09 首帧为 `SLAM_STARTING`、`bridge_alive=True` |

无硬件回归（本机 `python` = 3.12.4 / 板端 `python3` = 3.11.2，两边一致）：
`compileall` 通过；`python -m unittest discover -s code/test -p "test_*.py"`
→ 本机 **Ran 825 tests, OK (skipped=8)**、板端 **Ran 825 tests, OK (skipped=8)**（基线 781）；
两边 `git diff --check` 干净。

实车复测已完成，见 §7.6。

## 7. 实车验收流程（需操作者监护、急停可及、空旷区域）

前置只读检查：

```sh
lsusb | grep 8087                                   # T265 在 USB 上；本次实测在 Bus 002 Device 003 (8087:0b37)
python3 tools/d500_diag.py --port /dev/ttyS6 --seconds 5   # 约 49 帧完整扫描
```

运行环境：两个 shell 变量，并且**运行时必须用 micromamba 环境里的 python**
（系统 `python3` 3.11.2 没有 `rclpy`，`SlamBridge._run_ros` 会立刻抛异常，
`slam_status.state` 恒为 `SLAM_FAILED`、`tf_lookup_fail_count=0` —— 运行 run01 即此坑）：

```sh
export MAMBA_ROOT_PREFIX=/home/radxa/robocup_ros/root
export PYTHONPATH=/home/radxa/robocup_ros/python_ext:/home/radxa/car/code
MAMBA=/home/radxa/robocup_ros/bin/micromamba
ENV=/home/radxa/robocup_ros/env
$MAMBA run -p $ENV python3 <tool> ...
```

### 7.1 有界预检（有界，30 s 必返回）

用 `tools/closed_loop_motion.py`；`tools/test_live_relative_pose.py` 的硬预算缺陷已修
（见 §6 台账），本次实车取证即用它（`HARDWARE_PROBE` + `sensor_only`：不开 C10B、不开继电器）。
确认 `fused_pose.state=ok` 且 `source_flags` 含 `t265`/`slam`/`fused` 连续 10 s。

### 7.2 复现实验（关键）

运行中人为制造一次 T265 连续性中断：短暂拔出再插回 T265 的 USB，
或用黑纸/手掌**完全贴紧遮住两个镜头 10 s**（实测 4 s 不够，`tracker_confidence` 不掉）。
**不要重启任何进程**，观察 `fused_pose` 是否在 ≤10 s 内回到 `state=ok`。

判据：
- `slam.first_tf_stamp_s` 从不回到 null；
- `slam.scan_publish_count` 持续增长；
- `slam.tf_starved_force_count` 可以 >0（说明看门狗工作了），但 `state` 必须自愈。

### 7.3 手工转车

不发任何电机指令，操作者用手把车原地慢转 90°（3–5 s），检查
`d500_innovation_m` 全程 ≤0.02 m（正常基线实测 0.0000）。

### 7.4 前进 20 cm

```sh
python tools/closed_loop_motion.py --config configs/robocup_diffdrive.toml \
  --output /home/radxa/car_test_logs/<run>/fwd20 \
  --confirm-motor-test --confirm-area-clear --confirm-estop-ready drive-distance --cm 20
```

### 7.5 取证

每次运行的日志目录固定在 `/home/radxa/car_test_logs/<run>/`（仓库外），至少保留：

- `events.jsonl`：`t265_pose`、`slam_status`、`fused_pose`、`drive_command`；
- `summary.json` / `manifest`：本次运行用到的 commit、TOML sha256、计数器起止值；
- "修改前/后"计数器对比：直接跑取证工具，它会打印 `scan_input_count`、
  `scan_publish_count`、`scan_drop_count`、`scan_drop_no_tf_window_count`、
  `tf_starved_force_count`、`first_tf_stamp_s`、`last_tf_stamp_s`、
  `no_window_measurement_s`、`tf_lookup_fail_count` 的起止值与判据结论：

```sh
# 单次运行体检（含验收项 1/2/5 的自动判据）
python3 tools/localization_chain_report.py /home/radxa/car_test_logs/<run>
# 修改前 vs 修改后
python3 tools/localization_chain_report.py --compare \
  /home/radxa/car_test_logs/<before_run> /home/radxa/car_test_logs/<after_run>
```

### 7.6 验收判据与实测结果

日志根目录：`/home/radxa/car_test_logs/slam_deadlock_fix_20261005/`（仓库外），
`manifest.json` 记录 commit / TOML sha256 / 各次运行计数器，
`before_after_counters.txt` 是 `--compare` 输出。

| # | 验收项 | 判据 | 实测 |
| --- | --- | --- | --- |
| 1 | TF 不再饿死 | 任何 ≥20 s 运行中 `slam.first_tf_stamp_s` 从不为 null | ✅ run02/03/05/09：仅第一帧 `slam_status`（首次发布之前）为 null，`null_after_first=0`；`last_tf_stamp_s` 全程推进；`tf_starved_force_count=0` |
| 2 | 扫描放行 | `scan_publish_count` 稳定增长，速率 ≥ 8/s | ⚠️ 稳定增长、从 0 变为 4.19–4.37/s。**≥8/s 与设计不符**：`SlamBridge.__init__` 的 `scan_hz=5.0` 把发布限速在 5/s，`slam_manual_accuracy.preflight()` 的健康带也是 3.5–6 Hz；`scan_input` 实测 10.0/s（D500 健康）。未改限速（超出本次修复范围，且会改变已验证的 slam_toolbox 输入率）——需要 10 Hz 输入的话应作为独立改动评估。`scan_drop_no_tf_window_count` 增量已降到 0–1（每 40–150 s 运行） |
| 3 | 自愈 | 人为 T265 连续性中断后，无需重启进程，≤10 s 回到 `state=ok` | ✅ run05：t=48.09–48.17 五帧 `tracker_confidence_too_low`（`state=lost`）→ t=48.19 回 `ok`（**0.022 s**）；随后的 0.1456 m 持久 anchor 新息 → t=49.76–50.02 `d500_degraded` → t=50.04 回 `ok`（**0.021 s**），此后 100 s 保持 `ok`。run04（修复前）同一实验永久卡在 `d500_degraded`。另外 run03/run05 开机的**第一帧** `tracker_confidence` 就是 0（正是会永久闩锁的场景），分别在 0.147 / 0.130 s 内恢复 |
| 4 | 定位精度不被破坏 | 手工转 90°：`d500_innovation_m` 全程 ≤0.02 m | ✅ run03：手工原地转 92.2°（t≈89–95 s），全程 7051 个样本 `d500_innovation_m = 0.000000 m` |
| 5 | 静止预检 | 连续 10 s `state=ok` + `source_flags` 含 `t265`/`slam`/`fused` | ✅ run02 最长连续 `ok` 59.5 s、run03 149.5 s、run05 100.3 s、run09 38.8 s，flags 恒为 `['t265','slam','fused']` |
| 6 | 无回归 | `compileall` 通过、≥781 tests OK、`git diff --check` 干净 | ✅ 本机与板端均 **825 tests, OK (skipped=8)**；两边 `git diff --check` 干净 |
| 7 | 安全门未放宽 | 未改 readiness、未提限速、未改 C10B 协议、未禁用看门狗 | ✅ 仅改 `PoseFusion` 的连续性与 anchor 语义、`SlamBridge` 的放行/看门狗/启动状态、运行时取用哪个 T265 访问器；`estimate()` 的 state 判定、readiness、驱动限速、C10B 编码、驱动看门狗均未触碰 |

端到端动作（额外验证，不在 7 条验收项内）：

- **前进 20 cm 成功**（run08，`closed_loop_motion.py drive-distance --cm 20`，操作者监护 + 急停就位）：
  `valid=true`、`state=succeeded`、动作时长 4.81 s；
  融合位移 **+0.1960 m**（目标 0.20 m，纵向误差 4 mm）、横向 −0.0008 m、偏航 +0.354°；
  动作中 `slam_innov max = 0.0003 m`；单样本最大步长 9.1 mm（0 次 >2 cm）；
  `t265 raw displacement = 0.1965 m`。
  上一轮同一条动作曾 30 s 超时失败（车卡在 `initializing`）。
- run07 是同一动作的**失败尝试**：融合位姿在一次 0.2556 m 锚点新息后 0.27 s 掉出 `ok`，
  工具按安全门主动中止 —— 该行为在修复前后一致（冻结锚点同样会在 0.5 s 后掉出 `ok`），
  根因是 sidecar 连续跑多轮后退化；**按文档约定只杀节点 PID、不杀父进程**重启
  `async_slam_toolbox_node` 后 run08 一次通过。

其他实测：

- `slam.tf_starved_force_count = 0`（所有运行）——窗口容差 + 上游修复后看门狗一次都不用触发，
  但保留为兜底并纳入监控。
- `slam.tf_lookup_fail_count` 从 2589（故障 3）降到 0–4。
- `slam.bridge_queue_overwrite_count` 从 3482（run01 桥接线程死亡时）降到 0–12。
- 桥接 `state`：`SLAM_STARTING` 1 帧后 `SLAM_OK` 稳定 39/40、59/60、148/149、118/119。
- run01（59 s）是**无效运行**，保留作取证：用系统 `python3`（无 `rclpy`）启动，
  `SlamBridge._run_ros` 立刻抛异常，`state` 恒 `SLAM_FAILED`、`scan_publish=0`、`tf_lookup_fail=0`。
  判据：`tf_lookup_fail_count == 0` 且 `scan_publish == 0` ⇒ 先怀疑解释器/环境，不是传感器。

## 8. 尚未验证的既有嫌疑（本次未改动）

1. **`sensor_clock._offset_s` 是否发生复位**：`sensor_clock.py` 在 `delta_ms < 0` 或
   `delta_ms > modulus/2` 时复位。复位会把 `measurement_s` 直接设为 `received`（当刻单调时间）。
   若 D500 计数器出现一次非单调跳变，就会触发复位并改变整个时间基准。
   现已用 `slam.scan_age_ms` 暴露此类丢帧。
2. **D500 计数器非单调跳变的物理来源**（供电/串口误码），需要串口原始时间戳长时间采集：
   `tools/d500_clock_measure.py`（写 CSV）+ `tools/d500_clock_fit.py`（拟合）。
