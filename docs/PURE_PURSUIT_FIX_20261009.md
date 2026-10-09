# Pure Pursuit 第二轮软件修复验收（2026-10-09）

依据 `ROBOCUP_Pure_Pursuit_20261009_Fix.md` 实施 Phase 1–4，开发基线为 GitHub/main `596f68785e6c761fc2c6be9f1b54ccbb8ea957e4`。本文记录本机离线验收；没有部署到 ROCK 5A，没有实车或继电器硬件测试。

## 实现与提交

1. `a7eadfc` — `fix(pp): keep virtual lookahead and reject real line overshoot`。虚拟 carrot 使用 `progress + lookahead_m`；真实投影独立判断终点。超过负停车容差返回双零 `BLOCKED/line_overshoot`，容差内保留 3 cm 横向验收。前进与倒车速度符号保持。
2. `d30bf4b` — `fix(pp): require measured turn settling before rejoining line`。第一次大于 60° 的入线偏角锁定 carrot 朝向，复用现有 TurnController。四个新鲜实测 T265 稳定样本后返回一次双零 `align_settled`，下一周期才平移。global/local line、navigate_to、navigate_to_pose 使用同一入口；runtime 继续负责 T265 有效性，未修改其生产代码。
3. 本报告所在提交 — `fix(payload): validate fixed yaw before payload release`。投放前用固定 `side_yaw` 和归一化角差逐新样本验收；超过 3° 拒绝释放，保留原异常、停车和继电器清理，并扩充原事件字段。本提交还补充合并回归发现的两个细节：保留 TurnController 的原始 BLOCKED 原因；同步缩短完整任务测试的搜索终点。

## 修改文件

生产代码仅修改：

- `code/components/differential_navigation.py`
- `code/components/basic_motion_controller.py`
- `code/payload_detour.py`

回归和离线工具：

- `code/test/test_pure_pursuit_line.py`
- `code/test/test_differential_navigation.py`
- `code/test/test_basic_motion_controller.py`
- `code/test/test_robocup_runtime.py`
- `code/test/test_payload_detour.py`
- `code/test/test_competition_task.py`
- `tools/simulate_pure_pursuit.py`

未改动 TurnController 稳定判定、硬件协议、驱动、融合、运行时全局门控、速度上下限、主循环或停靠容差；未添加 TOML 字段。

## 实际运行结果

| 检查 | 结果 |
| --- | --- |
| `py -3 -m compileall -q code tools` | 退出码 0 |
| 完整 unittest discovery | 891 项，884 通过，7 跳过；无失败或错误 |
| `test_pure_pursuit_line.py` | 21 项通过 |
| `test_differential_navigation.py` | 11 项通过 |
| `test_basic_motion_controller.py` | 12 项通过 |
| `test_basic_motion_startup.py` | 6 项通过（原规则未修改） |
| `test_payload_detour.py` | 29 项通过 |
| `tools/simulate_pure_pursuit.py` | 六组均 arrived |
| `git diff --check` | 退出码 0 |

原始全量与专项日志：[pure_pursuit_fix_20261009_tests.log](pure_pursuit_fix_20261009_tests.log)。其中异常日志来自故障注入测试，不代表测试失败。

仿真结果：[pure_pursuit_fix_20261009_simulation.json](pure_pursuit_fix_20261009_simulation.json)。保留原四组，新增两组：

- 47 cm 终点前剩余 7 mm、横偏 2 cm：一轮 50 ms 正向小幅修正，速度 0.045 m/s，没有入线旋转或倒车；最终沿线 0.465250 m、横偏约 0.019998 m，满足 5 mm/3 cm 验收。
- 大角度入线、50 ms 采样、连续六帧 0.5° 余转：未稳定前始终 v=0；达到四帧稳定后先双零，下一周期平移；共 79 个对齐周期，最终沿线约 2.797432 m。

首次全量检查发现完整任务测试搜索终点未随路线缩短，车实际到达约 0.658 m，超过其 0.45 m 第二拐角，正确触发 `line_overshoot`。只把测试 fixture 的搜索终点同步为 0.40 m，保留成功与 SAFE_STOP 断言及生产停车规则。

投放回归覆盖 ±5 mm 沿线、2 cm 横偏、固定朝向静止释放；+10°/-3.1° 拒绝、+2.9° 允许；跨 ±π 的 1° 差允许；稳定累计前从 2° 漂到 4° 当即拒绝；固定 B2→A2 倒车路线保留。

## 本机配置与未验收项

只读核对 `configs/robocup_diffdrive.example.toml` 和被 Git 忽略的本机 `configs/robocup_diffdrive.toml`，两者 `navigation.lookahead_m=0.20`。没有改写或提交实际 TOML；此结果不能证明 ROCK 5A 正在使用同样配置。

离线软件修复通过。SLAM anchor 的全局就绪门控保留；C10B 左右轮迟起、制动与超程、USB 断连、实际定位精度及电磁铁几何偏移仍需独立实测。理想仿真不代表这些硬件问题已经解决。
