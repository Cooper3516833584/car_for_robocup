# 2026 小车三边比赛路线

正式入口为 `code/main_robocup.py --mode hardware-mission --competition`；
单机入口 `code/car_full_flow_test.py` 仍通过地面站 RUN 进入同一个
`competition_task.run_full_mission()`。唯一路线数值位于该模块顶部，
不使用独立路线配置、地图或 280/420/250 cm 测试路线生成器。

## 路线及坐标

比赛局部原点为启动时 D500 中心，初始车头为 +X（图中向下），
初始左侧为 +Y（图中向右），正航向逆时针，单位为 m。

| 点 | 局部 X | 局部 Y |
|---|---:|---:|
| START | 0.00 | 0.00 |
| CORNER_1 | 7.60 | 0.00 |
| CORNER_2 | 7.60 | 7.36 |
| FINISH | 0.00 | 7.36 |

`ROUTE_POINTS = (START, CORNER_1, CORNER_2, FINISH)`，
`ROUTE_HEADINGS_DEG = (0.0, 90.0, 180.0)`。
依次前进 7.60 m → 转到启动航向 +90° → 前进 7.36 m
→ 转到启动航向 +180° → 前进 7.60 m → 终点停车。
终点是 `(0, 7.36)`，完成第三段后才 `mission.finish()`，终点无额外转向。
两个路口均使用融合地图绝对 `rotate_to()`，不累计上一次实际转角误差。

OCR 完成、融合位姿有效、底盘尚未出发时捕获一次 `origin_base`。
该不可变位姿在投放、转向和定位修正后均不刷新。局部点不是 SLAM 零点，
运动 API 始终消费融合地图中的驱动轴中心 `base_link` 坐标。
现有 `runtime.config.d500_mount.x_m/y_m` 为雷达相对 base_link 的安装偏移。
局部雷达目标 q、相对航向 h 转为底盘目标的公式为：

`p_base = p0 + R(theta0)*d + R(theta0)*q - R(theta0+h)*d`。

只转换路线目标，不再次变换已经是 base_link 的融合测量，不重置 SLAM/T265。
偏移为零时自动退化为一次旋转和平移。原地转向以 base_link 为轴，
非零雷达偏移会绕小弧；不自动添加横向补偿。现场仍需确认外参和车轮转弯空间。

## 一次巡航、两份物资

启动次序保持：地面站 RUN → 既有倒计时/最后五秒蜂鸣与重建图
→ 相机 +90°、原地 OCR → 等待融合定位并捕获原点 → 分配颜色
→ 正式模式确认无人机配额和启动（单机模式跳过）→ 三边巡航。
OCR 连续失败三次仍用 `(1,2,1)`。出发点已经在路线上，full 不额外调用 go_to_lane。

两份物资为 `pending_payloads = {"yellow": 0, car_color: 1}`。
按原有分配规则选一种单数量颜色给小车，其余三份自由颜色给无人机。
三条边均保持相机 +90° 朝场地内部，只建立一个模型/相机工作线程。
`YoloVision.observe_colours()` 在同一把锁下提供同帧颜色名、框和时间戳；
颜色由模型 `result.names` 解析，不假定 class id。

巡航使用原 `find_centered_target()` 和 `track_global_line(A, B)`。
仅筛选尚未完成的黄色及分配色。可见目标使速度上限降为
`0.08 m/s × translation_speed_scale`，无待投目标可见时恢复原 PP 巡航速度。
框水平中心进入既有中央 10% 时停车；两色同时满足时选离水平中心最近者，
距离相同按颜色名排序。框 Y 不参与投放距离控制。

道路精对齐继续用三个新帧、既有像素增益探针和沿道路前后微调，
反馈固定为此次选中的颜色，另一颜色不能接管。随后调用既有
`perform_payload_detour()`：左转 90°、融合定距接近 `DROP_SAFE_M=0.43 m`、
停车复测/修正、释放（或模拟等待）、原轴直倒、右转恢复道路朝向。
保持既有 0.05 m/s 接近上限、最后 6 cm 的 0.012 m/s、0.6 s 复测、
3 mm 融合距离容差、最多四次修正及原释放等待时间，未修改底层控制参数。

仅往返成功完成后移除该待投颜色，每个 slot 最多释放/模拟释放一次。
不能稳定对齐则跳过本次机会；EntryLatch 要求三个新的清空帧才重新触发，
防止原框原地反复停靠。第二种颜色可以先出现，也可到第三边才出现。
投放后始终再次提交该路段原始的 `A_fused → B_fused`，PP 从真实当前位置续走，
不回 A，不重建车道原点，不重复走横向车道。

一路未找到颜色也走完三边，到终点才记录未完成 slot 为 skipped。
模型加载失败沿用继续道路而不做视觉投放的策略；已启动视觉的相机故障、
推理异常、过期帧、定位丢失、STOP 和总超时沿用原有停车/清理路径。
未配置物资 PWM 时，正式模式沿用跳过硬件往返的策略；单机模式即使
未配置 PWM 也执行完整接近、停车复测和返回，绝不输出物资舵机命令。

## 日志

- `competition_route_origin`：一次性 fused base 起点、D500 mount、推导雷达起点、局部四点。
- `competition_route_leg_start/done`：leg 1/2/3、局部端点、转换后的融合 base 端点、局部目标航向、实际融合位姿。
- `competition_route_turn_start/done`：路口编号、目标局部航向、实际底盘和雷达融合位置。
- `payload_drop`：颜色、slot、success、simulated、detour_completed；模拟不记实投成功。
- `competition_finished`：实际融合终点和 `finish_local_xy`，局部与融合字段明确区分。

## 独立 stage

独立 stage 使用调用者负责的融合 base 坐标，不从中途车姿推断比赛原点。
`competition_stage_frame` 明确记录该约定。旧坐标常量只作为四点的兼容别名，
不在单机入口复制。单独执行前必须确认这些数值确实属于当前融合地图。

`lane` 起点重合时仅确认定位；`go_to_task_board()` 点位重合时原地读板，
均不提交零位移导航或零长度直线。`cross-lane` 保留历史预备入口，
full 不调用它；`yellow-search` 保留单颜色搜索和显式去搜索起点的调试行为，
full 使用三边待投巡航，不调用该兼容搜索入口。
`corner2` 以原横向道路起止点继续固定线；`finish` 以第三条边至终点，只按位置结束。

四个 direct 阶段 `task-board/hc-send/yellow-detect/drop` 仍只操作各自设备，
不启动底盘定位 runtime。其他运动 stage 需要 `--mode hardware-mission`。
独立 `drop` 会实际操作物资舵机。没有新增 release-mode/relay-port/payload-slot 选项。

```sh
python3 code/main_robocup.py --competition-stage task-board --task-board-camera 0
python3 code/main_robocup.py --competition-stage hc-send --task-red-count 2 --task-blue-count 1 --task-green-count 1
python3 code/main_robocup.py --competition-stage yellow-detect
python3 code/main_robocup.py --mode hardware-mission --competition-stage yellow-search
```

## 软件验证与现场边界

```sh
python3 -m compileall -q code
python3 -m unittest discover -s code/test -p 'test_competition_*.py' -v
python3 -m unittest discover -s code/test -p 'test_pure_pursuit_line.py' -v
python3 -m unittest discover -s code/test -p 'test_payload_detour.py' -v
python3 -m unittest discover -s code/test -p 'test_*.py'
git diff --check
```

测试用假传感器和假底盘，通过真实 PP、drive.step 和融合位姿反馈执行同拓扑路线，
另含生产 7.60/7.36/7.60 m 长度仿真。测试不改变生产容差、轮距和加速度。
这不构成真车路程精度、雷达外参、车轮不越界或避障验收；当前路径不避障。
本次仅本地修改、测试和提交，不推送部署设备、不重启服务、不运行真车。

2026-10-15 本地软件验收（Python 3.13）：compileall、diff --check 通过；
比赛相关 100 项、Pure Pursuit 27 项、投放往返 36 项全部通过。
全量 1014 项：1007 通过、7 项因 Windows 符号链接创建权限限制跳过、0 失败。
日志位于本地忽略目录 `logs/three_leg_competition.txt`、
`logs/three_leg_pure_pursuit.txt`、`logs/three_leg_payload.txt`、
`logs/three_leg_full_suite.txt`。
