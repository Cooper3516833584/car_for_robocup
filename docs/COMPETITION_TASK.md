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
| CORNER_1 | 7.23 | 0.00 |
| CORNER_2 | 7.23 | 7.15 |
| FINISH | 0.00 | 7.15 |

`ROUTE_POINTS = (START, CORNER_1, CORNER_2, FINISH)`，
`ROUTE_HEADINGS_DEG = (0.0, 90.0, 180.0)`。
依次前进 7.23 m → 转到启动航向 +90° → 前进 7.15 m
→ 转到启动航向 +180° → 前进 7.23 m → 终点停车。
终点是 `(0, 7.15)`，完成第三段后才 `mission.finish()`，终点无额外转向。
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

监听启动时物资舵机 OPEN。正式模式的启动顺序为：

1. 地面站 RUN 后关闭小车物资，向无人机发送 CLOSE；必须收到匹配的 `COMPLETED` ACK。
2. 从 CLOSE 确认后开始完整计时 20 秒，这段计时不蜂鸣。
3. 计时结束后发送无人机重新建图指令，并重启、确认小车 SLAM 新实例；等待无人机重新建图 `COMPLETED` ACK。
4. 完整蜂鸣 5 秒，随后才进入运行程序：相机 +90°、原地 OCR、等待融合定位并捕获原点、分配颜色。
5. 任务板及出发准备完成时短响一声；任务配额首次成功写入串口后连响两声（重传不重复响）。
6. 配额 `COMPLETED` ACK 的源、session、seq 和命令号匹配后短响一声；单响结束后才发送无人机 START。
7. START 确认成功后，小车开始三边巡航。短响各为 0.15 秒，双响间隔为 0.15 秒。

`ACCEPTED` 只表示建图请求已受理，不代替建图完成；配额也不以 `RECEIVED`/`ACCEPTED` 作为保存成功。
任意等待或蜂鸣期间收到 STOP 都立即取消后续启动，关蜂鸣并沿用停车、物资 OPEN 和 STOP 转发。
ACK 超时、拒绝、建图失败或提示音输出异常均不进入下一启动步骤。
单机测试继续屏蔽所有无人机指令/ACK：本机物资 CLOSE 后计时 20 秒，再重建小车地图、蜂鸣 5 秒进入 OCR；不播放无人机任务已发送/已确认提示。
OCR 连续失败三次仍用 `(1,2,1)`。出发点已经在路线上，full 不额外调用 go_to_lane。

小车共四份物资：黄色 slot 0、单份颜色 slot 1、双份颜色 slot 2/3；无人机两份。
`pending_payloads` 按颜色保存对应槽位元组，双份目标在同一次往返中释放两个槽位。
三条边均保持相机 +90° 朝场地内部，只建立一个模型/相机工作线程。
`YoloVision.observe_colours()` 在同一把锁下提供同帧颜色名、框和时间戳；
颜色由模型 `result.names` 解析，不假定 class id。

巡航使用原 `find_centered_target()` 和 `track_global_line(A, B)`。
仅筛选尚未完成的黄色及分配色。可见目标使速度上限降为
`0.08 m/s × translation_speed_scale`，无待投目标可见时恢复原 PP 巡航速度。
一帧新鲜且有效的框水平中心进入既有中央 10% 时立即停车；多色同时满足时选离水平中心最近者，
距离相同按颜色名排序。框 Y 不参与投放距离控制。

判据为 `abs(cx - frame_width / 2) <= frame_width * 0.05`，使用实际帧宽：
640 宽为闭区间 [288, 352]，1280 宽为 [576, 704]。停车后不再采样确认、
估计像素增益或沿道路前后微调，直接调用既有 `perform_payload_detour()`：
左转 90°、融合定距接近
`DROP_SAFE_M=0.43 m`、停车复测/修正、释放（或模拟等待）、原轴直倒、
右转恢复道路朝向。
保持既有 0.05 m/s 接近上限、最后 6 cm 的 0.012 m/s、0.6 s 复测、
3 mm 融合距离容差、最多四次修正及原释放等待时间，未修改底层控制参数。

仅往返成功完成后移除该待投颜色，每个 slot 最多释放/模拟释放一次。
未进入中央窗口绝不投放，路段耗尽则继续路线；不再提供道路对齐失败后的盲投兜底。
已投颜色从待投集合移除，EntryLatch 保留三个新清空帧的防重复机制。
其他待投颜色可以先出现，也可到第三边才出现。
投放后始终再次提交该路段原始的 `A_fused → B_fused`，PP 从真实当前位置续走，
不回 A，不重建车道原点，不重复走横向车道。

一路未找到颜色也走完三边，到终点才记录未完成 slot 为 skipped。
模型加载失败沿用继续道路而不做视觉投放的策略；已启动视觉的相机故障、
推理异常、过期帧、定位丢失、STOP 和总超时沿用原有停车/清理路径。
正式模式与单机模式均默认真实 SG90；单机只禁止无人机命令。
物资 PWM 不可用或写入失败时停车并报告失败，监听层最终 OPEN 两只。
STOP、定位失败、异常和完赛同样全开；正常释放仅开对应 slot。
仅离线测试显式 simulate_release=True 才跳过物资硬件，仍完整执行往返。
接线、权限准备和脉宽约定见 [PAYLOAD_SG90.md](PAYLOAD_SG90.md)。

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
另含生产 7.23/7.15/7.23 m 长度仿真。测试不改变生产容差、轮距和加速度。
这不构成真车路程精度、雷达外参、车轮不越界或避障验收；当前路径不避障。
本次仅本地修改、测试和提交，不推送部署设备、不重启服务、不运行真车。

2026-10-15 本地软件验收（Python 3.13）：compileall、diff --check 通过；
比赛相关 100 项、Pure Pursuit 27 项、投放往返 36 项全部通过。
全量 1014 项：1007 通过、7 项因 Windows 符号链接创建权限限制跳过、0 失败。
日志位于本地忽略目录 `logs/three_leg_competition.txt`、
`logs/three_leg_pure_pursuit.txt`、`logs/three_leg_payload.txt`、
`logs/three_leg_full_suite.txt`。
