# 2026 小车比赛主程序与逐项调试

入口仍是 `code/main_robocup.py`。比赛流程在 `code/competition_task.py`，复用现有
融合定位、BasicMotionController、DifferentialDrive、OCR、HC15SerialDriver、LCUSRelay。
本次任务层整合未改动 D500/T265/融合算法、C10B 帧发送器或底盘 watchdog。

完整流程：等待融合定位 → 车道入口 → 任务板观察点 → OCR → HC 单发 → 第一转角
→ 横向车道 → 连续黄色搜索至画面横向中部 → 融合位置投放往返（含 0° 视觉精对准）
→ 第二转角 → 终点坐标停车。

正式车道使用现有 `track_global_line()`，按测定的线段方向运行和回线：

| 函数 | 固定路线段/动作 |
|---|---|
| `go_to_lane` | 起点进入 `LANE_ENTRY`，保留点到点导航及入口朝向 |
| `go_to_task_board` | `LANE_ENTRY → TASK_BOARD`，到位后转到任务板观察朝向 |
| `go_to_first_corner` | `TASK_BOARD → CORNER_1` |
| `enter_cross_lane` | 转到横向车道朝向，沿 `CORNER_1 → YELLOW_SEARCH_START` 行驶 |
| `go_to_second_corner` | 按 `YELLOW_SEARCH_START → CORNER_2` 固定车道回线、到达第二转角 |
| `go_to_finish` | `CORNER_2 → FINISH`，只按位置到达，不要求最终 yaw |

`TASK_BOARD_X/Y` 是车道上的读板观察位置，不是任务板实体中心。若现场观察点离开
车道，应在任务层明确安排观察和回车道路线后再运行，不改变底层导航算法。

## 2026-10-10 验证能力合入

保留上述比赛路线坐标、OCR 读板、任务数量和 HC 单发顺序；不调用验证程序的
280/420/250 cm 路线生成器。主程序与验证程序现在共用 `code/target_patrol.py`
的视觉、`code/payload_detour.py` 的投放动作、`code/config/mission_profile.py`
的速度/继电器策略，以及相机舵机的 PWM 导出重试。旧 `center_target_route.py`
仅保留导入兼容入口。

读板结束后，在黄色搜索阶段把相机舵机保持在标定的 +90°（2500 us）。沿主程序
`YELLOW_SEARCH_START → YELLOW_SEARCH_END` 连续推理，只接受 yellow 类且置信度
至少 0.8 的目标。目标可见时将速度限制到 `8 cm/s × translation_speed_scale`，
目标中心 X 进入画面横向 45%～55% 即停止搜索；Y 不作为触发条件。
未找到目标则跳过投放并继续原第二转角和终点。相机/推理故障与过期结果结束任务。

2026-10-11 修复后，中央 10% 仅触发停车。仍借用同一个 +90° YOLO worker，
取 3 个不同时间戳的新框中心中位数；需要时在车道内先前进 2 cm 测一次像素增益，
固定符号并沿原道路朝向正/反微调到水平中心 ±4 px。探测起点使用采样后的最新融合位置。
失去可靠框或增益不可辨识时跳过投放。居中后直接左转 90° 一次，记录实际转后 pivot。
按用户最新要求，保持相机 +90°，完全按融合定位盲投；固定 pivot → `DROP_SAFE_M` 的
侧向直线仅修正行进横偏。停车复测轴向距离，必要时沿同轴低速正/反微调，保留横向坐标。
不再使用 0° 视觉 Y 提前停车。
投放后从最新融合位置沿 side_yaw 平行直倒，回到 pivot 内侧截面后才右转 90°。
靠边段没有横移、原地旋转、误差方向自动翻转或历史最佳 XY 回跑。

`DROP_SAFE_M=0.43 m` 来自用户 2026-10-11 将 47 cm 修改为 43 cm 的要求。
主程序仍只接受中央区域触发并投放固定一路。+90° 中心使用 `SEARCH_REFERENCE_CX=320 px`，按用户要求已删除
居中后的 7 cm 机械补偿。侧向接近限速 0.05 m/s，返回使用原巡航上限。
最后 6 cm 限速 0.012 m/s；停车更新融合 0.6 s，距离容差 ±0.003 m，最多 4 次轴向微调。
始终不能收敛则停止动作，不释放。该容差只描述融合结果，不代表物理定位精度。
0° 参考照片仅用于独立停车预览。其它颜色、其它仓位同样融合定距。步骤和验收见
[CH3 车道边界修复](CH3_LANE_SAFE_20261011.md)。定位损坏、设备故障、STOP、
总超时及继电器清理保持原处理。返回车道后继续原固定路线段。

长直线沿用已验证的随计划速度增加前视距离；短于或等于 0.50 m 的直线仍使用
基础前视。`--speed-scale` 默认 1，显式使用 2 可选择此前验证的速度倍率，
同时缩放轮速上限和转角控制增益，不修改路线、加速度或活动 TOML。
自适应参数和计算式见 [巡线测试说明](CENTER_TARGET_ROUTE_TEST.md)。

主程序的整场默认仍按继电器配置运行。`--release-mode simulate` 强制禁用真实
继电器并使用四路内存继电器；`--release-mode relay` 要求已启用的确认端口，
模拟模式不能同时传 `--relay-port`。只有 full 接受 `--release-mode`。
整场没有继电器对象或吸持/释放回读失败会结束任务。

定位运动任务默认总时限 300 s，`--max-seconds` 可设为 1～600 s。
`<log-dir>/STOP`（默认 `logs/competition/STOP`）、SIGINT 和 SIGTERM 均能中断
搜索及嵌套投放动作，并执行底盘停车、继电器退出、视觉关闭与舵机保持清理。
四个 direct 阶段仍走原有独立设备路径。

## 现场参数

现场高频参数统一在 `code/competition_task.py` 顶部的
`FIELD / COMPETITION QUICK TUNING` 区域。硬件端口、底盘和传感器配置仍使用
现有 `configs/robocup_diffdrive.toml`。

| 参数 | 当前值/现场要做的事 |
|---|---|
| `START_*` | 起点记录，供现场参考；不重置或变换融合坐标系 |
| `LANE_ENTRY_* / TASK_BOARD_* / CORNER_1_* / CORNER_2_* / FINISH_*` | 全部为 0 的未实测占位值，填写真实融合 XY；入口、任务板观察朝向分别设置 |
| `FINISH_YAW_DEG / CORNER_1_YAW_DEG / CORNER_2_YAW_DEG` | 保留记录值，当前正式车道和终点不以它们做最终朝向对齐 |
| `CROSS_LANE_YAW_DEG / YELLOW_SEARCH_START_* / YELLOW_SEARCH_END_*` | 测量横向车道朝向、搜索起止点；默认搜索段长度为 0 |
| `FIXED_DROP_ROUTE` | 当前仅 `[("drive", 0.0)]`，只用于独立 `drop-route` 调试；full 使用共享融合投放往返 |
| `YELLOW_MIN_CONF` | 0.8，主程序仍只搜索 yellow 类 |
| `YELLOW_TARGET_CX_PX / YELLOW_CX_TOL_PX` | 当前 320 / 32，即横向中央 10%，基于 640×480 参考画面；检测坐标按实际画面尺寸换算 |
| `YELLOW_IMGSZ / CAMERA_WIDTH / CAMERA_HEIGHT / CAMERA_FPS` | 共用 `components/yolo_cpu.py` 的 320 / 640 / 480 / 30；OCR 仍使用自己的采集设置 |
| `ALIGN_PIXEL_TO_DRIVE_SIGN / ALIGN_STEP_M` | 当前 +1 / 0.02 m，实测前后移动对 cx 的影响 |
| `SHORT_MOVE_TOLERANCE_M` | 当前 0.005 m；短距离动作使用融合位置，按纵向距离完成，动作后恢复配置；到位后不转向追点 |
| `PAYLOAD_SLOT_TO_RELAY / PAYLOAD_ACTIVE_ON / PAYLOAD_RELEASE_HOLD_S` | 选项1/2/3对应右前CH2/中间CH3/左前CH4、False（通电吸住/断电释放）、0.5 s；与组件及路线测试共用已确认接线 |
| `CH3_PAYLOAD_SLOT / TARGET_COLOR` | 2 / `yellow`；正式比赛默认中间 CH3，`--payload-slot` 缺省值同为 2 |
| `SEARCH_REFERENCE_CX` | +90° 画面水平中心 320 px；居中后直接左转，没有前进补偿 |
| `DROP_SAFE_M` | 0.43 m；用户 2026-10-11 更新，需重复验证停车超程、漂移和全部车轮范围 |
| `DROP_Y_TOL_PX / DROP_FRAME_TRIES` | 10 px / 3 帧，保留独立预览接口；不参与盲投距离控制 |
| `DROP_FINE_SPEED_M_S` | 0.05 m/s，整条侧向接近限速，不分段重启 PP |
| `DROP_REFERENCE` | 0° 照片历史接口，仅预览；盲投不加载参考照片 |
| `TASK_BOARD_CAMERA / YELLOW_CAMERA` | 当前索引 0，现场优先填稳定的设备路径 |
| `HC_BRIDGE_ENVELOPE / HC_BAUDRATE` | 用户于 2026-10-07 确认 raw / 115200，默认 `/dev/ttyS4` |
| `HC_TASK_MESSAGE_TEMPLATE` | `TASK,{red},{blue},{green}\n`，需要无人机接收程序按此格式解析 |
| `YELLOW_MODEL_PATH` | 仓库已有 `models/best_car.pt`，与路线测试和实测使用相同权重；可用 `--yellow-model` 覆盖 |

这些数值尚未完成实车路线验收。全零坐标会产生零长度车道段，现有 motion 会拒绝，
必须先测量填写不同的端点。使用 `get_current_pose(runtime)` 读取当前融合坐标，直接
填写路线常量；`START_*` 不参与相对坐标变换。
没有加入场地 TOML、路线 loader、舵机扫描、重新标定、障碍绕行或 footprint 判定。

## 原子函数

以下函数可从 `competition_task` 直接导入，调用者拥有 runtime 生命周期并在退出时
调用 `runtime.close()`：

- 定位与运动：`get_current_pose`、`wait_for_fused_localization`、`run_motion_action`、
  `move_to_pose`、`move_to_xy`、`follow_lane_segment`、`turn_to_deg`、`drive_distance`。
- 路线：`go_to_lane`、`go_to_task_board`、`go_to_first_corner`、`enter_cross_lane`、
  `go_to_second_corner`、`go_to_finish`。
- 感知与通信：`read_task_board`、`send_task_to_drone_once`、`detect_yellow_once`、`detect_yellow_from_camera`、
  `search_centered_yellow_on_line`、`align_yellow_drop_zone`。
- 投放与串联：`align_drop_position`、`perform_payload_detour`、`run_drop_align`、
  `run_fixed_drop_route`、`drop_payload`、`run_full_mission`、`run_competition_stage`。

`drop_payload(relay, slot=1/2/3)` 返回 bool，整场与独立投放均使用 `--payload-slot`，默认2
（中间 CH3，继电器通道 3）。盲投的定距接近与停车复测由 `align_drop_position` 完成，可由
`payload_detour.run_payload_detour(..., fine_align=...)` 直接调用，只使用调用方传入的
融合运动闭包，不新建运动控制器；不传 `fine_align` 时同样直接左转，再固定 43 cm 往返。
完整流程在首段移动前给所选电磁铁通电吸住；投放组件断电释放后保持 OFF，
异常和键盘中断也尝试关闭所选路；完整 runtime 负责继电器统一
退出。CLI 的独立 drop 阶段自己负责 `all_off()` 和关闭串口，包含失败/中断退出，
且独立 drop 即使 `disconnect_on_shutdown=False` 也会请求 `all_off()`。
独立 drop 不会先吸合电磁铁；吸持或投放回读未确认会停止任务。
可在已启用的 `[devices.relay]` 配置上运行，也可用 `--relay-port` 显式启用已确认端口。
主程序只在 `full/drop` 接受该端口覆盖，并强制状态回读和退出关闭全部触点。

`detect_yellow_once` 接收已打开且有 `read()` 的 camera 以及 Ultralytics detector。
搜索/对齐函数也支持相机索引或设备路径：内部打开的相机会释放，外部传入的
camera 由调用者关闭。`run_full_mission` 的直接调用需传入 detector；CLI 自动加载。

`search_yellow_drop_zone` 保留旧停车分步搜索的直接调用兼容性，full 和 CLI
`yellow-search` 均使用连续搜索；`yellow-align` 仍可独立执行 2 cm 微调。
full 已在中央范围的检测直接通过对齐，不再重复采集或微调。

CLI 加载的 detector 复用路线测试的 CPU 优化：单线程推理，临时选择当前允许的
最高频 CPU 核心，推理返回或异常后恢复调用线程的亲和性。Ultralytics 首次后端
初始化会重设线程数，包装器在其返回后重新设为 1。首次初始化仍在停车状态进行。
外部直接传入的 detector 由调用者配置线程/亲和性，YOLO 输入仍由任务指定为 320。
2026-10-07 当前红色目标静止实测，现有权重在 320 输入下连续 300 帧采集及识别
约 13.35 FPS；这是当前场景的视觉吞吐量，不代表整个含停车/移动流程的频率，
也未验证黄色目标或远处小目标的识别率。

## CLI 单项入口

以下命令在小车仓库根目录执行，属于现场操作者单独触发的测试。本次编码未执行
这些硬件命令。`task-board`、`hc-send`、`yellow-detect`、`drop` 四个阶段直接使用各自
设备，不构造/启动 D500、T265、SLAM 或底盘 runtime，不要求 `--mode`。它们是实际设备
测试，即使附带原有 `--mode dry-run` 也仍走 direct path；`--mode` 用于定位/运动 runtime。
所有运动阶段及 full 仍使用 `--mode hardware-mission` 并保留原有启动检查。
原有普通 dry-run/replay 模式保持原用途；比赛软件验收使用下方假设备测试。

```bash
# 车停在任务板观察位置，只读 OCR，不导航、不发 HC。
python3 code/main_robocup.py --competition-stage task-board --task-board-camera 0

# 单发一次；独立进程没有已读任务时使用顶部 fallback=(1,2,1)，不自动 OCR。
python3 code/main_robocup.py --competition-stage hc-send

# 只看黄色目标，车不动。沿用已有权重，不训练或下载模型。
python3 code/main_robocup.py --competition-stage yellow-detect --yellow-model /absolute/path/best_car.pt

# 导航到 YELLOW_SEARCH_START，沿搜索线连续识别并在目标横向居中时停车。
python3 code/main_robocup.py --mode hardware-mission --competition-stage yellow-search --yellow-model /absolute/path/best_car.pt

# 从当前位置前后微调；不会先跑搜索或完整路线。
python3 code/main_robocup.py --mode hardware-mission --competition-stage yellow-align --yellow-model /absolute/path/best_car.pt

# 只做 0° 投放精对准：操作员先把车摆成左转结束、面向黄色目标的位姿。
# 舵机转 0°、走 0.38 m 粗靠近并按参考弧顶修正；不释放电磁铁、不返回、不加载 YOLO。
python3 code/main_robocup.py --mode hardware-mission --competition-stage drop-align --yellow-camera 0

# 只跑固定动作；不释放物资。
python3 code/main_robocup.py --mode hardware-mission --competition-stage drop-route

# 只释放指定仓位，不执行路线。
python3 code/main_robocup.py --competition-stage drop --config configs/robocup_diffdrive.toml --payload-slot 2 --relay-port /dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0

# 整场：先填写路线常量，再完成单项验收后使用。
python3 code/main_robocup.py --mode hardware-mission --competition --yellow-model /absolute/path/best_car.pt

# 同一正式场地路线，速度倍率 2，模拟释放（仍会启动真实电机）。
python3 code/main_robocup.py --mode hardware-mission --competition --speed-scale 2 --release-mode simulate

# 单项测试程序：舵机 +90° → 通电吸住所选仓位 → 沿当前融合朝向边走边找目标
# → 目标横向居中后停车 → 直接左转 90° / 融合盲投 43 cm → 释放 → 融合返回 → 结束。
# 不走任务板、HC、车道和终点；搜索线由当前位姿生成，不读比赛路线常量。
python3 tools/run_ch3_drop_test.py --confirm-motor-test --search-distance-m 3 \
  --payload-slot 2 --release-mode relay --relay-port /dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0 \
  --log-dir logs/ch3-drop-test
```

其它 stage：`lane`、`corner1`、`cross-lane`、`corner2`、`finish`、`full`。
独立搜索+投放测试程序在 `tools/run_ch3_drop_test.py`：它复用 `target_patrol` 的连续
搜索与 `competition_task.perform_payload_detour` 的同一套融合投放，只把搜索线换成
"当前位姿沿当前朝向前进 `--search-distance-m`"，因此不需要先跑车道或读板。
`corner1/corner2` 各沿固定车道到对应点；`cross-lane` 转向并沿车道进入黄色搜索起点；
`finish` 沿第二转角到终点的固定线段回线，到达位置后停车。读板的观察点运动可独立调用
`go_to_task_board(runtime)`，`task-board` stage 本身只识别。
`yellow-search` 仍自包含地导航到搜索起点。full 中到搜索起点的小重复动作保持不变。

比赛模式不与旧 `--goal-*`、`--task-board-turn-deg` 同时使用。
`--task-board-camera` 在比赛模式只选择相机，不要求启动转角。
`--task-board-debug-dir` 用于原有 startup 流程和独立 task-board；完整比赛读板参数仍在顶部。

| 单项阶段 | 成功退出码 | 失败退出码 |
|---|---|---|
| `task-board` | 0，含明显 WARNING 的 `(1,2,1)` fallback | 非识别流程的异常为 1 |
| `hc-send / drop` | 0 | 1，drop 清理失败也为 1 |
| `yellow-detect / yellow-search / yellow-align` | 0，有检测/对齐结果 | 1，无结果或异常 |
| `drop-align` | 0，停车 0° 观测有效，无车轮运动和投放 | 1，无有效特征或异常 |
| `drop-route` | 0，所有固定动作正常完成 | 1，动作失败或异常 |
| 其它运动阶段 | 0，正常完成 | 1，runtime safe-stop/error 或异常 |

键盘中断或 SIGTERM 退出码为 130。full 的 OCR/HC/未找到黄色目标沿用比赛容错规则；
完成到终点才返回 0，runtime safety stop/error 或未完成返回 1。

## 视觉资产与依赖

使用现有 `models/best_car.pt`，权重内类别是 red/blue/green/yellow。
推理调用沿用 `target_yolo/verify_model.py` 中的 `model.predict(frame, ...)`。
`yellow_yolo_adapter.py` 仅提取黄色最高置信度 bbox 的中心和大小。
`target_yolo/vision.py` 是数据标注用 HSV 代码，比赛主程序不调用它。

`components/drop_target_vision.py` 的 HSV 全弧拟合仅供 0° 停车预览，不控制投放运动。
正式投放只在左转前用 YOLO 对齐水平画面中心，左转后按融合定位固定前进 43 cm。

板端需要原有 OCR 环境（`requirements-task-board.txt`）和可运行既有模型的
Ultralytics/PyTorch 环境。Ultralytics、模型和相机仅在视觉 stage 中按需加载；
普通启动、纯逻辑单测不打开相机、不加载 YOLO 权重。

## 失败与日志

OCR 重试失败返回 `(1,2,1)`；HC 单发失败继续路线；黄色未找到或对齐失败跳过投放；
继电器投放或回读失败结束任务，不继续第二转角和终点。`cy` 仅记录，不作投放条件。

融合定位短暂掉线沿用 runtime 的停车和恢复策略；完全失效由原安全路径结束。
在停车状态等待融合定位超过 `LOCALIZATION_WAIT_S` 时，也结束定位运动。
现有驱动错误、watchdog、启动错误和急停仍保留。

使用 runtime 的比赛阶段默认事件文件为 `logs/competition/events.jsonl`，也可传
`--log-dir` 选择目录。四个 direct 阶段直接在终端报告结果，不创建定位事件日志。
事件包括阶段开始/完成、任务数量及 fallback、HC 内容/结果、黄色中心、对齐误差、
固定动作、投放结果和终点实际坐标。HC 的 success 表示本地 write 完成，不代表接收
确认；整场投放要求继电器驱动状态回读确认。

## 无硬件验收

```powershell
py -3 -m compileall -q code tools
py -3 -m unittest discover -s code/test -p "test_*.py"
git diff --check
```

当前回归覆盖三仓映射/极性/清理、HC 单发、YOLO 类别/0.8 置信度/中心，
不同于测试矩形的正式路线衔接、速度倍率和自适应前视、原始 T265 平移停滞下的
融合投放往返、不同画面尺寸的中心换算、无目标跳过投放、视觉/释放失败停车，
嵌套动作中的 STOP、整场超时、SIGTERM 和清理。旧独立微调/分步搜索兼容测试保留。
运动验收使用生产 runtime 和 motion 配合假底盘反馈，本轮整合未接真实硬件。

2026-10-11 新回归覆盖：整条侧向接近只有一个融合直线、X 大跳变与圆弧宽度波动
不触发横移、视觉丢失/参考缺失融合定距回退、局部遮挡与挡板变化、车道内固定
正反像素增益、重复时间戳拒绝、3～8 cm 转向平移下从实际 pivot 出发、直倒完成
才转回、CH3 一次释放、停车预览不构造底盘/继电器 runtime。

以下 2026-10-10 记录为旧实现历史，边缘二维精调已在 2026-10-11 删除。

2026-10-10 CH3 视觉精对准验收：本地 Python 3.13 执行 932 项测试，925 项通过、
7 项因环境限制跳过；compileall 与 git diff --check 通过。新增 18 项验收。
未执行真实相机、YOLO 推理、串口、电机或继电器测试。

2026-10-10 上一轮整合验收：本地 Python 3.13 执行 914 项测试，907 项通过、
7 项因环境限制跳过；compileall、git diff --check 和原有 dry-run 均通过。
新增 11 项验收包含主程序完整假设备任务与共享视觉工作线程的实际控制逻辑。

以下 2026-10-07 初版验收记录仅描述当时实现与环境。

2026-10-07 初版本地结果：compileall、git diff --check 和原有 dry-run 通过；
同步 GitHub 最新继电器修复后，默认测试共 775 项，759 项通过、16 项因环境限制
跳过，新增 31 项均通过。
本机 `py -3` 未找到 Python，实际使用 Codex 内置 Python 3.12.14 执行同等检查。
没有执行真实 YOLO 推理、串口发送、电机运动或继电器投放；模型类别仅检查了
现有权重元数据。

本轮补充测试覆盖：终点附近 yaw 与记录值相差很大仍直接完成，主流程固定车道段，
投放偏移后继续使用同一横向车道线，四个 direct 阶段不构造定位 runtime，继电器退出
清理，以及 standalone 失败退出 1 / full 同类失败仍到终点退出 0。
本轮以 `36569d9` 为基线新增 30 项测试；使用 Python 3.12.14 验收，共 805 项，
789 项通过、16 项因环境限制跳过。compileall、git diff --check 和原有 dry-run
均通过。本轮未操作真实相机、串口、继电器或电机。

## 待做的 HC 现场完整行验收

本轮没有改变 HC 协议或增加发送后的等待。现场独立发送 **20 次** `TASK,2,1,1\n`，
使用现有 `send_task_to_drone_once(TaskCounts(2,1,1))`，每次都执行当前的
connect → write 一次 → 立即 close，不加入 ACK 或重发。接收端只做串口打印/记录，
核对收到 **20/20 条完整行**，检查尾部换行和数量字段。发送端 True 只表示本地 write
完成，不能替代接收端实测结果。

若实测确有 close 丢尾字节，再单独修复发送后固定约 0.05 s 等待；没有证据前保持
原有单发实现。此项尚未执行，不能标记为无线链路现场通过。

现场顺序：task-board → hc-send（含上述 20 条完整行验收）→ yellow-detect → drop 空载
→ 定位 get_current_pose → lane → go_to_task_board/第一车道段 → corner1 → cross-lane
→ yellow-search → yellow-align → drop-align（车摆成左转结束、面向目标位姿）
→ drop-route 不装物资 → drop 装测试物 → corner2 → finish → full。前四项不依赖完整定位 runtime。
