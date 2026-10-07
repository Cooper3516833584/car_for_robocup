# 2026 小车比赛主程序与逐项调试

入口仍是 `code/main_robocup.py`。比赛流程在 `code/competition_task.py`，复用现有
融合定位、BasicMotionController、DifferentialDrive、OCR、HC15SerialDriver、LCUSRelay。
本次未改动 D500/T265/融合算法、C10B 帧发送器或底盘 watchdog。

完整流程：等待融合定位 → 车道入口 → 任务板观察点 → OCR → HC 单发 → 第一转角
→ 横向车道 → 黄色搜索 → 像素对齐 → 固定投放路线 → slot 1 投放 → 第二转角
→ 终点坐标停车。

## 现场参数

现场高频参数统一在 `code/competition_task.py` 顶部的
`FIELD / COMPETITION QUICK TUNING` 区域。硬件端口、底盘和传感器配置仍使用
现有 `configs/robocup_diffdrive.toml`。

| 参数 | 当前值/现场要做的事 |
|---|---|
| `START_*` | 起点记录，供现场参考；不重置或变换融合坐标系 |
| `LANE_ENTRY_* / TASK_BOARD_* / CORNER_1_* / CORNER_2_* / FINISH_*` | 全部为 0 的未实测占位值，填写真实融合坐标和朝向 |
| `CROSS_LANE_YAW_DEG / YELLOW_SEARCH_START_* / YELLOW_SEARCH_END_*` | 测量横向车道朝向、搜索起止点；默认搜索段长度为 0 |
| `FIXED_DROP_ROUTE` | 当前仅 `[("drive", 0.0)]`；实测后填写 drive/rotate/rotate_to 序列 |
| `YELLOW_TARGET_CX_PX / YELLOW_CX_TOL_PX` | 当前 640 / 20，按真实 1280×720 画面调节 |
| `ALIGN_PIXEL_TO_DRIVE_SIGN / ALIGN_STEP_M` | 当前 +1 / 0.02 m，实测前后移动对 cx 的影响 |
| `SHORT_MOVE_TOLERANCE_M` | 当前 0.005 m；短距离动作临时缩小现有 3 cm 容差，动作后恢复 |
| `PAYLOAD_SLOT_TO_RELAY / PAYLOAD_ACTIVE_ON / PAYLOAD_RELEASE_HOLD_S` | 当前 CH1/2/3、True、0.5 s，确认接线和释放极性 |
| `TASK_BOARD_CAMERA / YELLOW_CAMERA` | 当前索引 0，现场优先填稳定的设备路径 |
| `HC_BRIDGE_ENVELOPE / HC_BAUDRATE` | 用户于 2026-10-07 确认 raw / 115200，默认 `/dev/ttyS4` |
| `HC_TASK_MESSAGE_TEMPLATE` | `TASK,{red},{blue},{green}\n`，需要无人机接收程序按此格式解析 |
| `YELLOW_MODEL_PATH` | 本地现有 `../target_yolo/best_car.pt`，板端部署后明确指定实际权重路径 |

这些数值尚未完成实车路线验收。全零坐标不会生成真实赛道路线，必须先测量填写。
没有加入场地 TOML、路线 loader、舵机扫描、相机标定、障碍绕行或 footprint 判定。

## 原子函数

以下函数可从 `competition_task` 直接导入，调用者拥有 runtime 生命周期并在退出时
调用 `runtime.close()`：

- 定位与运动：`get_current_pose`、`wait_for_fused_localization`、`run_motion_action`、
  `move_to_pose`、`follow_lane_segment`、`turn_to_deg`、`drive_distance`。
- 路线：`go_to_lane`、`go_to_task_board`、`go_to_first_corner`、`enter_cross_lane`、
  `go_to_second_corner`、`go_to_finish`。
- 感知与通信：`read_task_board`、`send_task_to_drone_once`、`detect_yellow_once`、
  `search_yellow_drop_zone`、`align_yellow_drop_zone`。
- 投放与串联：`run_fixed_drop_route`、`drop_payload`、`run_full_mission`、
  `run_competition_stage`。

`drop_payload(relay, slot=1/2/3)` 返回 bool，当前整场仅调用 slot 1。
投放组件在异常和键盘中断时也尝试恢复非释放状态；继电器统一退出仍由 runtime
负责 `all_off()` 和关闭串口。

`detect_yellow_once` 接收已打开且有 `read()` 的 camera 以及 Ultralytics detector。
搜索/对齐函数也支持相机索引或设备路径：内部打开的相机会释放，外部传入的
camera 由调用者关闭。`run_full_mission` 的直接调用需传入 detector；CLI 自动加载。

## CLI 单项入口

以下命令在小车仓库根目录执行，属于现场操作者单独触发的测试。本次编码未执行
这些硬件命令。比赛 CLI 使用 `hardware-mission`，保留现有 runtime 的启动检查。
原有 dry-run/replay 模式保持原用途；比赛软件验收使用下方假设备测试。

```bash
# 车停在任务板观察位置，只读 OCR，不导航、不发 HC。
python3 code/main_robocup.py --mode hardware-mission --competition-stage task-board --task-board-camera 0

# 单发一次；独立进程没有已读任务时使用顶部 fallback=(1,2,1)，不自动 OCR。
python3 code/main_robocup.py --mode hardware-mission --competition-stage hc-send

# 只看黄色目标，车不动。沿用已有权重，不训练或下载模型。
python3 code/main_robocup.py --mode hardware-mission --competition-stage yellow-detect --yellow-model /absolute/path/best_car.pt

# 搜索会先导航到 YELLOW_SEARCH_START，然后在该段停车识别、短步前进。
python3 code/main_robocup.py --mode hardware-mission --competition-stage yellow-search --yellow-model /absolute/path/best_car.pt

# 从当前位置前后微调；不会先跑搜索或完整路线。
python3 code/main_robocup.py --mode hardware-mission --competition-stage yellow-align --yellow-model /absolute/path/best_car.pt

# 只跑固定动作；不释放物资。
python3 code/main_robocup.py --mode hardware-mission --competition-stage drop-route

# 只释放指定仓位，不执行路线。
python3 code/main_robocup.py --mode hardware-mission --competition-stage drop --payload-slot 1

# 整场：先填写路线常量，再完成单项验收后使用。
python3 code/main_robocup.py --mode hardware-mission --competition --yellow-model /absolute/path/best_car.pt
```

其它 stage：`lane`、`corner1`、`cross-lane`、`corner2`、`finish`、`full`。
`corner1/corner2` 各导航到对应点及朝向；`cross-lane` 转向并进入黄色搜索起点；
`finish` 从当前位置导航到终点并停车。读板的观察点运动可独立调用
`go_to_task_board(runtime)`，`task-board` stage 本身只识别。

比赛模式不与旧 `--goal-*`、`--task-board-turn-deg` 同时使用。
`--task-board-camera` 在比赛模式只选择相机，不要求启动转角。
`--task-board-debug-dir` 目前用于原有 startup 流程。

## 视觉资产与依赖

使用现有 `target_yolo/best_car.pt`，权重内类别是 red/blue/green/yellow。
推理调用沿用 `target_yolo/verify_model.py` 中的 `model.predict(frame, ...)`。
`yellow_yolo_adapter.py` 仅提取黄色最高置信度 bbox 的中心和大小。
`target_yolo/vision.py` 是数据标注用 HSV 代码，比赛主程序不调用它。

板端需要原有 OCR 环境（`requirements-task-board.txt`）和可运行既有模型的
Ultralytics/PyTorch 环境。Ultralytics、模型和相机仅在视觉 stage 中按需加载；
普通启动、纯逻辑单测不打开相机、不加载 YOLO 权重。

## 失败与日志

OCR 重试失败返回 `(1,2,1)`；HC 单发失败继续路线；黄色未找到或对齐失败跳过投放；
继电器投放失败继续第二转角和终点。`cy` 仅记录，不作投放条件。

融合定位短暂掉线沿用 runtime 的停车和恢复策略；完全失效由原安全路径结束。
在停车状态等待融合定位超过 `LOCALIZATION_WAIT_S` 时，也结束定位运动。
现有驱动错误、watchdog、启动错误和急停仍保留。

默认事件文件 `logs/competition/events.jsonl`，也可传 `--log-dir` 选择目录。
事件包括阶段开始/完成、任务数量及 fallback、HC 内容/结果、黄色中心、对齐误差、
固定动作、投放结果和终点实际坐标。HC 的 success 表示本地 write 完成，不代表接收
确认；投放的 success 依循继电器驱动返回值，默认不回读。

## 无硬件验收

```powershell
py -3 -m compileall -q code tools
py -3 -m unittest discover -s code/test -p "test_*.py"
git diff --check
```

新增测试覆盖：三仓映射/极性/恢复，HC 一次 write/关闭，YOLO 类别/置信度/中心，
连续运动衔接，2 cm 实际步进，搜索段末步限制，多帧确认，对齐方向/丢失/次数上限，
定位短暂掉线恢复和完全失效停车，以及整场 HC/投放失败后继续到终点。
运动验收使用生产 runtime 和 motion 配合假底盘反馈，未接真实硬件。

2026-10-07 本地结果：compileall、git diff --check 和原有 dry-run 通过；
同步 GitHub 最新继电器修复后，默认测试共 775 项，759 项通过、16 项因环境限制
跳过，新增 31 项均通过。
本机 `py -3` 未找到 Python，实际使用 Codex 内置 Python 3.12.14 执行同等检查。
没有执行真实 YOLO 推理、串口发送、电机运动或继电器投放；模型类别仅检查了
现有权重元数据。

现场顺序：读定位 → 单段路线 → OCR → HC → 黄色只看图 → 搜索 → 对齐 → 固定路线
→ 单仓投放 → 黄色组合流程 → 后半程 → 整场。
