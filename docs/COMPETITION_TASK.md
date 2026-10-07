# 2026 小车比赛主程序与逐项调试

入口仍是 `code/main_robocup.py`。比赛流程在 `code/competition_task.py`，复用现有
融合定位、BasicMotionController、DifferentialDrive、OCR、HC15SerialDriver、LCUSRelay。
本次未改动 D500/T265/融合算法、C10B 帧发送器或底盘 watchdog。

完整流程：等待融合定位 → 车道入口 → 任务板观察点 → OCR → HC 单发 → 第一转角
→ 横向车道 → 黄色搜索 → 像素对齐 → 固定投放路线 → 所选 slot 投放 → 第二转角
→ 终点坐标停车。

正式车道使用现有 `follow_segment()`，按测定的线段方向运行和回线：

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
| `FIXED_DROP_ROUTE` | 当前仅 `[("drive", 0.0)]`；实测后填写 drive/rotate/rotate_to 序列 |
| `YELLOW_TARGET_CX_PX / YELLOW_CX_TOL_PX` | 当前 320 / 10，基于 640×480 参考画面；非此尺寸的检测坐标先按比例换算 |
| `YELLOW_IMGSZ / CAMERA_WIDTH / CAMERA_HEIGHT / CAMERA_FPS` | 共用 `components/yolo_cpu.py` 的 320 / 640 / 480 / 30；OCR 仍使用自己的采集设置 |
| `ALIGN_PIXEL_TO_DRIVE_SIGN / ALIGN_STEP_M` | 当前 +1 / 0.02 m，实测前后移动对 cx 的影响 |
| `SHORT_MOVE_TOLERANCE_M` | 当前 0.005 m；短距离动作临时缩小现有 3 cm 容差，动作后恢复 |
| `PAYLOAD_SLOT_TO_RELAY / PAYLOAD_ACTIVE_ON / PAYLOAD_RELEASE_HOLD_S` | 选项1/2/3对应右前CH2/中间CH3/左前CH4、False（通电吸住/断电释放）、0.5 s；与组件及路线测试共用已确认接线 |
| `TASK_BOARD_CAMERA / YELLOW_CAMERA` | 当前索引 0，现场优先填稳定的设备路径 |
| `HC_BRIDGE_ENVELOPE / HC_BAUDRATE` | 用户于 2026-10-07 确认 raw / 115200，默认 `/dev/ttyS4` |
| `HC_TASK_MESSAGE_TEMPLATE` | `TASK,{red},{blue},{green}\n`，需要无人机接收程序按此格式解析 |
| `YELLOW_MODEL_PATH` | 仓库已有 `models/best_car.pt`，与路线测试和实测使用相同权重；可用 `--yellow-model` 覆盖 |

这些数值尚未完成实车路线验收。全零坐标会产生零长度车道段，现有 motion 会拒绝，
必须先测量填写不同的端点。使用 `get_current_pose(runtime)` 读取当前融合坐标，直接
填写路线常量；`START_*` 不参与相对坐标变换。
没有加入场地 TOML、路线 loader、舵机扫描、相机标定、障碍绕行或 footprint 判定。

## 原子函数

以下函数可从 `competition_task` 直接导入，调用者拥有 runtime 生命周期并在退出时
调用 `runtime.close()`：

- 定位与运动：`get_current_pose`、`wait_for_fused_localization`、`run_motion_action`、
  `move_to_pose`、`move_to_xy`、`follow_lane_segment`、`turn_to_deg`、`drive_distance`。
- 路线：`go_to_lane`、`go_to_task_board`、`go_to_first_corner`、`enter_cross_lane`、
  `go_to_second_corner`、`go_to_finish`。
- 感知与通信：`read_task_board`、`send_task_to_drone_once`、`detect_yellow_once`、`detect_yellow_from_camera`、
  `search_yellow_drop_zone`、`align_yellow_drop_zone`。
- 投放与串联：`run_fixed_drop_route`、`drop_payload`、`run_full_mission`、
  `run_competition_stage`。

`drop_payload(relay, slot=1/2/3)` 返回 bool，整场与独立投放均使用 `--payload-slot`，默认1。
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

# 搜索会先导航到 YELLOW_SEARCH_START，然后在该段停车识别、短步前进。
python3 code/main_robocup.py --mode hardware-mission --competition-stage yellow-search --yellow-model /absolute/path/best_car.pt

# 从当前位置前后微调；不会先跑搜索或完整路线。
python3 code/main_robocup.py --mode hardware-mission --competition-stage yellow-align --yellow-model /absolute/path/best_car.pt

# 只跑固定动作；不释放物资。
python3 code/main_robocup.py --mode hardware-mission --competition-stage drop-route

# 只释放指定仓位，不执行路线。
python3 code/main_robocup.py --competition-stage drop --config configs/robocup_diffdrive.toml --payload-slot 1 --relay-port /dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0

# 整场：先填写路线常量，再完成单项验收后使用。
python3 code/main_robocup.py --mode hardware-mission --competition --yellow-model /absolute/path/best_car.pt
```

其它 stage：`lane`、`corner1`、`cross-lane`、`corner2`、`finish`、`full`。
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
| `drop-route` | 0，所有固定动作正常完成 | 1，动作失败或异常 |
| 其它运动阶段 | 0，正常完成 | 1，runtime safe-stop/error 或异常 |

键盘中断退出码为 130。full 的 OCR/HC/黄色视觉/投放失败继续遵循比赛容错规则；
完成到终点才返回 0，runtime safety stop/error 或未完成返回 1。

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

使用 runtime 的比赛阶段默认事件文件为 `logs/competition/events.jsonl`，也可传
`--log-dir` 选择目录。四个 direct 阶段直接在终端报告结果，不创建定位事件日志。
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
→ yellow-search → yellow-align → drop-route 不装物资 → drop 装测试物 → corner2
→ finish → full。前四项不依赖完整定位 runtime。
