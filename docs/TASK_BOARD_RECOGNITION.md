# 开局任务板识别

功能入口是 `code/main_robocup.py` 的 `hardware-mission` 模式。显式提供
`--task-board-camera` 和 `--task-board-turn-deg` 后，在后续导航目标启动前执行：

```text
等待定位 READY → 按实测角转向 → TARGET_OPERATION 停车
→ 持续 step() 检查定位并停稳 0.3 s → 相机预热 6 帧
→ 最多识别 10 帧，3 票一致接受 → 再检查定位
→ 保存 mission.task_counts 和识别日志 → READY，继续原任务流程
```

启动此命令就是操作员在裁判开始后的触发动作；代码不监听裁判信号，也不自动判断遮板是否揭开。
未提供这两个参数时沿用已有运行流程。任务板识别组件不控制底盘，不进入
`RobocupRuntime.step()` 周期路径，不使用 YOLO/RKNN，也不写死示例 2/1/1。

## 数据与失败行为

`TaskCounts(red, blue, green)` 只接受整数 0..4，且总和必须为 4。
OCR 支持阿拉伯数字、零/〇/一/二/两/三/四、繁体颜色及同一行多个 OCR box。
数量不能越过下一颜色锚点；多位数、负数、小数、越界数字和同颜色冲突会明确失败。

只缺一个颜色时，可根据总和唯一补全，降低 confidence 并记录 inferred_colors；
两个颜色缺失不能补全。业务流程只接受默认 3 票一致结果；单张图片工具只做
单帧检查，不能将其 votes=1 的结果直接当作比赛开局验收。

定位/转向默认超时 30 s；无法转向、相机/OCR 异常、票数不足或识别后定位丢失均
请求 SAFE_STOP，不赋值任务。KeyboardInterrupt 先停车再交给主入口清理。
相机在识别或异常后释放，后续视觉可再占用。OCR 的相机 read 和模型执行是阻塞调用，
30 s 超时只约束定位、转向和停稳阶段；识别阶段底盘已停车。

成功事件 `task_board_recognition` 含 red/blue/green、confidence、votes、
inferred_colors、raw_lines、转向角及各阶段耗时；失败事件
`task_board_recognition_failed` 含 reason。启用后如未设置 `--log-dir`，
识别日志默认在 `logs/task-board/events.jsonl`。任务分配策略仍由后续任务模块
单独记录 `task_allocation`，本组件不决定小车/无人机分工。

## 安装与离线验证

OCR 依赖与底盘依赖分开，导入模块和构造 reader 不加载模型、不打开相机、不启动线程。
RapidOCR 第一次实际读取时加载。开发机可建立独立环境：

```powershell
py -3 -m venv .venv-task-board
.venv-task-board\Scripts\python.exe -m pip install -r requirements-task-board.txt
.venv-task-board\Scripts\python.exe tools\task_board_demo.py --image C:\Users\TZDEZACR\Downloads\robocup_task_board_package\synthetic\view_00_front.png
.venv-task-board\Scripts\python.exe tools\validate_task_boards.py C:\Users\TZDEZACR\Downloads\robocup_task_board_package\synthetic --ocr --output logs\task-board\synthetic.json
```

本次已在项目内 `.venv-task-board` 安装 RapidOCR 3.9.2、ONNX Runtime 1.30.0、
OpenCV 5.0.0、NumPy 2.5.3，并执行真实 CPU OCR。Python 为 3.13.12。
逐图结果保存在 `docs/task_board_synthetic_results.json`。
合成图留在外部任务包；仓库只新增可复跑的验证脚本与文本报告。

Rock 5A 使用独立环境并优先复用已有系统 OpenCV。RapidOCR 的包依赖会引入
`opencv-python`；如需保留系统 V4L2/GStreamer 构建，可先确认 `import cv2`，
在 `--system-site-packages` 环境显式安装其他依赖后，以 `--no-deps` 安装 RapidOCR：

```bash
python3 -m venv --system-site-packages .venv-task-board
.venv-task-board/bin/python -m pip install 'onnxruntime>=1.23,<2' 'numpy>=1.24,<3' pyclipper shapely PyYAML Pillow six tqdm omegaconf requests colorlog
.venv-task-board/bin/python -m pip install --no-deps rapidocr==3.9.2
.venv-task-board/bin/python tools/task_board_demo.py --camera /dev/v4l/by-id/ACTUAL_CAMERA --debug-dir logs/task-board/debug
```

离线部署前需在板端预先执行一次静态识别并确认模型文件可用；本次开发机安装包含所用
PP-OCRv6 检测/识别模型。不要在首次比赛启动时才确认依赖和模型。

## 已部署的小车任务环境（2026-10-05）

本车实际任务与 ROS2/SLAM 共用 `/home/radxa/robocup_ros/env`。按用户要求，
RapidOCR 3.9.2、ONNX Runtime 1.30.0 已直接安装到该环境；现有 OpenCV 4.12.0、
NumPy 1.26.4、PyYAML 6.0.3、six 1.17.0 和 packaging 26.3 保持原版本。
已在该环境验证 rclpy、pyrealsense2、OCR 导入及全部 14 张合成图，12 个标注视角
均识别为红 2 / 蓝 1 / 绿 1，无颜色推断。静态视角图单帧约 0.9–1.0 s。
详见 [小车同步记录](CAR_SYNC_20261005.md)。

可在同一个任务环境重新执行静态检查（不打开相机或底盘）：

```bash
export MAMBA_ROOT_PREFIX=/home/radxa/robocup_ros/root
export PYTHONPATH=/home/radxa/robocup_ros/python_ext:/home/radxa/car/code
cd /home/radxa/car
/home/radxa/robocup_ros/bin/micromamba run -p /home/radxa/robocup_ros/env \
  python tools/task_board_demo.py \
  --image /home/radxa/.cache/car-sync/20261005_1637/synthetic/view_00_front.png
```

比赛任务也使用 `micromamba run -p /home/radxa/robocup_ros/env python` 执行主入口，
保留上述 PYTHONPATH。下面的独立环境启动示例适用于另外配置的部署，
本车无需为了任务板识别切换解释器。

## 实机启动

下面的环境变量必须由现场测量填写，不提供默认 +90°。正数为车体逆时针 yaw，
相机方向、任务板位置决定符号。相机使用稳定设备路径。

```bash
.venv-task-board/bin/python code/main_robocup.py \
  --config configs/robocup_diffdrive.toml --mode hardware-mission \
  --task-board-camera "$TASK_BOARD_CAMERA" \
  --task-board-turn-deg "$TASK_BOARD_TURN_DEG" \
  --log-dir logs/task-board-run
```

当前硬件 readiness、定位和电机安全门禁仍生效。若配置/固件不支持原地转向，
流程会停车失败；应先完成现有底盘转向验证或另外实现上层相机舵机朝向步骤，
不能仅为了 OCR 改成未经验证的差速协议。
`main_robocup` 目前的默认硬件定位采用 relative SLAM；`--goal-*` 按当前融合
位姿坐标系解释。该直线导航不使用静态地图，也不提供障碍避让。

## 2026-10-05 验收记录

- 任务包调研基线：`792d9090161a0f401250fa38a31228d015f92642`。
- 本地实际基线/当前 HEAD：`81d67fb470b0bde6e90a3a796f497ae3f8bff087`；已审查后续差异并适配，未回退。
- 改动保留在工作区，本次未创建提交或推送。开工前已有两个 service 修改及
  `tools/battery_alarm_chain_test.py` 均保留。旧 SSH 传输工具已停用，后续按仓库根目录 `AGENTS.md` 经 GitHub 部署。
- 基线：compileall 通过；751 项测试通过，8 项跳过。
- 修改后：compileall 通过；790 项测试通过，8 项跳过；git diff --check 通过。
- 新增 39 项测试，其中 27 项覆盖解析/几何/投票/相机清理/导入边界，12 项覆盖启动集成。
  一项遍历全部 15 种合法数量组合，验证未将示例写死。
- 合成几何：12/12 找到任务板；平均角点误差 8.332 px，最差单图平均 8.807 px。
- 真实模型 OCR：12/12 正确，均通过 rectified 路径，未使用单色补全。
- 软件启动集成：真实 runtime/motion 控制器、模拟定位和 fake reader 的 +90° 转向、
  停车、任务赋值及日志通过；识别失败、阻塞转向、定位/转向超时、定位丢失、
  OCR 异常、中断均验证停车，不加载真实模型进普通单测。
- 实际打印板：未执行；本次没有提供现场照片/视频。
- 真车 90° 开局姿态下的转向/停稳/识别：未执行；没有操作真实电机或相机。
- Rock 5A 实际任务环境静态推理已验证，见同步记录；相机安装距离、曝光和 30 次连续开局成功率仍待现场测试。

| 合成图 | 几何 | 真 OCR | confidence | 单帧耗时 ms |
|---|---|---|---:|---:|
| view_00_front | 通过 | 2/1/1 | 0.936770 | 3672 |
| view_01_yaw_left_15 | 通过 | 2/1/1 | 0.935970 | 2918 |
| view_02_yaw_right_15 | 通过 | 2/1/1 | 0.927050 | 3423 |
| view_03_yaw_left_30 | 通过 | 2/1/1 | 0.937603 | 3251 |
| view_04_yaw_right_30 | 通过 | 2/1/1 | 0.933660 | 3092 |
| view_05_pitch_up_12 | 通过 | 2/1/1 | 0.935627 | 3006 |
| view_06_pitch_down_12 | 通过 | 2/1/1 | 0.946310 | 3224 |
| view_07_compound_left | 通过 | 2/1/1 | 0.932487 | 3320 |
| view_08_compound_right | 通过 | 2/1/1 | 0.960270 | 3425 |
| view_09_far | 通过 | 2/1/1 | 0.957617 | 3206 |
| view_10_dim | 通过 | 2/1/1 | 0.999300 | 3295 |
| view_11_soft_motion | 通过 | 2/1/1 | 0.934570 | 3203 |

耗时来自 Windows 开发机 CPU，一次批量测试，首图包含模型初始化，不能替代 Rock 5A 数据。

新增：`code/components/task_board_reader.py`、`code/task_board_startup.py`、
`code/test/test_task_board_reader.py`、`code/test/test_task_board_startup.py`、
`tools/task_board_demo.py`、`tools/validate_task_boards.py`、`requirements-task-board.txt`、
本说明和逐图 JSON。修改生产入口、mission 的任务保存及 runtime 日志接口、README、
引用说明和 .gitignore。

上述测试数量为任务板实现阶段的记录。用户随后授权提前清理旧阿克曼逻辑，
旧任务/转向/赛道/视觉/v1 配置/串口屏启动器及 ROS2 导航输出已移除。
当前测试数量、删除清单及保留的共享资产见 `LEGACY_CLEANUP.md`。
任务板的真实打印板、相机及真车开局验收仍未执行。
