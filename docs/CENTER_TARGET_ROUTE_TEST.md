# 目标减速与侧向投放巡线测试

入口：`tools/run_center_target_route.py`。只使用生产 runtime 的融合定位、
motion 动作和 DifferentialDrive 输出，不调用比赛占位坐标。默认执行模拟投放，
真实继电器保持关闭；载荷动作复用现有 `components/payload_task.py` 和 LCUS 驱动，
本次不增加继电器驱动。

路线从启动时的融合位置/朝向生成：前进 280 cm → 左转 90° → 前进
420 cm → 左转 90° → 前进 250 cm。位置与转角完成容差沿用本车配置。

摄像头舵机先转到仓库标定的 **+90°（2500 us）**，整个任务及退出后保持
PWM。这里的角度沿用 ServoAxis 的 ±90° 坐标，0° 是中位 1500 us。

YOLO 使用 `models/best_car.pt`（原有训练权重，red/blue/green/yellow 全部参与）。
置信度至少 0.8（2026-10-10 调整）。直行巡线时目标出现在画面内，动作层线速度上限降到
8 cm/s，速度变化仍经过 DifferentialDrive 的原加减速限制。目标消失时恢复
原巡线速度上限。目标框中心 X 进入画面横向中央 10%（45%～55%）才触发投放；
上下位置不参与触发，多个检测框中任意合格目标可触发。

可复用模块 `code/payload_detour.py` 的动作是：沿当前方向再前进 7 cm → 停车
→ 左转 90° → 前进 47 cm → 投放 → 后退 47 cm → 右转 90° → 恢复原巡线路段。
左右转使用同一保存的道路朝向，防止角度误差累加；直行投放动作临时使用
5 mm 纵向距离完成容差；横向偏差持续交给 PP 纠偏并记录，结束或异常后恢复原设置。
直行距离到位后直接停车，不转向追逐横向残差；前进和后退47 cm共用保存的侧向朝向。
2026-10-10 按用户要求删除几何偏差中止门限：沿线达到或超过终点即双零完成本段，
终点横偏、超程及投放位置/朝向偏差只记录日志，继续任务。投放前发送零速指令并记录一次 `payload_drop_pose`，随后直接释放，删除速度/角速度
稳定样本检测及其 2 秒超时；定位失效、设备故障、STOP与任务超时仍停车。
转向控制器自身的朝向到位与完成条件沿用原控制逻辑。实车精度尚需验收。
前进的 7 cm 计入原巡线距离，返回后继续原终点，不重走整段。

默认 `--release-mode simulate`，使用4路内存继电器，只记录模拟释放，不打开
真实继电器。`--payload-slot 1/2/3` 分别选择右前 CH2、中间 CH3、左前 CH4，默认1；
CH1 未连接电磁铁。首次移动前所选电磁铁通电吸住，投放时断电并保持关闭，
默认等待0.5 s后返回；模拟模式走相同状态变化。本次运行固定选择该路，
返回后再次触发也不会重新吸合已释放的电磁铁。投放/返回期间不接受新触发，也不因转向时
目标离开画面而重新允许触发。返回巡线后需连续三个不同的新鲜画面无目标，
才允许再次触发。持续可见的同一目标只投放一次。

`--target-speed-cm-s` 调整可见目标时的速度上限；
`--drop-center-width-ratio` 调整横向中部范围。例如0.2代表横向40%～60%。
`--target-action beep` 保留原中央50%（同时约束X/Y）立即停车、鸣叫1 s模式。

相机/推理错误、结果超过 2 s 未更新、定位安全停止、超时、SIGINT/SIGTERM
或 STOP 文件会结束测试并停车、关闭蜂鸣器。默认总时限 300 s。
首次加载模型必须在停车状态完成；不自动下载替代模型。
`tools/benchmark_target_yolo.py` 可在停车状态测量摄像头读取、导入、模型加载、
预处理、网络推理和后处理，对比线程数、大小核亲和性及 416/320 输入尺寸。

默认与比赛程序共用 `components/yolo_cpu.py`：摄像头 640×480、30 FPS、
缓冲 1 帧，YOLO 输入 320，使用现有权重和单线程 CPU 推理。2026-10-07
停车实测当前中央红色目标：416 输入约 8.70 FPS，320 连续 300 帧约 13.35 FPS，
300 帧均检出，置信度中位数 0.93；256 约 17.08 FPS，但置信度约 0.81。
这些是实时采集及识别的吞吐量，远处小目标和黄色目标仍需现场验证。
`tools/benchmark_target_yolo_live.py` 使用与任务相同的 detector 包装器，保存
实时帧耗时、检测框、置信度和标注图，可复查尺寸变化的影响。

Ultralytics CPU 后端初始化会覆盖预设的 PyTorch 线程数；共用包装器在每次
推理返回或异常后重新设为 1，再用后续新鲜帧声明就绪。最高频 CPU 组按当前
设备动态选择（本次实测为 4/5/6/7），亲和性仅在视觉工作线程设置。
不改变主控/定位线程的亲和性；PyTorch 线程数为进程级设置。
首帧初始化不参与移动时的目标判断。

板端先启动新 SLAM 会话（参见 `SLAM_TOOLBOX_ROCK5A.md`），再使用同时具备
ROS、pyrealsense2、OpenCV、PyTorch、Ultralytics 的 Python 环境启动：

本次板端使用 `/home/radxa/robocup_ros/vision_env`，由已有 ROS Python 创建
带 `--system-site-packages` 的独立 venv；CPU 依赖版本记录在
`tools/requirements_center_target_route.txt`。不替换已有 ROS 环境的包。
首次 PWM 导出后允许 udev 最多 2 s 赋予 `pwm` 组权限；仍拒绝则结束测试。
冷启动慢帧会被丢弃，只有两秒以内的新鲜识别结果才允许开始运动。
可先执行 `python tools/run_center_target_route.py --check-vision`，只检查
摄像头/模型延迟，不打开运动、蜂鸣器或舵机。

```bash
python tools/run_center_target_route.py --confirm-motor-test \
  --payload-slot 1 --release-mode simulate \
  --log-dir /home/radxa/car_test_logs/center-route-YYYYMMDD-HHMMSS
```

这条命令仍会真实运行小车的路线和投放往返动作，只有继电器为模拟。
`--startup-alarm-seconds 10` 在运动设备启动前先确认全部继电器关闭，
给所选电磁铁通电吸持，再鸣叫10 s并关闭蜂鸣器，之后准备视觉/定位并开始路线。
它只适用于drop模式，默认0（不鸣叫）。中间电磁铁使用 `--payload-slot 2`；
真实投放同时显式传入 `--release-mode relay --relay-port <已确认路径> --relay-channels 4`。
STOP或信号可中断鸣叫，异常会关闭蜂鸣器和全部继电器。
本次接线与释放逻辑变更进行软件模拟验收后，通过 GitHub 同步到板端，
未启动此运动命令。可先执行 `--check-vision` 检查当前视觉状态。

真实投放须显式选择 `--release-mode relay
--relay-port /dev/serial/by-path/platform-fc8c0000.usb-usb-0:1:1.0-port0
--relay-channels 4`。现有驱动负责通电吸住和断电释放，退出必须 all_off。
投放或关闭状态未确认会停止任务，不自动倒车或恢复巡线。实际板子为4路，
用户于2026-10-07逐路确认接线；投放使用CH2～CH4。通电吸持未确认时不会开始巡线。

2026-10-07 中间电磁铁实车测试在第二次返回时因 `reverse_goal_requires_turn` 停止，
未完成整条路线。日志证实47 cm到位后的点位闭合控制引发额外转角；修复和离线验证见
`PAYLOAD_MOTION_DIAG_20261007.md`。该修复没有自动重新启动实车。

紧急停止可在 SSH 中执行 `touch <本次日志目录>/STOP`；运行输出、事件和
`result.txt` 可用于核对动作与报警。执行测试已由用户明确授权。

## 临时黄色投放演示（2026-10-08）

为拍摄演示显式使用 `--demo-no-position-checks`。该选项独立执行整段
280cm→左90°→420cm→左90°→250cm路线，不启动T265、D500、SLAM或位置融合，
不因横向偏差、航向偏差、位置跳变或定位丢失中断。运动量由驱动限幅后的
速度指令及时间估算，包括指令加速阶段；不是实测距离或角度，不主动纠偏。
不能用演示完成结果判断定位精度或宣布起步滞后修复。未加此选项时仍运行
原定位控制与保护流程；无需修改配置即可切回。

演示只处理YOLO权重中类别名为 `yellow` 的目标，按权重的实际类别映射查找，
不写死数字类别ID。红、蓝、绿不会引起减速或投放。缺少yellow类别、摄像头
错误或结果过期会停车。黄色目标出现在画面中开始减速，进入水平中间10%
后（不限制上下位置）前进7cm、停车左90°、前进47cm、断电投放、后退47cm、
右90°，然后继续剩余路线；额外7cm计入本段巡线路程。转后停顿1秒。
持续出现的同一目标不会连续触发，返回后需要三个连续无遮挡新帧再重新武装。

演示模式默认真实继电器投放，使用已确认的by-path串口和4路板；
默认仅吸持中间电磁铁slot2/CH3，投放时断电，释放后不重新吸合。
启动时只关闭未选中的通道，再确认所选通道ON；若操作者已提前吸持物品，
不会在初始化时对该通道发送OFF。异常或退出仍关闭全部通道。
其他通道保持关闭；`--payload-slot 1/2/3`仍分别选择右前CH2/中间CH3/左前CH4。
若需只看动作而不操作真实继电器，可显式加 `--release-mode simulate`。
舵机先转90°并保持；默认巡航15cm/s，看到黄色时8cm/s，转弯请求0.4rad/s，
全部指令仍经过原驱动限速、加速度、串口看门狗和独占硬件锁。
`--demo-speed-cm-s`及`--demo-turn-rad-s`可以调整估算速度，但不能超过配置上限。

无需启动SLAM，使用已有视觉环境启动。日志目录每次必须新建：

```bash
cd /home/radxa/car
PYTHONPATH=/home/radxa/car/code /home/radxa/robocup_ros/vision_env/bin/python \
  tools/run_yellow_payload_demo.py \
  --confirm-motor-test --servo-angle-deg 90 --payload-slot 2 \
  --log-dir /home/radxa/car_test_logs/yellow-payload-demo-YYYYMMDD-HHMMSS
```

仅检查黄色识别、不开电机/舵机/继电器：

```bash
PYTHONPATH=/home/radxa/car/code /home/radxa/robocup_ros/vision_env/bin/python \
  tools/run_yellow_payload_demo.py --check-vision
```

演示仍保留STOP文件、SIGINT/SIGTERM、摄像头新鲜度、继电器状态确认和
默认300秒总时限（可配置1–600秒）。完成、异常或取消都先停车并关闭全部继电器。
本模式不负责录制视频，供外部拍摄；本次只修改并部署代码，没有自动启动实车。
`tools/run_yellow_payload_demo.py`是同一路线入口的薄包装，自动选择演示选项；
也可直接给原入口 `tools/run_center_target_route.py` 加 `--demo-no-position-checks`。
