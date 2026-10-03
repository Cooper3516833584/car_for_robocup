# 差速小车基础运动单动作验收

> 本文记录 2026-10-02 的后置 T265、单传感器闭环方案，以下安装位置和验收方式已过时。当前左置 T265、主任务融合定位的两级实验使用 `DRIVE_CALIBRATION_20261003.md`：开环 50 cm／90° 暂按 5 cm／6° 筛查，最终闭环仍按 3 cm／3° 验收。

本阶段只检查前后运动、10–1000 cm 定距和相对原地旋转；不启动比赛任务，也不使用 D500 全局定位。`tools/basic_motion.py` 是需操作者现场确认的电机测试工具，不是自动避障程序。先核对实际 C10B 端口、限速、物理急停和测试区域。未经实测，不要把示例配置的占位外参当成车上参数。

## 测量和测试顺序

1. 以左右主动轮轴线中点为 `base_link`，测量 `base_link -> t265_link` 的 x（前为正）、y（左为正）、z（上为正）及 roll/pitch/yaw（弧度）。用户已确认雷达中心投影与该点重合；T265 在正后方、鱼眼朝后，雷达到外壳背面和镜头面分别约为 24.8、26.0 cm。Intel 手册中的位姿跟踪原点在两只鱼眼成像器之间，距外壳背面约 5.95 mm，模块中心相对外壳横向中心偏 9.10 mm；因此 x 的初值约为 −0.254 m，不能把外壳中心或 y=0 直接当成已验证的跟踪原点。若外壳正装且居中，y 初值约为 −0.0091 m，yaw 初值为 π；z 仍须在车上测量，roll/pitch 暂按 0 并用手动转车检查。将测量、安装方向、方法和复核记录填入本机配置及 `HARDWARE_MEASUREMENTS.md`。本阶段不使用 D500，也不据此将它的外参标记为已测量。参见 [T265 数据手册图 4-2](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/IntelRealSenseTrackingT265Datasheet.pdf) 和 [Intel 对跟踪原点深度的确认](https://community.intel.com/t5/Items-with-no-label/Intel-T265-center-of-tracking/m-p/661987)。
2. 电机不启动时，使用有界 T265 探针观察原始相机与外参补偿后的 `base_link`；绕地面标出的轴线中点手动转车，若中心轨迹明显画弧或 T265 置信度不足，先修正外参或环境。T265 每次启动建立自己的相对世界原点，不作为比赛地图原点。
3. 架空车轮，在急停可用时分别执行低速 `jog forward/backward/left/right`，核对左右轮方向、固件响应和停车。若原地旋转被固件拒绝，停止并记录，不切换未验证的协议模式。
4. 在空旷区域、T265 外参已测并核对后，先执行正反向 10、20 cm，再执行正反向 50 cm 定距及正负不同角度旋转。每条命令都须等待现场人员确认就绪；动作结束后用地面标记和卷尺测主动轮轴线中点，用角度模板测车头朝向。T265 输出用于控制和诊断，验收误差以外部实测为准。
5. CSV 中 `base_x_cm/base_y_cm` 是补偿后的车体中心位置；`raw_camera_dx_cm/raw_camera_dy_cm` 是相机原始位置相对首个记录样本的位移。旋转时比较两者的轨迹，确认相机偏心圆弧没有被误当成车体移动。日志中的 `stop`、`settle` 样本用于分析停车后滑行。

示例命令（仅在准备好的车上执行）：

```sh
python3 tools/t265_probe.py --config configs/robocup_diffdrive.toml --seconds 20
python3 tools/basic_motion.py --config configs/robocup_diffdrive.toml --confirm-motor-test jog --direction forward --seconds 1
python3 tools/basic_motion.py --config configs/robocup_diffdrive.toml --confirm-motor-test jog --direction left --seconds 1
python3 tools/basic_motion.py --config configs/robocup_diffdrive.toml --confirm-motor-test --confirm-t265-mount-measured --trace /tmp/forward_50.csv distance --cm 50
python3 tools/basic_motion.py --config configs/robocup_diffdrive.toml --confirm-motor-test --confirm-t265-mount-measured --trace /tmp/turn_37.csv rotate --deg 37
```

`distance --cm` 的正值是前进、负值是后退，绝对值必须在 10–1000 cm。`rotate --deg` 是**相对于动作开始朝向**的角度，正值逆时针、负值顺时针，范围为 −360° 至 +360°。输入 0° 时不驱动电机。默认 CSV 写入临时目录；需长期保存时显式传 `--trace`。

SSH 控制时每次只发送一条有界动作；连接中断产生 SIGHUP 时工具进入停车路径。现场仍须有人能立即使用物理急停。T265 的距离单位由其位姿直接给出，不用轮径换算；轮径另行记录用于底盘标定。

| 动作 | 目标 | T265 停稳读数 | 外部实测 | 中心位移 | 日期/操作者/日志 |
|---|---:|---:|---:|---:|---|
| 前进 | 50 cm | — | — | — | — |
| 后退 | −50 cm | — | — | — | — |
| 左转 | +37°、+90°、+180°、+270°、+360° | — | — | — | — |
| 右转 | −37°、−90°、−180°、−270°、−360° | — | — | — | — |

50 cm 定距的外部实测误差目标为不超过 5 cm；转角误差目标为不超过 3°，旋转时主动轮轴线中点位移不超过 5 cm。10–1000 cm 输入范围和 1000 cm 超时路径由无硬件测试覆盖。当前没有合适的长距离场地，超过短距样本的实车精度暂不验收；上表空白处不能作为已通过的记录。
