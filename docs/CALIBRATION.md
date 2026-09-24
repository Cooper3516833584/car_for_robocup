# RoboCup 差速底盘标定手册

本流程对应 schema-v2 差速配置 `configs/robocup_diffdrive.toml`。测量数据记录在
[HARDWARE_MEASUREMENTS.md](HARDWARE_MEASUREMENTS.md)。目前没有实车测量结果；请将示例值视为软件测试占位值，不能据此解锁正式任务。

## RoboCup 差速标定顺序

每次只标定一类数据，并记录日期、操作者、测量工具、原始数据和配置提交。坐标约定为 `base_link` 位于左右主动轮轴线中点，`+X` 朝前、`+Y` 朝左、`+Z` 朝上，yaw 逆时针为正。

1. **准备配置和安全措施。** 复制 `configs/robocup_diffdrive.example.toml` 到本机忽略文件 `configs/robocup_diffdrive.toml`。确认 `geometry_measured`、`sensor_extrinsics_measured`、`c10b_diff_firmware_verified` 均为 `false`；架空车轮或选空旷测试区，确认急停可用，限速保持最低。
2. **测量机械参数。** 测左右主动轮接触中心线距离、左右轮有效滚动直径、车身四边相对 `base_link` 的 footprint，以及前后支撑轮坐标。轮距做至少三次并记录平均值。按实测结果更新 v2 geometry。
3. **记录传感器外参。** 分别填写 `base_link -> d500_link` 和 `base_link -> t265_link` 的 x/y/z/roll/pitch/yaw。不要因传感器看起来位于车体中心就把偏置写成零。
4. **验证 T265 外参。** 固定车体位置缓慢原地转动；raw camera center 可走圆，adapter 输出的 `base_link` 位置应基本固定。若仍呈圆周，检查外参方向和变换顺序。
5. **验证 D500 外参。** 在已稳定建图的区域原地转 360°；adapter 输出的 base pose 不应随雷达偏置明显绕圈。先修正外参，再评估 ICP/墙面约束。
6. **检查轮速符号。** 架空车轮低速验证 `(v>0, ω=0)` 两轮前进，`(v=0, ω>0)` 左轮后转、右轮前转。符号错误应在 backend/电机配置层修正，不改运动学坐标约定。
7. **验证 C10B 固件能力。** 按 HARDWARE_MEASUREMENTS 表格逐项检查停车、前进/后退、原地旋转、多个半径转弯和左右镜像。只有验证差速 `v/ω` 命令后才设置 `c10b_diff_firmware_verified=true` 并使用 `differential_vx_vz`；否则使用 compatibility 模式。
8. **标定有效轮距与速度尺度。** 使用多个角速度/方向拟合有效轮距；直行 1–2 m，在左右方向和多个速度档对比命令值与传感器实测值。不要用导航控制增益掩盖驱动尺度误差。
9. **先分开检查定位，再启用融合。** 分别画 T265 base 轨迹、D500 map 轨迹，确认单位、方向、外参正确后才调整 fusion gain 和创新门限。保存带时间戳的 JSONL 日志，用 `tools/replay_pose_log.py` 比较参数变更。
10. **最后调导航。** 从很低速度开始，依序调整原地旋转增益、最终 yaw 容差、路径航向、lookahead、减速距离、加速度限制；确认停车和失定位策略后再逐步提速。
11. **更新解锁标记。** 只有实际完成并复核测量后，才将对应 calibration flag 改为 `true`。提交配置变更时同时填写测量记录和 Git commit。运行时会在标记缺失时警告，并由 readiness gate 拒绝未测 geometry/extrinsics 的 `hardware-mission`。

所有步骤按顺序执行；若定位丢失、轮速方向不符、固件拒绝命令或出现不可解释的漂移，立即发出零速/急停并返回前一项排查。

---

## Legacy Ackermann 标定参考

以下内容只适用于旧 Ackermann 车辆、v1 配置和旧任务入口，不适用于 RoboCup 差速底盘。

旧任务入口为 `code/main_task1.py` / `code/main_task2.py`，旧配置使用 `configs/car.example.toml`。

---

## Legacy Ackermann vehicle calibration (reference)

按以下顺序操作。每一项完成后把测量结果填进配置文件（复制
`configs/car.example.toml` 得到自己学校的配置）。**不要**修改主程序。

> 单位注意：TOML 里长度用 mm、速度用 cm/s、角度用 rad（舵机）/ deg（雷达安装）。

## 0. 准备工作

```bash
cp configs/car.example.toml configs/my_car.toml
export CAR_CONFIG=configs/my_car.toml
```

## 1. 测轴距（wheelbase_mm）

把车放平，用卷尺/卡尺量**前后轴中心**的距离（不是车身长度）。

```toml
[vehicle.geometry]
wheelbase_mm = 142.5
```

## 2. 测左右后轮中心距（physical_track_width_mm）

量左右后轮**中心**的距离。通常先量车轮厚度和左右外侧总宽再相减：

```toml
wheel_thickness_mm = 26.4
outer_wheel_width_mm = 143.5
# physical_track_width_mm = outer_wheel_width_mm - wheel_thickness_mm = 117.1
physical_track_width_mm = 117.1
```

## 3. 测车身长宽（body_length_mm / body_width_mm）

碰撞检测用矩形车身尺寸：

```toml
body_length_mm = 230.0
body_width_mm = 145.0
```

## 4. 测后轴到车身中心（rear_axle_to_body_center_mm）

导航位姿原点在后轴中心。车身前后悬对称时 = `wheelbase / 2`（当前车
`71.25 mm`）。若前后悬不对称，实测后轴中心到车身几何中心的纵向距离。

## 5. 舵机回中（center_us）

把舵机接到目标 PWM 输出，**断电状态下**手动把前轮摆到目视正前方，记录
当前 PWM 脉宽；或先用厂家默认（1500 us）上电，再微调脉宽让前轮完全居中。
当前车实测 `1580 us`。

```toml
[vehicle.steering]
center_us = 1580
```

## 6. 确认转向方向（direction_sign）

- 正角度（逻辑左转）输出脉宽应 **> center_us**，且前轮实际向左偏；
- 负角度（逻辑右转）输出脉宽应 **< center_us**，且前轮实际向右偏。

若方向相反，`direction_sign` 取反（当前车实测 `-1.0`）。

> ⚠️ 标定期间小车必须架起，后轮悬空，防止意外移动。

## 7. 标定左右机械极限（logical_right_max_rad / logical_left_max_rad）

缓慢增大转角直到舵机/连杆到达机械限位前，记录能稳定到达的最大正负角度。

```toml
logical_right_max_rad = -0.32
logical_left_max_rad = 0.49
```

（`calibration_min_rad` / `calibration_max_rad` 是厂家曲线的角度范围，
一般沿用厂家数据 `-0.49 / +0.32`。）

## 8. 标定角度→PWM（曲线系数）

若沿用 WHEELTEC L150 厂家三次曲线，填写：

```toml
curve_a3 = -0.628
curve_a2 = 1.269
curve_a1 = -1.772
curve_a0 = 1.573
curve_scale = 640.62
factory_center_us = 1501
```

换非 L150 舵机/转向机构时，需要重新拟合角度→脉宽映射，并把系数填到这里。

**回归检查**（当前车）：`-0.12 rad → 1454 us`，`0 → 1580 us`，
`+0.12 rad → 1728 us`。可用 `python3 -m pytest code/test/test_steering_config.py`
验证。

## 9. 测雷达相对后轴位置（sensors.radar.mount）

以**后轴中心**为原点：

```toml
[sensors.radar.mount]
x_forward_cm = 0.0   # 雷达测量原点在后轴前方 cm（在后方填负数）
y_left_cm = 0.0      # 在后轴左侧 cm（右侧填负数）
yaw_cw_deg = 0.0     # 雷达零度相对车头顺时针偏角
```

## 10. 标定摄像头透视（sensors.camera.perspective）

把车停在场地上，前放一张 A4 纸或标记物：

1. 在俯视图像上标出近端左右、远端左右四个角点；
2. 记录四点在**归一化坐标**（除以宽高）中的位置，按 左下、右下、右上、左上
   顺序填 `source_points_norm`；
3. 量出梯形对应的地面宽/深（cm）填 `ground_width_cm` / `ground_depth_cm`。

同时按当前场地黑线实测填写 `[sensors.camera.line]`（线宽窗口、扫描距离、
形态学尺寸等）。

## 11. 低速测试

```bash
python3 code/main_task1.py --config configs/my_car.toml --no-camera-correction \
  --ab-speed-cm-s 4 --bc-speed-cm-s 4 --cd-speed-cm-s 4 --da-speed-cm-s 4
```

- 先确认 C10B 电机已使能（串口遥测第 2 字节为 `00`）；
- 确认 D500 建图成功、车能沿黑线低速走完一圈；
- 确认声光报警 on/off 正常（`active_low` 是否正确）。

## 12. 比赛参数调优

把 `[missions.task1]` / `[missions.task2]` 的速度调到比赛值，再按
`[missions.control]` 逐项调相机纠偏增益/死区/各段进度窗口。每次只改一个
参数，记录效果。
