# T265 + D500 + SLAM 人工精度测试

本测试只读取 T265 和 D500。操作者手动移动整车；程序使用假电机后端，
不打开 C10B 和继电器端口，不启动导航任务。生产配置文件只读：测试程序在
内存中启用 D500、`slam_toolbox` 相对定位，并关闭尚未测量的场地墙锚点。
测试通过也不会设置 `hardware_mission_validated`。
测试模式直接将 D500 完整扫描送入 SLAM，跳过不参与该融合链的 D500
扫描间 ICP；原始串口帧仍由现有 CRC 解析器校验。

## 地面真值

在地面标记主动轮轴线中点 P0，以初始车头方向为 +X、左侧为 +Y，
逆时针角度为正。每次停止后，用卷尺测量轴心相对 P0 的 **X、Y 坐标**，
用地面方向标记测量车头角度。输入实际读数，不输入计划移动的 50 cm。
车轮全程着地，左右横移时保持车体水平、朝向尽量不变。旋转时也记录
轴心真实移动，绕一整圈时连续角度写 `360` 或 `-360`，不能写 `0`。

按前→后、左→右、左转 90°→右转复位各重复三次，再做正反 360° 和
约 50 cm 边长的人工矩形闭合。每次移动前等待程序确认 `BEGIN`；停止后
保持静止至少两秒，再通知控制程序的人标记 `end`，随后报告独立实测值。

## 板端启动与交互

先只读检查没有其他 T265、D500 或小车任务进程占用设备，确认板端工作树
干净。两个 SSH 终端把 `RUN_ID` 设成同一个从未使用过的名称，例如
`slam_accuracy_20261002_01`，再使用同一 ROS 环境：

```bash
RUN_ID=slam_accuracy_20261002_01
mkdir -p "/home/radxa/car_test_logs/$RUN_ID/ros"
export MAMBA_ROOT_PREFIX=/home/radxa/robocup_ros/root
export PYTHONPATH=/home/radxa/robocup_ros/python_ext:/home/radxa/car/code
export ROS_LOG_DIR="/home/radxa/car_test_logs/$RUN_ID/ros"
```

第一个终端启动 SLAM 节点：

```bash
/home/radxa/robocup_ros/bin/micromamba run -p /home/radxa/robocup_ros/env \
  ros2 launch /home/radxa/car/launch/robocup_slam.launch.py
```

第二个终端启动交互探针。每次使用新 `RUN_ID`；程序拒绝覆盖已有目录：

```bash
/home/radxa/robocup_ros/bin/micromamba run -p /home/radxa/robocup_ros/env \
  python -B /home/radxa/car/tools/slam_manual_accuracy.py \
  --config /home/radxa/car/configs/robocup_diffdrive.toml \
  --output "/home/radxa/car_test_logs/$RUN_ID/accuracy"
```

先手动小幅移动预热 T265、返回 P0，保持静止十秒。控制者输入 `status` 和
`preflight`；只有预检查通过后才输入 `begin forward_1`。每段终点静止两秒
后输入 `end`，程序此时不显示估计位移，避免影响人工测量。操作者报告地面
绝对坐标后，控制者输入 `truth X_CM Y_CM YAW_DEG`，例如前进后实测
`truth 49.8 0.4 -0.5`。程序再显示测量值、估计值和误差。依次使用
`begin`／`end`／`truth`；结束时输入 `quit`。SSH 中断时进程退出并关闭
传感器；若看到 `REJECTED` 或 `PREFLIGHT FAIL`，先排查后再继续。
原地转向段使用 `begin left_90_1 pivot` 等命令，并照常实测轴心 X、Y；
程序将真实轴心漂移纳入二维位置误差。
矩形测试前，车回到起点并静止两秒，输入 `loop-start rectangle`；四条边
各完成一次 `begin`／`end`／`truth` 后，输入 `closure rectangle` 计算只针对
这圈矩形的闭合误差。

## 判定与日志

程序对起终点各取约两秒静止窗口的中位位姿，计算起点车体坐标系中的
`dx、dy、dyaw`。单段二维位置误差 `hypot(dx误差, dy误差) ≤ 3 cm`，
角度误差 `≤ 3°` 才算通过。低 T265 置信度、SLAM 锚点过期、融合位姿
丢失、扫描率过低、TF 时间误差过大或超过 30 cm 的瞬时跳变会把该段标为
`INVALID`，不得当作通过。预检查要求约 4–5 Hz 扫描、TF 配准 p95
低于 30 ms、T265 置信度 3、十秒静置抖动 p95 不超过 1 cm／1°。

`events.jsonl` 保存原始运行事件和逐帧观测，`segments.csv` 保存各段的
真值、估计、误差和有效性，`summary.json` 保存计数和日志错误。原地旋转
另检查轴心漂移；矩形回 P0 的闭合结果按实施方案第一阶段目标另行报告：
位置不超过 15 cm、角度不超过 5°。如果单段超差，依次核对 T265 原始
值、安装外参、D500 扫描与 TF 时间、SLAM 锚点、融合输出；修正后重测
同一动作，不调宽通过阈值。
