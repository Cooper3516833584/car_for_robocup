# 小车单机全流程测试

临时自启动入口为 `code/car_full_flow_test.py`，由现有
`competition-boot.service` 启动。上电后只监听地面站，收到原有 RUN 指令后
才运行真实底盘任务；不会上电直接行驶。

完整顺序：RUN 确认 → 20 秒倒计时（最后 5 秒蜂鸣）→ 重启并确认小车 SLAM 新实例
→ 相机 +90° → 起点 OCR（沿用三次失败后 1/2/1 兜底）→ 等待融合定位
→ 分配小车任务颜色 → 三边固定道路（0°、90°、180°），沿三边巡检
→ 任意先后识别黄色及分配色 → 各自道路精对齐、左转 90°、融合接近 43 cm、停车复测
→ 保留释放等待但不驱动物资舵机 → 原轴直倒、右转恢复道路朝向、继续原线段
→ 第三段末端停车。没有重新回横向起点搜索第二种颜色。

复用比赛程序及同一组路线参数，没有复制另一套任务逻辑。即使两个抛投 PWM
均未配置，测试模式仍执行完整接近、融合距离验证和返回。日志中的
`payload_release_skipped` 明确标记模拟释放；`payload_drop` 的 `simulated=true`、
`success=false`、`detour_completed` 区分未实际释放与接近返回是否完成。
目标缺失或不能可靠对齐时沿用正式程序的跳过该目标策略，不会伪造目标；
要验收两段完整往返，现场必须提供黄色与任务分配颜色目标。

测试入口不创建无人机任务发送器，不发送无人机关闭、重建图、任务、启动或 STOP
指令，也不等待无人机回应。HC-15 继续接收地面站 RUN/STOP/PING 并回复 ACK。
小车抛投舵机在启动、任务、STOP、异常和退出时均无输出；相机舵机正常工作。
底盘停车、定位丢失、任务超时、STOP 文件及信号清理沿用现有机制。

路线唯一来源为 `code/competition_task.py` 的 `ROUTE_POINTS`：
`((0.00,0.00), (7.60,0.00), (7.60,7.36), (0.00,7.36))`，
对应启动 D500 雷达中心的局部坐标；三段航向为 `(0°,90°,180°)`。
测试入口没有独立坐标副本。OCR 后一次性记录融合 base_link 起点，
读取既有 D500 XY 安装偏移，把局部雷达目标转换为融合底盘目标；途中不刷新原点。
转向以底盘轴中心为轴，雷达可能绕弧，现场须人工核对雷达 offset 和车轮转弯空间。
`DROP_SAFE_M=0.43 m` 不变。该入口是真实运动测试，软件仿真不代表实车精度验收。
本次路线修改仅本地测试和提交；以下是之后人工确认部署时的操作参考，
本次不执行推送/部署设备、安装或 service 重启。

部署须先在 PC 提交并推送，再确认板端分支、HEAD、工作区干净且能够快进，运行
`git pull --ff-only`。然后在板端仓库运行：

```sh
sudo sh deploy/install_car_services.sh
sudo systemctl restart competition-boot.service
systemctl show competition-boot.service -p ExecStart -p ActiveState
```

安装器更新 service 内容，保留原有启用状态。若原来未启用，执行
`sudo systemctl enable competition-boot.service`。重启监听服务不会发出 RUN。
不要在任务执行中重启服务。查看日志用
`journalctl -u competition-boot.service -f`。

本车使用 Debian 12 的 RoboStack 环境，不存在 `/opt/ros/humble/setup.bash`。
SLAM 服务通过 `robocup_ros/bin/micromamba run -p robocup_ros/env` 启动。
任务监听同样激活该 ROS 环境，并使用带系统包的 `vision_env/bin/python`，
通过 `PYTHONPATH=robocup_ros/python_ext:car/code` 加载既有 T265 扩展。
安装器会实际导入 ROS、T265、OCR 和 YOLO 依赖，缺少任何一项都拒绝安装，
避免仅能接收 RUN 却在建图或视觉启动时才失败。

恢复正式空地协同：在 PC 将 `deploy/competition-boot.service` 的 ExecStart 入口改回
`code/competition_boot.py`，提交推送、板端快进更新后重新安装并重启监听服务。
正式入口默认继续执行无人机协同和真实抛投策略。
