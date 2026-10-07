# 中央目标停车测试

入口：`tools/run_center_target_route.py`。只使用生产 runtime 的融合定位、
motion 动作和 DifferentialDrive 输出，不调用比赛占位坐标或释放载荷。

路线从启动时的融合位置/朝向生成：前进 260 cm → 左转 90° → 前进
380 cm → 左转 90° → 前进 220 cm。位置与转角完成容差沿用本车配置。

摄像头舵机先转到仓库标定的 **+90°（2500 us）**，整个任务及退出后保持
PWM。这里的角度沿用 ServoAxis 的 ±90° 坐标，0° 是中位 1500 us。

YOLO 使用 `models/best_car.pt`（原有训练权重，red/blue/green/yellow 全部
参与，不再按颜色筛选）。置信度至少 0.5，目标框中心位于画面宽和高各自
25%～75% 时触发。视觉线程只提交结果，控制线程在下个周期先取消动作并
停止驱动，再响 1 s。随后恢复原直线终点或转向终点，不重走整段。
持续占用中央区域只响一次；连续三帧离开后重新进入可以再触发。

相机/推理错误、结果超过 2 s 未更新、定位安全停止、超时、SIGINT/SIGTERM
或 STOP 文件会结束测试并停车、关闭蜂鸣器。默认总时限 300 s。
首次加载模型必须在停车状态完成；不自动下载替代模型。

板端先启动新 SLAM 会话（参见 `SLAM_TOOLBOX_ROCK5A.md`），再使用同时具备
ROS、pyrealsense2、OpenCV、PyTorch、Ultralytics 的 Python 环境启动：

```bash
python tools/run_center_target_route.py --confirm-motor-test \
  --log-dir /home/radxa/car_test_logs/center-route-YYYYMMDD-HHMMSS
```

紧急停止可在 SSH 中执行 `touch <本次日志目录>/STOP`；运行输出、事件和
`result.txt` 可用于核对动作与报警。执行测试已由用户明确授权。
