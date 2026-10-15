# 小车单机全流程测试

临时自启动入口为 `code/car_full_flow_test.py`，由现有
`competition-boot.service` 启动。上电后只监听地面站，收到原有 RUN 指令后
才运行真实底盘任务；不会上电直接行驶。

完整顺序：RUN 确认 → 20 秒倒计时（最后 5 秒蜂鸣）→ 重启并确认小车 SLAM 新实例
→ 相机 +90° → 起点 OCR（沿用三次失败后 1/2/1 兜底）→ 等待融合定位
→ 分配小车任务颜色 → 车道入口 → 第一转角 → 横向车道
→ 搜索黄色目标并在道路上精对齐 → 左转 90° → 融合直线接近 43 cm、停车复测和修正
→ 停车保留原释放等待时间，但不驱动抛投 → 直线倒回并右转恢复道路朝向
→ 对任务板分配颜色再执行同样完整搜索、对齐、接近及返回
→ 第二转角 → 终点停车。

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

路线坐标仍在 `code/competition_task.py` 顶部；当前仓库中的路线点是未实测的零值
占位，必须填入现场融合坐标才能验收真实整场路线，测试入口不会推测场地坐标。
`DROP_SAFE_M=0.43 m` 沿用既有场地调试参数。该程序会真实驱动底盘。

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

恢复正式空地协同：在 PC 将 `deploy/competition-boot.service` 的 ExecStart 入口改回
`code/competition_boot.py`，提交推送、板端快进更新后重新安装并重启监听服务。
正式入口默认继续执行无人机协同和真实抛投策略。
