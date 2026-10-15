# 小车单机全流程测试

自启动服务使用 `code/car_full_flow_test.py --listen`，由 `competition-boot.service` 运行。
它使用真实底盘、视觉和两只 SG90，仅屏蔽无人机命令；上电先 OPEN 两只并监听地面站。
收到地面站有效 RUN 或本机启动请求才 CLOSE 两只并开始任务，上电不会自行行驶。

## VS Code Remote SSH 无参数启动

SSH 连接小车（2026-10-15 当前地址 `radxa@10.202.217.163`），打开
`/home/radxa/car/code/car_full_flow_test.py`，直接点击 VS Code 的“运行 Python 文件”按钮，
不填任何参数，即启动同一套完整流程。该启动入口只需 Python 标准库；实际任务仍由
自启动服务在原 micromamba + vision_env 环境执行，不依赖 VS Code 选中的视觉/ROS 环境。
终端跟随任务日志，Ctrl+C 会请求 STOP。程序显示“任务已结束”只代表本轮工作线程退出，
是否成功完成须看任务日志；完赛、停止或失败后均继续监听，可再次点击运行。

本机启动通过权限为 0600 的 `/run/robocup-car/control.sock` 联系已运行的监听服务，
与地面站 RUN 复用同一任务入口、忙碌检查、20 秒倒计时和收尾。
它不另开串口或第二个任务实例，不依赖无线链路成功；地面站 STOP 仍然有效。
重复运行会报告 BUSY，不中断正在执行的任务。
若服务未运行或未升级，入口报连接错误，不另起一套硬件程序。

命令行等价用法：

```sh
python3 /home/radxa/car/code/car_full_flow_test.py
python3 /home/radxa/car/code/car_full_flow_test.py --stop
python3 /home/radxa/car/code/car_full_flow_test.py --status
```

`--status` 只读、不启动；`--listen` 专供自启动服务，手工运行它不会启动任务。

## 任务流程
正式和单机监听都通过 `--config` 明确传入当前仓库的
`configs/robocup_diffdrive.toml` 绝对路径，启动日志打印该路径。
该文件缺失时任务报错退出，不回退到示例配置；示例配置的相机舵机默认禁用。

完整顺序：RUN → 两只 CLOSE（1500 μs）→ 原有 20 秒倒计时（最后 5 秒蜂鸣）
→ 重启并确认 SLAM 新实例 → 相机 +90° → 起点 OCR（三次失败用 1/2/1）
→ 等待融合定位 → 分配小车颜色 → 三边固定道路 → 任意先后识别黄色和分配色
→ 道路精对齐、左转 90°、融合接近 43 cm、停车复测 → 只 OPEN 对应槽位（1000 μs）
→ 保持 0.5 秒、原轴倒退、右转恢复、继续原道路 → 第三段末端停车 → 两只 OPEN，继续监听。
STOP、定位丢失、超时、异常、信号退出均停车并 OPEN 两只，不会完赛后自动 CLOSE。

单机入口不创建无人机发送器，不发送 CLOSE/REMAP/TASK/START/STOP，不等待无人机 ACK。
HC-15 继续接收地面站 RUN/STOP/PING 并回复 ACK。
`car_only_test` 与模拟物资无关；只在离线测试显式传入内部模拟参数时跳过物资硬件，
同时保留完整接近、等待和返回。模拟日志 success=false、simulated=true；真实 PWM
失败明确报错并停车，不伪造成功。写入成功也不代表存在物资落下的机械反馈。

路线唯一来源为 `competition_task.ROUTE_POINTS`：
`((0,0), (7.60,0), (7.60,7.36), (0,7.36))`，三段航向 `(0°,90°,180°)`。
沿用启动 D500 中心局部坐标和一次性融合原点、现有 D500 安装偏移；不复制另一套路线。
目标缺失或不能稳定对齐时沿用跳过本次目标策略；PWM 不可用时中止真实投放任务。
实际定位精度、外参、转弯空间、舵机脉宽与机构动作仍需现场验收。

两只物资舵机接线、0777 权限预启动、overlay 和独立测试见 [PAYLOAD_SG90.md](PAYLOAD_SG90.md)。
部署须 PC 提交推送后板端干净快进，安装更新 service 模板；重启监听立即操作舵机，
由操作者确认现场机构可操作后执行。当前入口不改回正式比赛入口。

比赛与监听使用已安装 RoboStack/micromamba 与 vision_env Python，保留原 ROS、T265、
OCR、YOLO 导入检查。恢复空地协同时由操作者在 PC 修改 service 入口为
`code/competition_boot.py`，提交推送、板端快进后重新安装服务。
