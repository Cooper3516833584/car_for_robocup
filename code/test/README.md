# 测试与硬件工具

默认单元测试覆盖差速运动、C10B 后端、D500/T265 定位、配置、watchdog、
通信、报警/继电器及任务板识别，不访问真实电机、相机或串口。

```bash
python3 -m compileall -q code tools
python3 -m unittest discover -s code/test -p 'test_*.py'
```

旧转向舵机、赛道视觉、task1/task2 及 v1 配置测试已随旧实现删除。
当前数量与删除清单见 `docs/LEGACY_CLEANUP.md`。

新诊断脚本放在仓库 `tools/`。现有共享工具包括：

- `wheel_test.py`：低层电机手动测试，需操作员监督。
- `d500_uart_probe.py`：D500 UART 只读帧统计。
- `hc14_*.py`：无线链路探测，修改无线参数仍需明确授权。
- `relay_selftest.py`：默认查询；显式输出触点动作见 `docs/RELAY_LCUS.md`。
- `tools/basic_motion.py`：现行差速动作测试入口。
- `tools/task_board_demo.py`、`tools/validate_task_boards.py`：离线/静态识别验收。

保留的 C10B、雷达、GPIO/PWM、通信及继电器测试保护共享硬件行为。
真实相机、OCR 模型和实车动作不放进普通 unittest discovery。