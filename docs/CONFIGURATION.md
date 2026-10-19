# 差速小车配置

现行入口是 `code/main_robocup.py`，配置只支持 schema v2。
旧 task1/task2 入口、v1 loader、舵机参数和旧车辆 profile 已删除。

`configs/robocup_diffdrive.toml` 自 2026-10-19 起纳入版本管理，保存本车实测值，
新板拉取后可直接使用；示例文件 `configs/robocup_diffdrive.example.toml` 仍是
占位模板。换板时按 `docs/BOARD_PORTING.md` 修改端口、引脚和标定值，
不要用示例值覆盖实车配置。

```powershell
Copy-Item configs\robocup_diffdrive.example.toml configs\robocup_diffdrive.toml
py -3 code\main_robocup.py --config configs\robocup_diffdrive.toml --mode dry-run
```

配置加载使用 `config.v2_loader.load_v2_config()`，CLI 用 `--config` 指定。
没有指定路径时加载 schema-v2 示例，硬件任务仍由 readiness 校验拒绝未标定参数。
详细字段、模式门禁和单位见 [CONFIG_SCHEMA_V2.md](CONFIG_SCHEMA_V2.md)；
现场测量见 [HARDWARE_MEASUREMENTS.md](HARDWARE_MEASUREMENTS.md)。

任务板相机和实测转向角由开局 CLI 参数传入，见
[TASK_BOARD_RECOGNITION.md](TASK_BOARD_RECOGNITION.md)。