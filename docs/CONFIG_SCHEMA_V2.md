# Differential robot configuration v2

Schema v2 lives beside the existing v1 competition profile and uses SI units.
The checked-in `configs/robocup_diffdrive.example.toml` contains temporary
software-test values only. Its geometry and sensor mount coordinates must
remain marked unmeasured until checked on the actual car.

Create a private working copy in PowerShell:

```powershell
Copy-Item configs\robocup_diffdrive.example.toml configs\robocup_diffdrive.toml
```

That local file is ignored by Git. Load schema v2 through
`config.v2_loader.load_v2_config()`; schema-v1 profiles continue through
`config.loader.load_car_config()`. Passing a v1 file to the v2 loader gives a
clear migration error rather than silently reinterpreting its fields.

`validate_runtime_readiness(config, RuntimeMode.DRY_RUN)` and `REPLAY` allow
placeholder geometry. `HARDWARE_PROBE` also allows it, but
`runtime_constraints()` caps linear/angular speeds using the configured probe
limits and disables autonomous navigation. `HARDWARE_MISSION` blocks when
required geometry or sensor extrinsics are not measured. When the selected
protocol is `differential_vx_vz`, the mission is also blocked until the C10B
differential firmware is marked verified. Selecting
`ackermann_firmware_compat` keeps the compatibility backend explicit.

The existing v1 builders, including `build_ackermann_drive()`, remain active
for the old production entries. V2 builder placeholders report which later
migration step introduces each component.
