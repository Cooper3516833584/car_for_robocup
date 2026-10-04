# Differential robot configuration v2

Schema v2 lives beside the existing v1 competition profile and uses SI units.
The checked-in `configs/robocup_diffdrive.example.toml` contains temporary
software-test values only. Check its geometry and sensor mount coordinates
on the actual car before motion testing.

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
limits and disables autonomous navigation. The production `navigate_to*`
actions follow a direct path in the current fused pose frame; schema v2 no
longer loads a static competition map or a robot-footprint readiness flag.
The four former measurement-status gates have been removed. When the selected
protocol is `differential_vx_vz`, the mission is also blocked until the C10B
differential firmware is marked verified. Selecting
`ackermann_firmware_compat` keeps the compatibility backend explicit.

Older board-local profiles may still contain the retired measurement flags,
`[navigation.map]`, `[navigation.footprint]`, and `navigation.safety_margin_m`.
The v2 loader ignores those exact legacy fields so a Git pull does not require
editing a private board configuration; they have no navigation effect.

The existing v1 builders, including `build_ackermann_drive()`, remain active
for the old production entries. V2 builder placeholders report which later
migration step introduces each component.
