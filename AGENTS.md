# Repository guidance

## Project scope and active entry

This repository now contains a RoboCup differential ground robot path: two center driven wheels, passive front/rear supports, C10B motor controller, D500 lidar, and optional Intel RealSense T265. The active production entry is `code/main_robocup.py`; runtime composition and safe shutdown are in `code/robocup_runtime.py`.

Old Ackermann task entries and modules remain available for the older vehicle and are legacy-only. Do not add them to the RoboCup dependency path. Established module locations stay in place because old launchers, ROS bridge code, and regression tests import them. See `code/legacy/README.md` and `docs/legacy/`.

## Architecture boundaries

- `code/config/` owns TOML parsing, readiness checks, and factories. Components receive validated config objects and do not read TOML themselves.
- `code/core/` owns canonical SI types, frame math, health, and monotonic-time contracts; it must not import hardware drivers.
- `code/components/differential_kinematics.py` is pure math. `DifferentialDrive` is the only differential motion-output facade and owns limiting, acceleration shaping, watchdog, and safe stop.
- Differential navigation emits `Twist2D`; it must not write serial frames or import steering/Ackermann components.
- T265 and D500 callbacks provide measurements only. They must never call the motor driver.
- Pose fusion stays independent of navigation and mission strategy.
- New hardware behavior belongs behind an adapter or backend; avoid rewriting validated D500 packet parsing/ICP or the C10B low-level frame sender without a reproduced defect and regression coverage.

## Coordinate and time contract

The canonical robot frame is `base_link` at the midpoint of the two driven wheel axes. `+X` points forward, `+Y` left, `+Z` up, and yaw is counter-clockwise about `+Z`. Core distances use metres, speeds m/s, angles radians, and timestamps monotonic seconds. Convert centimetres, millimetres, clockwise degrees, and device-specific frames in adapters before data reaches the new core.

## Calibration and hardware safety

- Treat values in `configs/robocup_diffdrive.example.toml` as software-test placeholders. Keep geometry/extrinsics flags false until measurements and validation are recorded in `docs/HARDWARE_MEASUREMENTS.md`.
- `hardware-mission` must fail closed when readiness requirements are not met. Do not add a routine `--force` bypass. The CLI should clearly warn when the example measurements remain unverified.
- `hardware-probe` is sensor-only and does not run autonomous navigation. Any motor smoke test must be separately operator-triggered, low-speed, supervised, and have an immediately accessible emergency stop.
- Startup failure, sensor loss during motion, backend error, SIGINT, and normal mission completion must leave the base stopped. Stop drive output before joining sensor workers or closing logs.
- Never raise speed limits to mask scale, frame, or localization errors. Diagnose in order: raw sensor -> adapter -> fusion -> navigation -> kinematics -> backend -> physical motion.

## C10B and serial devices

- Preserve the tested C10B frame encoder, periodic sender, timeout/watchdog, and stop behavior in `code/components/rear_motor.py`.
- On the current ROCK 5A test rig the C10B is `/dev/ttyACM0`, `115200 8N1`; confirm the actual device on other hardware. Legacy firmware accepts chassis `Vx/Vz`, with compiled protocol track `0.164 m` and normal turning constrained by a `0.350 m` minimum radius. Keep `ackermann_firmware_compat` unless a differential `v/omega` protocol is verified and recorded; only then select `differential_vx_vz`.
- D500 on the current ROCK 5A rig is read-only at `/dev/ttyS6`, `230400 8N1` after enabling UART6-M1. Preserve the 47-byte `54 2C` packet/CRC framing and full-scan assembly. Do not transmit to D500 or change its packet parser as part of unrelated navigation work.
- HC-14 transparent serial uses `115200 8N1`; clear DTR/RTS appropriately and keep AT commands plain ASCII without CR/LF. Do not send any AT command that changes baud, channel, air rate, power, or factory settings without explicit user authorization.
- Never assume a serial device path or firmware capability from another vehicle. Fail closed on unknown protocol modes and keep all hardware opening behind runtime start.

## T265

`pyrealsense2` remains an optional, lazy import. Dry-run, replay, and unit tests must work without it. Transform raw T265 pose through the measured mount and native-axis conversion before projecting to planar `base_link`; enforce confidence and freshness limits. Stale or low-confidence measurements degrade or stop localization rather than terminating the control loop.

## Tests and changes

Use the project test command from the repository root:

```powershell
py -3 -m compileall -q code tools
py -3 -m unittest discover -s code/test -p "test_*.py"
git diff --check
```

Rerun relevant kinematics, backend, frame adapter, fusion, navigation, config, and watchdog tests after changing those layers. Keep real hardware tests out of default unit discovery. One migration step per commit; do not run destructive reset/clean commands, rewrite remotes, or push without direct instruction.
