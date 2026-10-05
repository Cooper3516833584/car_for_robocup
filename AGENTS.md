# Repository guidance

## Project scope and active entry

This repository now contains a RoboCup differential ground robot path: two center driven wheels, passive front/rear supports, C10B motor controller, D500 lidar, and optional Intel RealSense T265. The active production entry is `code/main_robocup.py`; runtime composition and safe shutdown are in `code/robocup_runtime.py`.

The old Ackermann task, steering, navigation, v1 config, serial-screen launcher and ROS2 actuation stacks have been removed. Do not reintroduce them. Optional ROS2 sensor/map utilities remain; active differential motion is owned by `main_robocup.py`. Keep the C10B `ackermann_firmware_compat` protocol adapter until differential firmware is physically verified. See `docs/LEGACY_CLEANUP.md`.

## Development and board deployment

`/home/radxa/car` on the ROCK 5A is a deployment copy, not a development
workspace. Make every tracked code, configuration, test, script, and document
change in the local PC checkout first. Run the applicable hardware-free checks
there, commit the intended files, and push the correct GitHub branch. Only
then connect to the board, confirm its branch and HEAD, verify that its tracked
worktree is clean and can fast-forward, run `git pull --ff-only`, and verify the
new HEAD and clean worktree.

Do not edit, copy, or upload repository files directly on the board, and do not
commit, merge, rebase, cherry-pick, or create development branches there. If the
board has local changes, untracked files that conflict with the update, or a
diverged branch, stop deployment and preserve the data before reconciling it on
the PC and GitHub. Routine deployment must not use board-side stash, reset,
restore, clean, or forced pull. A separately authorized one-time cleanup must
first save and verify a recoverable snapshot of all affected board files.

Pulling code does not authorize a motor test or autonomous mission. Keep SSH
passwords out of scripts, command arguments, environment variables, logs, and
the repository.

## Architecture boundaries

- `code/config/` owns TOML parsing, readiness checks, and factories. Components receive validated config objects and do not read TOML themselves.
- `code/core/` owns canonical SI types, frame math, health, and monotonic-time contracts; it must not import hardware drivers.
- `code/components/differential_kinematics.py` is pure math. `DifferentialDrive` is the only differential motion-output facade and owns limiting, acceleration shaping, watchdog, and safe stop.
- Differential navigation emits `Twist2D`; it must not write serial frames or import steering/Ackermann components.
- Relative SLAM navigation uses a direct start-to-goal path in the current fused pose frame. Do not reintroduce static-grid A*, map/footprint path gates, or measured map/footprint readiness requirements without an explicit user request. This path does not avoid obstacles.
- T265 and D500 callbacks provide measurements only. They must never call the motor driver.
- Pose fusion stays independent of navigation and mission strategy.
- New hardware behavior belongs behind an adapter or backend; avoid rewriting validated D500 packet parsing/ICP or the C10B low-level frame sender without a reproduced defect and regression coverage.

## Coordinate and time contract

The canonical robot frame is `base_link` at the midpoint of the two driven wheel axes. `+X` points forward, `+Y` left, `+Z` up, and yaw is counter-clockwise about `+Z`. Core distances use metres, speeds m/s, angles radians, and timestamps monotonic seconds. Convert centimetres, millimetres, clockwise degrees, and device-specific frames in adapters before data reaches the new core.

## Calibration and hardware safety

- Treat values in `configs/robocup_diffdrive.example.toml` as software-test placeholders. Record and validate geometry and sensor extrinsics in `docs/HARDWARE_MEASUREMENTS.md` before real motion; the retired geometry/extrinsics `measured` flags are not readiness gates.
- `hardware-mission` must fail closed when its current readiness requirements are not met. Do not add a routine `--force` bypass. Documentation must clearly identify example geometry and mounts as unverified placeholders.
- `hardware-probe` is sensor-only and does not run autonomous navigation. Any motor smoke test must be separately operator-triggered, low-speed, supervised, and have an immediately accessible emergency stop.
- Startup failure, sensor loss during motion, backend error, SIGINT, and normal mission completion must leave the base stopped. Stop drive output before joining sensor workers or closing logs.
- Never raise speed limits to mask scale, frame, or localization errors. Diagnose in order: raw sensor -> adapter -> fusion -> navigation -> kinematics -> backend -> physical motion.

## C10B and serial devices

- Preserve the tested C10B frame encoder, periodic sender, timeout/watchdog, and stop behavior in `code/components/rear_motor.py`.
- On the current ROCK 5A test rig the C10B is `/dev/ttyACM0`, `115200 8N1`; confirm the actual device on other hardware. Legacy firmware accepts chassis `Vx/Vz`, with compiled protocol track `0.164 m` and normal turning constrained by a `0.350 m` minimum radius. Keep `ackermann_firmware_compat` unless a differential `v/omega` protocol is verified and recorded; only then select `differential_vx_vz`.
- D500 on the current ROCK 5A rig is read-only at `/dev/ttyS6`, `230400 8N1` after enabling UART6-M1. Preserve the 47-byte `54 2C` packet/CRC framing and full-scan assembly. Do not transmit to D500 or change its packet parser as part of unrelated navigation work.
- HC-14 transparent serial uses `115200 8N1`; clear DTR/RTS appropriately and keep AT commands plain ASCII without CR/LF. Do not send any AT command that changes baud, channel, air rate, power, or factory settings without explicit user authorization.
- The optional LCUS USB relay (`code/components/relay_lcus.py`, `[devices.relay]`, `docs/RELAY_LCUS.md`) drives real contacts at `9600 8N1` and has no hardware interlock: it is payload output, never a safety loop. Relay contacts stay latched after the port closes, so anything that opens the relay must request `all_off()` on every exit path; `RobocupRuntime.close()` owns that order. Never guess its port from VID/PID and keep it disabled unless a udev-pinned port is configured.
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

## Scripts and workspace layout

Paths in this file are relative to the repository root.

Scripts of all kinds -- including SSH helpers, hardware probes, diagnostics and
scratch automation -- belong in this repository's `tools/` directory. Keeping
them here is deliberate: hardware work on this project depends on a shared,
reviewable set of probes (for example `tools/d500_diag.py`,
`tools/d500_boundary_diag.py` and `tools/yaw_wall_crosscheck.py`), and a probe
that only exists on one workstation cannot be re-run or reviewed by anyone else.

Remove a script once it is genuinely obsolete. The test command above still runs
from the repository root.
