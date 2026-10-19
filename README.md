# RoboCup Differential Ground Robot

This repository contains the RoboCup ground vehicle software for a differential-drive chassis with two driven center wheels and passive front/rear supports. The production stack uses C10B motor control, D500 lidar localization, optional Intel RealSense T265 odometry, canonical SI poses, and a separate mission runtime.

## Project at a glance

| Item | Value |
|---|---|
| Chassis | Differential drive, two driven centre wheels, passive front/rear supports |
| Motor controller | C10B over serial (`/dev/ttyACM0`, 115200 8N1 on the current rig) |
| Localization | D500 LiDAR (SLAM/ICP), optional Intel RealSense T265 odometry |
| Compute | ROCK 5A-class host running the Python runtime |
| Radio | HC-15 / HC-14 link for RUN / STOP / PING |
| Payload | Five SG90 drop servos (see `docs/PAYLOAD_SG90.md`) |

## Architecture

- [docs/SYSTEM_ARCHITECTURE.md](docs/SYSTEM_ARCHITECTURE.md) — diagrams plus a
  component-to-source evidence table.
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — layering rationale for
  `DifferentialNavigation → Twist2D → DifferentialDrive → C10B`.

The current runtime is a **directed-line Pure Pursuit** kernel; the relative-SLAM
path does not plan around obstacles. Example geometry and sensor mounts are
placeholders until measured.

## Validation status

- `dry-run` is the only mode exercised without a vehicle: fake sensors, fake drive
  backend and a synthetic field goal.
- `hardware-probe` starts sensors only and does not navigate.
- `hardware-mission` checks localization readiness and rejects unverified
  differential firmware when that protocol is selected.
- Measured hardware values and acceptance records live in
  `docs/HARDWARE_MEASUREMENTS.md`, `docs/INTEGRATION_ACCEPTANCE.md` and the dated
  test logs. This README makes no success-rate or performance claim.
- The `ackermann_firmware_compat` adapter remains until differential firmware is
  physically verified; do not treat the example config as calibrated.

## Quick start

On the car in VS Code Remote SSH, open either file and click **Run Python File**;
no arguments are needed:

- `code/main_robocup.py`: formal mission, including task-board recognition.
- `code/car_full_flow_test.py`: the same mission with fixed **red 1 / blue 2 /
  green 1**, skipping task-board recognition; yellow remains mandatory.

The quick tuning block at the top of **each** file contains the four right-wall
distances (left / bottom / right / top), standard payload advance, right-mount
payload advance, and final front-wall parking distance. All are metres. The
values are sent with that run and do not change the next radio-started mission.
The default right-wall distances are `(1.600, 1.895, 1.690, 2.330)`, and the two
payload advances are `0.43` and `0.32`.

Both run buttons use the existing `competition-boot.service` listener. Boot still
only starts the HC-15 RUN/STOP/PING listener; it does not start driving. The
existing 20-second preparation and 5-second buzzer remain. Radio RUN uses the
formal mission. Ctrl+C and radio/local STOP cancel the current run.

Missing terminal-wall measurements continue road-wall following; late distance
measurements recover a crossed target. Missing road-wall or complete lidar
feedback waits for a valid observation without a failure timeout. Unavailable
vision and failed payload commands no longer abort the round; failed slots are
not recorded as successful drops. Radio link/callback errors do not cancel an
active mission, and the old total mission timeout is disabled. Normal completion,
explicit STOP and process shutdown still clear drive output.

Run the complete synthetic runtime on Windows, Linux, or macOS without connecting a vehicle:

```powershell
py -3 code\main_robocup.py --config configs\robocup_diffdrive.example.toml --mode dry-run
```

The dry-run uses fake sensors, a fake drive backend, and a synthetic open-field goal. The example geometry and mounts are unverified placeholders. To make a local profile, copy the example and edit only that file:

```powershell
Copy-Item configs\robocup_diffdrive.example.toml configs\robocup_diffdrive.toml
```

`configs/robocup_diffdrive.toml` is tracked (since 2026-10-19) and holds the
measured values of the ROCK 5A car, so a freshly pulled board can run without a
second setup step. On another chassis, edit the ports/pins/geometry in place and
follow [docs/BOARD_PORTING.md](docs/BOARD_PORTING.md), which also documents the
board-level groups, udev rules, overlays, systemd units and ROS environment.
Record and verify geometry and sensor mounts before relying on them for real
motion; there are no geometry/extrinsics or map/footprint `measured` readiness
flags. `hardware-mission` checks localization readiness and rejects unverified
differential firmware when that protocol is selected. `hardware-probe` starts
sensors only and does not run autonomous navigation.

## Runtime modes

| Mode | Purpose |
|---|---|
| `dry-run` | Synthetic sensors, fake drive, short navigation demonstration |
| `replay` | Replay canonical T265/D500 JSONL pose events without hardware or algorithm sleeps |
| `hardware-probe` | Sensor-only hardware startup with probe speed policy; no autonomous task |
| `hardware-mission` | Autonomous hardware mode after the readiness gates pass |

Production entry point: [code/main_robocup.py](code/main_robocup.py). Composition, startup, event loop, and safe shutdown live in [code/robocup_runtime.py](code/robocup_runtime.py). This path uses `DifferentialDrive` and does not import the old competition task runtime.

The 2026 competition route is available with `--mode hardware-mission --competition`
or `--competition-stage` for an individual action. Quick tuning values for the
formal/test missions live at the top of their respective entry files; direct
diagnostic defaults remain in `code/competition_task.py`. See [docs/COMPETITION_TASK.md](docs/COMPETITION_TASK.md) for the stage
commands, existing YOLO model path, atomic functions, and field acceptance order.
The `task-board`, `hc-send`, `yellow-detect`, and `drop` CLI stages operate their
devices directly without starting localization or drive runtime. Measured lane
segments guide the main route, and the finish requires position only.

The formal and fixed-rule test missions share the four-wall route and five SG90
payload outputs. RUN CLOSEs all five slots (1500 us); listener startup, explicit
STOP and completion OPEN all five (1000 us). Each normal drop OPENs only the
assigned slots. Slots 0..3 use the standard detour; yellow slot 4 uses the
right-mount detour. The separate PWM7 camera axis is unchanged. See
[docs/PAYLOAD_SG90.md](docs/PAYLOAD_SG90.md) for permissions and tools.

## Task-board recognition

Task-board startup acquisition is available in `hardware-mission` with
`--task-board-camera` and an explicitly measured `--task-board-turn-deg`.
The car turns, stops, then reads a short camera burst using OpenCV and RapidOCR;
only three agreeing frames with red + blue + green = 4 are accepted. See
[docs/TASK_BOARD_RECOGNITION.md](docs/TASK_BOARD_RECOGNITION.md) for installation,
offline validation, launch arguments, and outstanding physical acceptance.

## Coordinate contract

| Convention | Meaning |
|---|---|
| `base_link` origin | Midpoint between the two driven wheel axes |
| `+X`, `+Y`, `+Z` | Forward, left, up |
| Yaw | Counter-clockwise about `+Z` |
| Internal length/speed/angle/time | m, m/s, rad, monotonic seconds |

Hardware-specific centimetres, clockwise degrees, and protocol units must be converted in adapters before reaching the differential navigation/fusion core. See [docs/COORDINATE_FRAMES.md](docs/COORDINATE_FRAMES.md) and [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Logging, replay, and tests

```powershell
py -3 code\main_robocup.py --mode dry-run --log-dir logs\dry-run
py -3 code\main_robocup.py --mode replay --replay-file logs\dry-run\events.jsonl
py -3 tools\replay_pose_log.py logs\dry-run\events.jsonl --output logs\dry-run\fused.jsonl
py -3 tools\summarize_run.py logs\dry-run\events.jsonl
py -3 -m compileall -q code tools
py -3 -m unittest discover -s code/test -p "test_*.py"
```

Runtime events use bounded asynchronous JSONL logging. `logs/` is ignored by Git. Staged software and physical acceptance requirements are in [docs/INTEGRATION_ACCEPTANCE.md](docs/INTEGRATION_ACCEPTANCE.md).

The differential module boundaries follow common linorobot2 interface ideas but use this repository's Python/C10B/D500 implementation. See [docs/REFERENCE_AND_LICENSE_NOTES.md](docs/REFERENCE_AND_LICENSE_NOTES.md).

## Measurements and calibration

The example profile contains software-test placeholders only. Record actual wheel geometry, sensor extrinsics, motor signs, and C10B protocol behavior in [docs/HARDWARE_MEASUREMENTS.md](docs/HARDWARE_MEASUREMENTS.md), then follow [docs/CALIBRATION.md](docs/CALIBRATION.md). Do not infer verified calibration from the example values. Relative navigation uses the current fused pose frame and a direct path; it does not provide static-map obstacle avoidance. T265 SDK import remains optional and is needed only when starting a real T265 source.

## Removed legacy stack

The old Task 1/Task 2, steering, Ackermann navigation, v1 configuration, serial-screen launcher and ROS2 actuation paths have been removed. The optional ROS2 packages retain D500 sensor and measured-field map utilities only. C10B firmware compatibility remains part of the differential backend until a new firmware protocol is verified. See [docs/LEGACY_CLEANUP.md](docs/LEGACY_CLEANUP.md) for the removal inventory and deployment transition.
