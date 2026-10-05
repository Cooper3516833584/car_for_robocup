# RoboCup Differential Ground Robot

This repository contains the RoboCup ground vehicle software for a differential-drive chassis with two driven center wheels and passive front/rear supports. The production stack uses C10B motor control, D500 lidar localization, optional Intel RealSense T265 odometry, canonical SI poses, and a separate mission runtime.

## Quick start

Run the complete synthetic runtime on Windows, Linux, or macOS without connecting a vehicle:

```powershell
py -3 code\main_robocup.py --config configs\robocup_diffdrive.example.toml --mode dry-run
```

The dry-run uses fake sensors, a fake drive backend, and a synthetic open-field goal. The example geometry and mounts are unverified placeholders. To make a local profile, copy the example and edit only the local file:

```powershell
Copy-Item configs\robocup_diffdrive.example.toml configs\robocup_diffdrive.toml
```

`configs/robocup_diffdrive.toml` is ignored by Git. Record and verify geometry and sensor mounts before relying on them for real motion; there are no geometry/extrinsics or map/footprint `measured` readiness flags. `hardware-mission` checks localization readiness and rejects unverified differential firmware when that protocol is selected. `hardware-probe` starts sensors only and does not run autonomous navigation.

## Runtime modes

| Mode | Purpose |
|---|---|
| `dry-run` | Synthetic sensors, fake drive, short navigation demonstration |
| `replay` | Replay canonical T265/D500 JSONL pose events without hardware or algorithm sleeps |
| `hardware-probe` | Sensor-only hardware startup with probe speed policy; no autonomous task |
| `hardware-mission` | Autonomous hardware mode after the readiness gates pass |

Production entry point: [code/main_robocup.py](code/main_robocup.py). Composition, startup, event loop, and safe shutdown live in [code/robocup_runtime.py](code/robocup_runtime.py). This path uses `DifferentialDrive` and does not import the old competition task runtime.

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
