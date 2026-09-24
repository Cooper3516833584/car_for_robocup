# RoboCup Differential Ground Robot

This repository contains the RoboCup ground vehicle software for a differential-drive chassis with two driven center wheels and passive front/rear supports. The production stack uses C10B motor control, D500 lidar localization, optional Intel RealSense T265 odometry, canonical SI poses, and a separate mission runtime.

## Quick start

Run the complete synthetic runtime on Windows, Linux, or macOS without connecting a vehicle:

```powershell
py -3 code\main_robocup.py --config configs\robocup_diffdrive.example.toml --mode dry-run
```

The dry-run uses fake sensors, a fake drive backend, and a synthetic open-field goal. It warns that the example geometry and mounts are unmeasured. To make a local profile, copy the example and edit only the local file:

```powershell
Copy-Item configs\robocup_diffdrive.example.toml configs\robocup_diffdrive.toml
```

`configs/robocup_diffdrive.toml` is ignored by Git. Keep calibration flags false until measurements are recorded. `hardware-mission` rejects unmeasured geometry/extrinsics and unverified differential firmware when that protocol is selected. `hardware-probe` starts sensors only and does not run autonomous navigation.

## Runtime modes

| Mode | Purpose |
|---|---|
| `dry-run` | Synthetic sensors, fake drive, short navigation demonstration |
| `replay` | Replay canonical T265/D500 JSONL pose events without hardware or algorithm sleeps |
| `hardware-probe` | Sensor-only hardware startup with probe speed policy; no autonomous task |
| `hardware-mission` | Autonomous hardware mode after the readiness gates pass |

Production entry point: [code/main_robocup.py](code/main_robocup.py). Composition, startup, event loop, and safe shutdown live in [code/robocup_runtime.py](code/robocup_runtime.py). This path uses `DifferentialDrive` and does not import the old competition task runtime.

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

## Measurements and calibration

The example profile contains software-test placeholders only. Record actual wheel geometry, body footprint, sensor extrinsics, motor signs, and C10B protocol behavior in [docs/HARDWARE_MEASUREMENTS.md](docs/HARDWARE_MEASUREMENTS.md), then follow [docs/CALIBRATION.md](docs/CALIBRATION.md). Do not set measured flags based on the example values. T265 SDK import remains optional and is needed only when starting a real T265 source.

## Legacy Ackermann stack

`code/main_task1.py`, `code/main_task2.py`, their camera/black-line runtime, steering servo, and Ackermann navigation are retained for the older vehicle. They are not dependencies of `main_robocup.py`. Stable legacy import paths remain in place because existing launchers and regression tests use them; see [code/legacy/README.md](code/legacy/README.md) and [docs/legacy/ACKERMANN_README.md](docs/legacy/ACKERMANN_README.md).
