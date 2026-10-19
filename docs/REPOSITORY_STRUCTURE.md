# Repository Structure

> Describes the current layout; nothing was moved or renamed by this change.

```text
car_for_robocup/
├── code/
│   ├── main_robocup.py         formal mission entry + runtime modes
│   ├── robocup_runtime.py      runtime composition and safe shutdown
│   ├── competition_boot.py     HC-15 RUN/STOP listener
│   ├── core/                   canonical SI types, frames, health
│   ├── config/                 TOML models, readiness checks, factories
│   ├── components/
│   │   ├── differential_navigation.py  directed-line Pure Pursuit
│   │   ├── differential_drive.py       limits, watchdog, safe stop
│   │   ├── differential_kinematics.py  pure math
│   │   ├── c10b_diff_backend.py        C10B serial backend
│   │   ├── pose_fusion.py, radar_pose_adapter.py, t265_pose_adapter.py
│   │   ├── task_board_reader.py, yolo_cpu.py
│   │   ├── payload_servos.py, servo_axis.py
│   │   ├── hardware_control_lock.py
│   │   └── diagnostics_log.py, pose_log_replay.py
│   ├── test/                   unittest suite
│   ├── tools/                  scripts and hardware probes
│   ├── legacy/                 archived code (reference only)
│   └── ros2_ws/                optional ROS 2 utilities
├── configs/                    tracked measured profile + example TOML
├── models/                     task-board YOLO weights
├── launch/, deploy/            runtime launch, systemd units, board setup
│   ├── board/                  ROCK 5A groups, udev, PWM0 helper
│   └── ros_env/                pinned ROS 2 + vision environment recipe
├── docs/                       architecture, calibration, measurements, acceptance
└── former_code/                earlier implementation (read-only)
```

## Entry points

| Purpose | Command |
|---|---|
| Formal mission | `code/main_robocup.py` |
| Full-flow test | `code/car_full_flow_test.py` |
| Boot listener | `code/competition_boot.py` |

## Where to add code

- Motion math → `code/core/` or `code/components/differential_*`
- Hardware backend → `code/components/*_backend.py`
- Mission logic → `code/robocup_runtime.py` / `code/mission_control.py`
- Tests → `code/test/`
- Probes/scripts → `tools/`

`docs/ARCHITECTURE.md` defines the layering rules and must be updated with any
structural change.

## Generated / ignored

`runtime/`, `logs/`, `__pycache__/`, `code/radar_center_config.json` and test
temp directories are ignored. `configs/robocup_diffdrive.toml` is tracked since
2026-10-19; see `docs/BOARD_PORTING.md` for the board-level layout.

## Related documents

- `docs/SYSTEM_ARCHITECTURE.md`, `docs/ARCHITECTURE.md`
- `docs/HARDWARE_MEASUREMENTS.md`, `docs/INTEGRATION_ACCEPTANCE.md`
- `docs/BOARD_PORTING.md`, `docs/REFERENCE_AND_LICENSE_NOTES.md`
- `docs/TESTING.md`
