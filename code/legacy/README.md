# Legacy Ackermann modules

The differential RoboCup runtime is the active production path. Ackermann files remain at their established import locations under `code/components/`, `code/config/`, and the root `code/main_task*.py` entries because current old task launchers, ROS bridge consumers, and regression tests import those names directly.

Legacy-only modules include:

- `components/ackermann_drive.py` and `components/steering_servo.py`;
- `components/navigation.py`, `components/competition_track.py`, and old mission/runtime modules;
- `main_task1.py`, `main_task2.py`, `main_radar_camera_line_following.py`, and `competition_task_runtime.py`.
- v1 vehicle profiles such as `configs/car.example.toml` and `configs/cooper_rock5a_l150.toml`, which remain at stable paths for the old loader and task defaults.

Do not add these imports to `main_robocup.py`, `robocup_runtime.py`, the schema-v2 differential factory, differential navigation, or pose fusion. The active path uses `DifferentialDrive`, `DifferentialNavigator`, schema-v2 config, and C10B/D500/T265 adapters. The shared low-level C10B `rear_motor.py` and D500 `radar_driver.py` remain shared validated assets.

`components` and `config` package convenience exports resolve lazily so importing a new differential submodule does not load the steering/Ackermann stack. Import-level regression coverage protects this boundary.
