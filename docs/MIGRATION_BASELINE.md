# Migration baseline audit

This document records the imported Ackermann repository before differential-drive changes.

## Checkout and test status

- Baseline commit: `704fb64a57d30b2ff84743da68635c39e160e2b3` (`chore: import verified Ackermann car baseline`).
- Python: CPython 3.13.5 on Windows.
- `py -3 -m compileall code`: passed.
- `py -3 -m unittest discover -s code/test -p "test_*.py"`: 438 tests run, 430 passed, 8 skipped, 0 failures/errors.
- The eight skips are legacy assertions in `test_radar_camera_line_main.py` for removed fixed-course trim, terminal visual steering changes, and post-lap extension behavior. The camera test guarded by OpenCV availability ran in this environment.
- The suite logs expected mocked hardware failures (for example GPIO unavailable); they did not fail tests. Standalone UART, motor, steering and GPIO hardware probes were not run.
- The imported patch file `code/test/c10b_a2_a3_gimbal.patch` has four pre-existing whitespace errors reported by `git diff --check` (`space before tab`); it was kept unchanged as part of the source baseline.

## Actual production entry points

- `code/main_task1.py` and `code/main_task2.py` select a mission and delegate to `code/components/radar_camera_line_following.py` (`run_mission`). The shared application initializes radar, camera, Ackermann drive and competition-track behavior from a TOML profile.
- `code/components/fixed_track_runtime.py` and `code/components/competition_track.py` contain reusable track/follower runtime logic. `code/components/competition_task_runtime.py` is a general task runtime wrapper.
- `code/components/navigation.py` implements a separate Ackermann autonomous point/pose navigation component. `code/components/coordinate_navigation.py` and `code/components/grid_rescue_mission.py` provide coordinate/grid task logic.
- `code/main_fixed_track_test.py`, `code/main_radar_camera_line_following.py`, and root-level `code/competition_task_runtime.py` named in older notes do not exist at those paths in this checkout. The shared mission core is under `code/components/`.
- `code/config/loader.py`, `models.py`, and `factory.py` load and inject TOML settings. Formal profiles are in `configs/`; hardware factories assemble rear motor, steering, Ackermann, radar mount and navigation geometry.

## C10B motor API and constraints

`code/components/rear_motor.py` owns the existing C10B command protocol and its periodic writer/watchdog. Relevant public types/functions are `WheelSpeeds`, `ChassisCommand`, `wheel_speeds_to_chassis()`, `build_velocity_frame()`, and `RearMotorDriver`. Driver lifecycle and command methods are `start()`, `set_linked()`, `set_wheels(left_mm_s, right_mm_s)`, `set_left()`, `set_right()`, `stop()`, and `close()`.

`set_wheels()` accepts signed left/right rear wheel targets in **mm/s**, validates finite values and per-wheel speed limits, then converts the requested pair to chassis values:

- `Vx = (left + right) / 2` in mm/s.
- `Vz = (right - left) / track_width` in rad/s, quantized to mrad/s in the C10B frame.
- The installed firmware uses a protocol track width of **164 mm** and clamps turns tighter than a **350 mm minimum radius**. The caller rejects an unrepresentable turn rather than silently changing the requested wheel pair.
- Counter-rotating wheels at `Vx = 0` are rejected by default. They require the explicit `allow_in_place_rotation` option, reserved for the grid-rescue maneuver.
- The command is an 11-byte `7B ... 7D` frame with XOR checksum, sent over 115200 baud. The worker refreshes the latest command at 20 Hz and sends zero after the command timeout; close also sends zero frames.

The physical track width is a separate parameter from the firmware's 164 mm protocol width. The checked-in vehicle profile and `AGENTS.md` record the measured physical wheel-center track as 117.1 mm; do not substitute it into the firmware conversion.

## Ackermann and steering coupling

`AckermannDrive` in `code/components/ackermann_drive.py` directly composes `RearMotorDriver` and `FrontSteeringServo`, calculates a motion plan, and enforces the physical steering/radius and wheel-speed limits. It is created by `code/config/factory.py` and the shared competition application. Production users include `radar_camera_line_following.py`, `competition_track.py`, `fixed_track_runtime.py`, and Ackermann `navigation.py`; grid rescue explicitly opts into the special in-place mode where configured.

`FrontSteeringServo` in `code/components/steering_servo.py` maps a logical steering angle to calibrated PWM. Its production dependency is through `AckermannDrive` / config factory; `radar_camera_line_following.py` and `fixed_track_runtime.py` also refer to the steering component/calibration in their application setup. Tests and manual hardware probes exercise it directly. TOML calibration and the PWM HAL are in `code/config/` and `code/hal/`.

## D500 API, units and coordinate conventions

The read-only implementation is `code/components/radar_driver.py`. Its protocol/localization API includes `D500PacketParser.feed(bytes)`, `RadarScanAssembler.feed(packet)`, `ICPScanMatcher.match(...)`, `RadarOdometry.update(scan)`, and the `D500SerialDriver` / `D500RadarComponent` start/close lifecycle. The parser handles 47-byte `54 2C` frames and CRC-8/0x4D; the assembler produces complete revolutions; localization uses ICP and optional wall fusion. `TrustedNavigationMap` filters accepted localization updates before incorporating returns into its occupancy grid.

D500 raw range values are millimetres; radar points, maps, and `Pose2D` translations use centimetres. Radar pose is represented with clockwise-positive yaw in degrees. The Cartesian axes are `+X` forward and `+Y` left. The localization origin is the car/body frame as configured by `RadarMount` (forward and left offsets); formal geometry and docs treat the navigation reference point as the rear-axle midpoint, so mount offsets must be expressed consistently with that point. The D500 parser, scan assembler, ICP core, and wall-line core are validated legacy assets to freeze during migration.

## Navigation pose units, heading and origin

`NavigationPose` and `NavigationGoal` in `code/components/navigation.py` use centimetres and degrees. The pose origin is the midpoint of the rear axle. Navigation heading is counter-clockwise positive, normalized to `[0, 360)`; when adapting radar pose use `heading = (-yaw_cw_deg) % 360`. Radar's coordinate frame also uses `+X` forward and `+Y` left. The local origin is established by the radar/field calibration at runtime; D500 global alignment must be explicitly calibrated and is not inferred from the navigation heading.

The imported repo has verified Ackermann vehicle measurements in its current `AGENTS.md` and TOML profile (117.1 mm physical track, 142.5 mm wheelbase, 230 x 145 mm body, and 164 mm firmware conversion track). This differs from the migration package's initial assumption that no dimensions have been measured. Keep measurements configuration-driven and preserve the documented provenance; do not replace them with dimensions from another vehicle.

## Competition-only and legacy task code

- Task 1 and Task 2 are the current RoboCup competition entries, sharing radar/camera line following and track-segment behavior. Mission-specific speeds, fleet requests and alarm duration are in TOML.
- `code/components/competition_track.py`, `radar_camera_line_following.py`, `fixed_track_runtime.py`, and `grid_rescue_mission.py` encode competition/task behavior and should be isolated from a new differential-drive core rather than used as its architecture.
- `code/former_code/radar_point_navigation.py` and `former_code/test/` contain archived coordinate-navigation experiments. `code/test/` contains both unit tests and explicitly named hardware/bring-up scripts; hardware scripts are not ordinary unit tests.

## Ackermann coupling search and migration disposition

- **Must replace in the differential production path:** `AckermannDrive` use in the production base/navigation command chain; steering-servo output; Ackermann path assumptions in `navigation.py` when creating the new differential navigator. Keep the old implementation available until the new chain is proven.
- **Must retain behind stable interfaces:** C10B serial frame encoding, periodic sender/watchdog and safety-stop behavior; D500 packet parser, scan assembly and localization; battery monitoring; sound/light alarm; generic map/occupancy utilities and TOML configuration infrastructure.
- **Isolate as old task code:** camera/radar competition follower, track runtimes, task 1/task 2 orchestration, coordinate/grid-rescue missions, and archived former-code experiments.
- **Potentially reusable mathematics/data structures:** coordinate transforms, occupancy grid and collision checks, waypoint/track utilities, ICP/wall-line estimation, and config validation. Reuse only after unit/coordinate contracts are explicit.

## No-regression freeze list

Until an adapter requires a change and regression coverage exists, treat these as frozen:

- D500 packet parser and CRC implementation;
- radar scan assembler;
- ICP matching and odometry core;
- wall-line localization/fusion core;
- C10B byte-frame encoder;
- the tested periodic sender/watchdog thread and safe-stop-on-close behavior;
- battery and sound/light alarm behavior, except for a thin interface adapter if required.

Any future edits to these modules require a focused regression test before the implementation change.
