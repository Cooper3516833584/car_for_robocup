# RoboCup staged integration acceptance

Advance one stage at a time. A failed gate means stop and debug that layer before adding speed or hardware complexity. Record the command, config revision, log path, result, and operator for each run.

## Stage 0 — static quality

Run from the repository root on Windows:

```powershell
py -3 -m compileall -q code tools
py -3 -m unittest discover -s code/test -p "test_*.py"
git diff --check
```

Acceptance: zero syntax errors, all new tests pass, and only the eight documented legacy skips remain. The current baseline is 530 tests with 8 skips; update this count after changing tests.

## Stage 1 — hardware-free dry-run

```powershell
py -3 code\main_robocup.py --config configs\robocup_diffdrive.example.toml --mode dry-run --log-dir logs\dry-run
```

Acceptance: the fake drive and fake sensors initialize, fusion and runtime steps execute, the synthetic open field runs a short demonstration goal through navigation and the fake drive, no COM port is opened, no T265 SDK is needed, all actuators remain fake, and shutdown closes the fake drive. The CLI warns that example geometry/mounts are unmeasured.

## Stage 2 — synthetic and recorded replay

Run the generated trajectory regression and runtime replay tests:

```powershell
py -3 -m unittest discover -s code/test -p test_pose_log_replay.py
py -3 -m unittest discover -s code/test -p test_robocup_runtime.py
```

The synthetic trajectory covers 1 m straight travel, a 90° in-place turn, further travel, gradual T265 drift, D500 corrections, a gated D500 outlier, and a low-confidence T265 interval. Acceptance: the outlier is rejected, a fresh D500 pose produces `t265_degraded` during T265 dropout, the fused final pose remains near the reference, and exhausted sensors during commanded motion produce a zero command and `SAFE_STOP`. Navigation tests separately assert path tracking and zero-command goal arrival.

To replay an existing canonical pose stream through the runtime or compare fusion output under another config:

```powershell
py -3 code\main_robocup.py --mode replay --replay-file logs\dry-run\events.jsonl
py -3 tools\replay_pose_log.py logs\dry-run\events.jsonl --config configs\robocup_diffdrive.example.toml --output logs\replay\fused.jsonl
py -3 tools\summarize_run.py logs\dry-run\events.jsonl
```

Replay uses the recorded event times without algorithm sleeps. Keep replay inputs and outputs under ignored `logs/`; commit only small synthetic fixtures.

## Stage 3 — sensors only on the car

Disconnect motor power or otherwise make wheel actuation impossible. Run `hardware-probe` and record at least 60 seconds stationary data, then controlled hand translations and rotations. Check the T265/D500 adapter signs and offsets against the coordinate contract in [COORDINATE_FRAMES.md](COORDINATE_FRAMES.md).

Acceptance: forward raises canonical x, left raises y, counter-clockwise rotation raises yaw, stationary `base_link` does not trace a sensor-offset circle, and static readings do not show unexplained jumps. Do not proceed if any adapter or freshness result is unclear.

## Stage 4 — wheels raised, low-speed sign check

Use a separate, explicitly operator-triggered motor smoke procedure with wheels off the ground and an emergency stop within reach. Check forward, reverse, positive/negative angular commands, stop, Ctrl+C stop, and watchdog timeout.

Acceptance: both wheels run forward for positive linear speed; positive yaw is left-reverse/right-forward; stop is immediate; watchdog timeout stops motion. Keep wheel/linear/angular limits at their probe caps.

## Stage 5 — open-loop floor movements

Only after sensor and sign checks pass, use an empty floor area, 0.05–0.10 m/s maximum, an operator at the emergency stop, and clear perimeter. Check short forward/reverse movements, ±90° in-place turns, and the configured curvature limits. Record requested and observed distance/yaw from JSONL.

Acceptance: direction, distance scale, turn direction, stop and backend accept/reject records match expectations. Recalibrate the backend before changing navigation gains.

## Stage 6 — localization closed loop

With a verified map and low speed, approach front-left, front-right, straight-ahead, and position-fixed yaw-only goals. Record position/yaw error, settle time, overshoot, T265 confidence, and D500 innovation. Do not claim accuracy tighter than the measurements support.

Acceptance: goals settle repeatably, final yaw is handled separately, and degraded localization reduces speed while lost localization stops the base.

## Stage 7 — obstacles and planning

Use fixed obstacles in a controlled test area. Check path around an obstacle, inflated footprint clearance, map boundary clearance, obstacle appearance during motion, and a no-path case.

Acceptance: paths preserve footprint clearance; blocked/no-path conditions stop safely and never drive through unknown cells.

## Stage 8 — mission integration

Only after Stages 0–7 pass and the readiness gate accepts measured geometry/extrinsics, connect recognition, payload actions, and return goals. Confirm payload work holds zero base command and pose loss prevents both movement and payload release.

## Current status

Stages 0–2 are software-verifiable and covered by automated tests. No physical robot, measured geometry/extrinsics, verified differential firmware, or real sensor run was available for this migration. Stages 3–8 remain pending; do not treat software tests as physical acceptance. `hardware-mission` remains blocked while required calibration flags are false.

## Regression after code changes

After changing kinematics, C10B backend, frame adapters, fusion, navigation, config, or watchdog, rerun Stage 0 and the related targeted tests before moving to a higher stage.
