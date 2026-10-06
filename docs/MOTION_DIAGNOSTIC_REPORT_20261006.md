# Motion diagnostics round — 2026-10-06

## Scope and safety

This round records read-only deployment evidence and adds offline diagnostics. It does not change motion-control parameters, start ROS, or command the wheels. No hardware or external chassis-yaw acceptance test was performed.

## Phase 0 — deployment and history evidence

| Item | Result |
| --- | --- |
| PC repository | main at 45dac0d32ccff553fdf2119965ad9a34af8906f2 |
| Board repository | Started at 2abbac303ce9aa26ccb9ddd3773e98250551e321; snapshot captured at diagnostic commit 0958257c5d98ce01947a45761b6d62a440926ef1, then report-only commit 10514a2 was fast-forwarded; final tracked worktree clean |
| Board active config SHA-256 | 3122d9a2917fe35570a0239df46126b7eacce62aacde9272caf8114d80aed3c3 |
| Effective protocol | ackermann_firmware_compat |
| C10B differential firmware verified | false |
| Geometry / protocol track width | 0.198 m / 0.164 m |
| Navigation tolerances | position 0.03 m; yaw 0.0523598776 rad |
| Drive limits | linear 0.20 m/s; angular 0.80 rad/s; linear acceleration 0.30 m/s²; angular acceleration 1.00 rad/s² |
| T265 mount | x −0.258 m, y 0, z 0.076 m, roll/pitch 0, yaw π rad |
| Localization backend | legacy |

The read-only board snapshot is at /home/radxa/car_test_logs/motion_diag_snapshot_20261006_01.json. It contains the full critical-source SHA-256 map, Python/platform version, effective config, and UTC timestamp. The snapshot command did not open sensors or start ROS.

The active board profile was not changed during alignment. An untracked backup file made the board tree appear dirty before cleanup; it did not conflict with the fast-forward. The final board status is clean.

The board retained the expected configuration fingerprint for stage 4 in configs/robocup_diffdrive.toml.bak-noservo (247eba20fa90109f396aba61f50cc53cfa5d013651cb7e4cf70ad45a77cec95e). It was removed later at the user's request to clear backups. The active profile remains at SHA-256 3122d9a2917fe35570a0239df46126b7eacce62aacde9272caf8114d80aed3c3.

The stage-4 pose_fusion.py fingerprint (01a73873c9503a7e4d65b8f247eb3402, after CR removal) was not found in the board backup tree, the board's unreachable Git objects, or the PC's unreachable commit objects. Exact stage-4 localization source recovery is therefore **unknown / not recovered**. Board manifests for 2026-10-04 _04 and _05 report HEAD 792d909; their configuration hashes vary. In _04, 21 manifests were found, with 12 at f8c27a23c6da5d12126017b6a7bee1b3cb125a217118cc88615b4e63c3b06984 and 9 at fdee1ad2ea59a97ce569922d2d4f738d701584b9777e5518001cc8e60415ab65. In _05, 20 manifests were found, with 6 at the expected stage-4 hash above and 14 at the fdee... hash. A single run group must not be inferred from HEAD alone.

## Phase 1 — offline diagnostics added

- tools/motion_diag_snapshot.py writes a read-only JSON fingerprint to a caller-selected path outside the repository.
- tools/closed_loop_motion.py records working-tree state, critical source hashes, effective motion/navigation/T265 configuration, localization backend, and Python/platform metadata. It atomically updates the manifest with the T265 serial after runtime startup; unavailable values remain unknown.
- code/robocup_runtime.py records the original mapped T265 measurement time before causal clamping, the time used afterward, receive time, clamp status, raw velocity and angular velocity, including rejected samples.
- code/components/pose_fusion.py exposes a snapshot of SLAM anchor innovation, candidate count/age, accepted-anchor age, and migration state. slam_anchor events record those fields.
- C10B backend/runtime diagnostics record the wheel pair passed to the frame driver and the quantized Vx/Vz command intent when available.
- tools/closed_loop_analyze.py counts unique slam_anchor events, renames the adapted T265 pose summary to t265_adapter, unwraps its yaw, and labels repeated fused-sample innovation counts accurately.
- tools/yaw_truth_analyze.py aligns an external truth CSV with the event log and reports fused, adapter, native-quaternion yaw, angular velocity, command, anchor diagnostics, time offset, and synchronization uncertainty. Physical-stop follow-ups are reported when the CSV marks physical_stop.
- closed_loop_motion now labels its precision result as fused_precision_met with precision_source: fused_pose; the old precision_met key remains as a compatibility alias.
- Corrected an existing test fixture that declared devices.servo twice while checking unknown device-table rejection.

## Verification

- python -m compileall -q code tools — pass.
- python -m unittest discover -s code/test -p "test_*.py" — 726 tests passed, 7 skipped.
- git diff --check — pass.

## Still unverified

- No repeated wheel-actuation sequence, T265 stationary/manual-rotation sequence, external yaw truth, or real-car motion matrix was run.
- No independent yaw CSV/video was available. Fused-pose precision is still internal closed-loop evidence only.
- Exact stage-4 localization source was not recovered; deleting the user's requested backups removed the matching config copy noted above.
- Do not describe the motion problem as solved.
