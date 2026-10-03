# ROCK 5A T265 + D500 SLAM sidecar

The active robot entry remains `code/main_robocup.py`. The optional bridge uses
the existing T265 adapter and complete D500 revolutions; it does not open a
second serial port or send drive commands. The default config keeps the legacy
fusion backend and leaves the bridge disabled.

## Frames and clocks

`slam_map -> t265_odom -> base_link -> d500_link` is the ROS chain. The bridge
publishes the fusion module's continuity corrected T265 pose as odometry TF and
the configured D500 mount as static TF. D500 clockwise angles are converted to
counterclockwise 0.5 degree bins, 720 beams. Both messages use measurement
monotonic times mapped to the ROS clock. Core localization remains on monotonic
time. `field -> slam_map` is established only from an accepted fixed wall
observation when field coordinates are required.

## Opt in sequence

1. Keep `backend = "legacy"`; set `[localization.slam] enabled = true` on a
   separately reviewed hardware profile for ROS publish only. The default
   example profile is unchanged in behavior.
2. Start the Python runtime with a ROS enabled Python environment, then start
   `ros2 launch /absolute/path/to/car/launch/robocup_slam.launch.py` separately.
   The launch file starts only `slam_toolbox` in asynchronous mapping mode.
3. Check `/d500/scan`, `t265_odom -> base_link`, `base_link -> d500_link`, and
   `slam_map -> t265_odom` with ROS CLI. Compare scan direction and timestamps
   against measured motion before allowing the SLAM pose into navigation.
4. For a no drive comparison, set `backend = "slam_toolbox"` only in an
   offline/replay or supervised localization run. A hardware mission with this
   backend is blocked until `hardware_mission_validated = true` and the existing
   hardware readiness checks pass. Record localization accuracy, concurrent
   CPU/memory and serial timing before setting that flag.

The bridge never waits for scan matching in the control loop. Old scans are
overwritten, and stale or rejected SLAM anchors cannot satisfy hardware mission
readiness. If ROS is absent, importing the ordinary runtime still works, but
starting the optional bridge reports a failed sidecar.

## Debian 12 board

The ROCK 5A runs Debian 12/aarch64. ROS Humble is installed only in the new
user owned `/home/radxa/robocup_ros/env` RoboStack environment; the system ROS
and apt configuration are untouched. Its Python is pinned to 3.11 to match the
board's existing T265 extension. OpenCV is pinned below 5 so that installing it
does not remove the Humble packages. The existing `pyrealsense2` extension is
linked from `/home/radxa/robocup_ros/python_ext`; no system file was changed.

Use the same environment for the optional SLAM launch and the robot runtime:

```bash
export MAMBA_ROOT_PREFIX=/home/radxa/robocup_ros/root
export PYTHONPATH=/home/radxa/robocup_ros/python_ext:/home/radxa/car/code
/home/radxa/robocup_ros/bin/micromamba run -p /home/radxa/robocup_ros/env \
  ros2 launch /home/radxa/car/launch/robocup_slam.launch.py
```

In a separate terminal, use the same two environment variables and
`micromamba run -p /home/radxa/robocup_ros/env python` for the robot entry.
The ROS launch and hardware runtime have not been started as part of the
installation. Confirm scan, TF, timestamps, T265 import, and CPU load with a
supervised stationary test before enabling the SLAM backend for a mission.

No hardware mission is implied by installing the environment or pulling code.

## Relative task localization

For hardware missions whose movement and turns are relative to the starting
pose, `code/main_robocup.py` applies the relative SLAM profile by default.
`--relative-slam` selects the same profile in other modes;
`--localization-from-config` explicitly uses the TOML localization settings.
Task code that calls `build_runtime()` directly must apply
`accepted_relative_slam_profile(config)` itself. This in-memory profile selects
the T265 confidence threshold 2/3, enables the D500 complete-scan SLAM path,
and does not require a fixed-wall field anchor. The fused pose is passed to
`runtime.motion.step()` for pending `drive_distance()` and `rotate()` actions.
The CLI rejects `--goal-*` field coordinates in this mode; selecting the
localization profile alone does not enqueue a movement.

The user has accepted the relative localization chain for task use. The
profile leaves the board configuration file untouched. Hardware mission
readiness still checks measured drive geometry, sensor extrinsics, the task
map and robot footprint, and the C10B firmware mode. At run time, motion waits
for a fresh accepted SLAM anchor and stops on localization loss. Start the ROS
launch separately and verify its topics before a supervised mission run.
