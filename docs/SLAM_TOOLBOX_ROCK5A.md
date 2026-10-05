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
In this mode, `--goal-x`, `--goal-y`, and optional `--goal-yaw` refer to the
current fused pose frame. Selecting the localization profile alone does not
enqueue a movement. Direct navigation follows the start-to-goal line; it does
not check a static map, enforce a footprint, or avoid obstacles.

The user has accepted the relative localization chain for task use. The
profile leaves the board configuration file untouched. Hardware mission
readiness still checks localization and the C10B firmware mode. At run time,
motion waits for a fresh accepted SLAM anchor and stops on localization loss.
Start the ROS launch separately and verify its topics before a supervised
mission run in a cleared area.

## Localization sessions and bounded anchor confirmation

Each new robot process sets its own T265 local origin. A `slam_toolbox` sidecar
left running across separate one-action processes may still hold the previous
map/odom session. Re-establish the sidecar and T265 session together while the
robot is stationary, before the next preflight; do not restart the sidecar
during an action. This is a session precaution, not a proven explanation for
every observed map correction.

The bridge publishes about five scans per second. Fusion rejects a large
single-anchor correction and waits for three mutually consistent anchors.
During that confirmation, a running action can continue on the last accepted
map transform and fresh T265 pose for at most one second from the accepted
anchor, provided the latest candidate and bridge TF stay fresh. Preflight
still requires a recently accepted anchor. A far outlier, stale SLAM source,
expired confirmation window, or lost T265 continues to stop motion. The
`slam_anchor` event records the source TF time, receipt time, acceptance and
rejection reason; `fused_pose` records the accepted-anchor age and whether
confirmation is pending.
