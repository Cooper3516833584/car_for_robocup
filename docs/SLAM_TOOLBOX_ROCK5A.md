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

The currently observed ROCK 5A runs Debian 12/aarch64 and has no ROS 2
installation. Do not install Ubuntu 22.04 ROS apt packages into this system.
A separate user owned RoboStack Humble environment is a candidate because it
provides Linux aarch64 packages. Keep it in a new directory, without editing
existing system or robot files. The robot's `pyrealsense2` is currently a
CPython 3.11 extension, so the environment must use Python 3.11 if the Python
runtime and ROS bridge share a process. Check imports of `rclpy`,
`slam_toolbox`, `serial`, and `pyrealsense2` in that environment before any
sensor or motor test. A ROS environment that cannot import the current T265
extension is not a working runtime configuration.

No hardware mission is implied by installing the environment or pulling code.
