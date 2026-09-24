# Reference projects and license notes

## Project sources

- The imported `car` repository provides the existing C10B low-level driver, D500 localization, configuration, logging, and legacy Ackermann stack retained in this repository.
- `car_for_robocup` is the differential RoboCup migration target.
- `linorobot2` and `linorobot2_hardware` informed the interface boundaries: body `Twist` separated from wheel kinematics, configurable 2WD geometry, odometry separated from fused pose, and command timeout/watchdog behavior.

## Implementation and license

The differential Python modules were implemented against this project's existing C10B/D500 code and local SI/frame contracts. No linorobot2 source file or non-trivial code fragment was copied into the differential runtime. The work uses architecture and public mathematical/interface ideas only; no ROS 2 or linorobot2 package is required to run `main_robocup.py`.

The referenced linorobot2 repositories use Apache-2.0. If a future change copies a source file or substantial code fragment, retain its copyright/license header, include the Apache-2.0 license and required notices, and record the source file in this document. Do not remove upstream attribution.

## Hardware SDK note

The T265 source imports `pyrealsense2` only when a real device is started. ARM/ROCK 5A support still requires physical installation and device verification; Windows software development uses fake/replay sources and does not establish hardware compatibility.
