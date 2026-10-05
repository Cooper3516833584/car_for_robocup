# Optional ROS2 sensor and map utilities

The Ackermann base bridge, mission-to-Nav2 bridge, Twist-to-steering converter,
Nav2 motion parameters, old vehicle footprint, behavior trees and autonomous
bringup have been removed. This checkout does not provide a replacement Nav2
chassis bridge. Differential motion runs through `code/main_robocup.py`.

Retained ROS2 utilities:

- `car_ros_bridge.d500_localization_node` publishes sensor data, odometry and TF.
- `car_ros_bridge.ros_conversions` and `field_geometry` remain hardware-free helpers.
- `car_nav_bringup.generate_field_map` creates map files from measured polygons.
- `car_nav_bringup hardware.launch.py` now launches only D500, with hardware disabled by default.
- `code/scripts/build_ros2.sh` builds/tests these remaining packages.

Do not run this D500 bridge at the same time as the standalone runtime's D500
owner. Supply measured radar mount values explicitly when selecting either path.

```bash
ros2 launch car_nav_bringup hardware.launch.py use_hardware:=false \
  radar_x_m:=MEASURED_X radar_y_m:=MEASURED_Y radar_yaw_rad:=MEASURED_YAW
ros2 run car_nav_bringup generate_field_map measured-field.yaml maps/field
```

The package name `car_nav_bringup` is retained for these existing utilities; it
no longer launches autonomous navigation. See `LEGACY_CLEANUP.md` for removal
inventory and migration of old systemd units. Actual ROS2 launch and board
deployment were not performed during this software cleanup.