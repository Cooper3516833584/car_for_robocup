# Coordinate and time contract

This contract defines the canonical interface for new motion, localization,
and sensor-adapter code. Legacy components retain their existing units behind
explicit adapters until each interface is migrated.

## Canonical `base_link`

The canonical frame uses SI units: metres, seconds, and radians. Its origin is
the midpoint of the left and right active drive-wheel axle line. `+X` points
toward the front of the car, `+Y` points left, and `+Z` points upward. Positive
yaw rotates counter-clockwise about `+Z` (right-hand rule). A canonical
`Pose2D` is `(x_m, y_m, yaw_rad, timestamp_s)`; `Twist2D` is forward `m/s` and
counter-clockwise `rad/s`.

## Frame chain and transform direction

The first localization design is:

```text
map/field
   |
   | map_T_t265_odom  (slow correction from D500 absolute/map observations)
   v
t265_odom
   |
   | t265_odom_T_base_link  (high-rate relative motion from T265)
   v
base_link
   |\
   | \__ lidar_link
   \____ t265_link
```

`parent_T_child` expresses the child frame in the parent frame. Sensor mounting
configuration always describes `base_link -> sensor_link`, including both
translation and rotation. `compose_pose2d(parent_T_child, child_T_object)`
returns `parent_T_object`; it applies the child transform first, then the
parent transform. `inverse_pose2d()` reverses that mapping.

If D500 already produces a pose in the field frame, its adapter can provide a
`map_T_base_link` observation directly. D500 and T265 adapters convert their
legacy units, angle signs, timestamps, and device-specific fields at the
boundary. They do not change the canonical types. T265 is not connected by
this architecture step.

The existing radar and navigation code uses centimetres and degrees, and some
legacy pose origins are expressed at the rear-axle midpoint. Before converting
a pose, confirm whether the driver has already applied its sensor mount and
which robot reference the pose describes. Apply only a remaining, measured
robot-reference offset; never infer it from the sensor name or apply the sensor
mount twice.

## Legacy D500 contract

`code/components/radar_driver.py` confirms that D500 ranges arrive in
millimetres; scan points, local maps, `RadarOdometry.pose`, and
`RadarLocalizationUpdate` positions use centimetres. Yaw is degrees,
clockwise-positive; the point convention is `+X` forward and `+Y` left.
`RadarMount` describes the lidar origin and yaw relative to the old car-body
reference, documented by `code/config` and `docs/CALIBRATION.md` as the rear
axle centre. `scan_points_in_body()` applies that mount before ICP and map
construction, so the resulting accumulated `RadarOdometry.pose` is already
the body/rear-axle reference. Do not apply the lidar mount again to that pose.

ICP matches the current body-frame scan to the previous reference scan and
returns `transform_current_to_reference` as an inter-scan delta in centimetres
and clockwise degrees. `RadarOdometry` integrates accepted deltas into a local
pose starting at identity. Optional wall-line fusion observes field/wall pose;
when accepted by `D500RadarComponent`, the global correction is converted back
to the local alignment and written to `RadarOdometry.pose`, so subsequent
odometry and mapping use the corrected local pose. The D500 component keeps
local odometry separate from optional `DroneGlobalAlignment` map pose.

In this repository, the old rear-axle midpoint and the new differential
`base_link` origin (active-wheel axle-line midpoint) refer to the same physical
point. `RadarPoseAdapter` therefore uses an identity reference offset for this
migration. Its `legacy_T_base_link` option exists for an explicitly measured
offset on a future vehicle; it is a robot-reference transform and must not
repeat `RadarMount`.

Conversion example: a D500 map pose `(100 cm, 0 cm, 0 deg CW)` becomes
canonical `(1.0 m, 0.0 m, 0 rad CCW)`. A legacy `-90 deg CW` heading becomes
`+pi/2 rad CCW`; a base-link offset of `+0.20 m` in legacy forward at that
heading moves the map position by `+0.20 m` in map `+Y`.

## Time contract

Freshness and timeout checks use `time.monotonic()` seconds. Wall-clock time is
for human-readable logs only and must not be subtracted from device timestamps
to determine freshness. Preserve a device timestamp separately when useful,
and document its clock domain. Replay code injects a clock/source timestamp so
the same freshness logic can be tested deterministically.

## Angle boundaries

Canonical algorithms use radians and counter-clockwise-positive yaw. Normalize
angles with `normalize_angle_rad()` into `[-pi, pi)`. Degree/radian conversion
and clockwise/counter-clockwise sign changes belong in sensor or hardware
adapters; mission and navigation logic should not repeatedly convert units.
