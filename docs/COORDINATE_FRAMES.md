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
legacy pose origins are expressed at the rear-axle midpoint. The adapter must
account for the exact configured `RadarMount`/reference-point offset when
converting that pose to the canonical active-wheel axle midpoint. Never infer
the offset from the sensor name.

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
