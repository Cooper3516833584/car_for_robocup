# C10B firmware compatibility

The `C10BDifferentialBackend` accepts SI wheel speeds and delegates framing,
serial I/O, periodic command refresh, watchdog stopping, and close behavior to
the existing `RearMotorDriver`. `ackermann_firmware_compat` applies the
installed firmware's tested motion limits. `differential_vx_vz` removes only
the legacy minimum-radius check and must be selected only after the firmware
and full motion chain have been verified on the actual car.

The following values describe different things and must stay separate:

| Parameter | Meaning | Use |
| --- | --- | --- |
| `physical_track_width_m` | Distance between the actual left and right drive-wheel contact centers | Differential kinematics and physical vehicle geometry |
| `firmware_track_width_m` | Width compiled into the old C10B firmware's `Vx/Vz` to wheel-speed mapping | Compatibility protocol conversion only |
| `firmware_min_turn_radius_m` | Minimum radius enforced by the old Ackermann firmware | Compatibility-mode feasibility gate only |

The imported car baseline records a **117.1 mm** measured physical track, a
separate **164 mm** C10B firmware conversion width, and a legacy minimum turn
radius of **350 mm**. Do not replace one with another. The v2 example uses
placeholders and labels them unmeasured.

The wrapper converts metres per second to the old driver's millimetres per
second at its call boundary. It does not pack bytes. Small-radius commands in
compatibility mode raise `UnsupportedFirmwareMotion`; they are not silently
clipped into a different path. Counter-rotating wheels require the explicit
`allow_in_place_rotation` setting. The lower-level driver's radius enforcement
remains enabled by default for all existing callers.
