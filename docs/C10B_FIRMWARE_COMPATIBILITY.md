# C10B firmware compatibility

## Current car motion verification — 2026-10-06

On `radxa@ROCK-5A`, the original active profile reproduced the vendor reverse
yaw inversion. Requested forward/reverse left arcs produced fused yaw
`+6.856 / -9.476 deg`; requested forward/reverse right arcs produced
`-5.909 / +4.840 deg`. Native quaternion and raw angular velocity agreed.
The temporary `differential_vx_vz` profile then passed all four signs twice:
`+10.386, +7.200, -5.615, -10.182 deg` and
`+9.082, +7.152, -5.570, -10.623 deg`. No C10B reject or event drop occurred.

Evidence: `/home/radxa/car_test_logs/auto_motion_diag_20261006_1655/`,
acquisition commit `8329418`. The 10-second stationary preflight drift was
1.406 mm / 0.0224 deg, with scan and anchor rates 4.288 Hz.

The operator authorized this car's ignored active TOML to receive only the
verified protocol change, as an explicit exception to the normal Git-only
board deployment rule:

```diff
-c10b_diff_firmware_verified = false
+c10b_diff_firmware_verified = true
-protocol_mode = "ackermann_firmware_compat"
+protocol_mode = "differential_vx_vz"
```

The PC profile is updated first; the board update must verify the original
SHA-256 `3122d9a2917fe35570a0239df46126b7eacce62aacde9272caf8114d80aed3c3`
and the PC result. Geometry, firmware conversion width, acceleration, and
speed settings are unchanged. This verifies the installed firmware's reverse
sign behavior. It does not establish full closed-loop motion acceptance;
later rotation/precision tests did not pass. The active TOML stays ignored.

The `C10BDifferentialBackend` accepts SI wheel speeds and delegates framing,
serial I/O, periodic command refresh, watchdog stopping, and close behavior to
the existing `RearMotorDriver`. `ackermann_firmware_compat` applies the
installed firmware's tested motion limits. `differential_vx_vz` removes
the legacy minimum-radius check and compensates for the WHEELTEC `Diff_Car`
firmware's reverse-yaw sign change. In the supplied `CONTROL/control.c`,
`Get_Target_Encoder()` does `if (Vx < 0) Vz = -Vz`; the backend therefore sends
the opposite `Vz` for a moving reverse turn so the physical wheels receive the
requested left/right targets. Straight reverse, forward turns, and in-place
turns keep their previous frames. This sign behavior must be checked on the
installed C10B firmware with a supervised reverse-turn test; the supplied
source alone does not prove which binary is flashed. Select this mode only
after the firmware and full motion chain have been verified on the actual car.

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
