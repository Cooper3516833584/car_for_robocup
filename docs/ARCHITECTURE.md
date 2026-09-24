# Target architecture

The new differential motion path is layered so mission logic does not depend
on a board protocol:

```text
Mission
  -> DifferentialNavigation
    -> Twist2D (m/s, rad/s)
      -> DifferentialDrive (limits, acceleration and watchdog)
        -> DifferentialKinematics
          -> C10B backend

D500 -> radar pose adapter --\
                              -> PoseFusion -> canonical Pose2D -> Navigation
T265 -> T265 pose adapter ---/
```

`code/core/` owns immutable SI motion types, SE(2) helpers, shared health
states, and the frame/time contract. It must not import device drivers.
Adapters isolate existing hardware conventions, including D500 centimetres
and clockwise-positive degrees and C10B protocol units. The C10B low-level
frame encoder, periodic sender, watchdog, and safe-stop behavior stay behind
the backend interface.

Pose fusion initially keeps T265 relative odometry as the high-rate local
motion source and applies slow D500 corrections to `map_T_t265_odom`. Raw
sensor poses and fused field poses remain distinct. Navigation consumes a
canonical pose and emits `Twist2D`; it does not construct serial frames or
write sensor-specific angles.

`AckermannDrive` and the existing Task 1/Task 2 entry points are the legacy
production path. They remain available while the differential path is brought
up, but the new RoboCup runtime should use `DifferentialDrive` explicitly.
Both drive types use the shared `HardwareControlLock` so only one process owns
the physical base. Unmeasured or unverified geometry remains configuration,
never a hard-coded assumption; formal hardware mode is gated until the
relevant measurements and backend are verified.

`DifferentialDrive` applies limits in `v/omega` space, rate-limits from the
previously applied twist using monotonic time, then converts and proportionally
scales wheel targets. Its watchdog stops the backend when no command arrives
within `command_timeout_s`; the existing C10B sender retains its independent
watchdog as a lower-level fallback. Explicit `stop()` bypasses acceleration
ramping and clears limiter state.
