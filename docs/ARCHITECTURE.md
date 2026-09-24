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

The existing Ackermann entry points and production behavior remain the
baseline during this structural step. Differential components are introduced
in later migration steps and should be selected explicitly by the new RoboCup
runtime. Unmeasured or unverified geometry remains configuration, never a
hard-coded assumption; formal hardware mode is gated until the relevant
measurements and backend are verified.
