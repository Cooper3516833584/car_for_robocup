# Intel RealSense T265 setup

## Coordinates and adapter contract

The T265 reports pose relative to its tracking origin. Its native device axes
are `+X` toward the right imager, `+Y` upward, and `+Z` toward the back of the
device; the tracking world keeps `+Y` aligned with gravity. The driver
preserves translation and quaternion in that native convention. The pose
adapter converts native coordinates once, applies the configured
`base_link -> t265_link` mount transform, rebases the first accepted body pose
to identity, then emits canonical SI `Pose2D` updates.

The adapter discards low-confidence, stale/future, non-finite, invalid
quaternion, and excessive-jump samples. Only updates with a valid quality flag
should reach pose fusion. T265 provides relative tracking; it does not assign
the car's competition-field origin. D500/map alignment provides that anchor.

## Development and deployment

- Windows development and automated tests use `FakeT265PoseSource`; importing
  T265 code does not require `pyrealsense2`.
- On ROCK 5A/ARM, first verify that the installed librealsense library and
  Python binding support the target architecture and the attached T265. A
  successful install on a Windows development machine does not establish ARM
  compatibility.
- If T265 is disabled or unavailable, the runtime can omit the T265 source and
  operate with D500 localization in a degraded mode. No fusion import should
  import or start a T265 device implicitly.
- Probe without connecting the motors using `py -3 tools/t265_probe.py`; stop
  with Ctrl+C. The probe reports pose, confidence and timestamps only.

The axis convention and the requirement to transform the T265 pose into the
target robot frame follow the [RealSense visual SLAM and T265 guide](https://dev.realsenseai.com/docs/intel-realsensetm-visual-slam-and-the-t265-tracking-camera/)
and the [T265 tracking-module datasheet](https://dev.realsenseai.com/download/42170/).
