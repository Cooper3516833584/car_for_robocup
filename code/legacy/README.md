# Removed legacy application stack

The old Ackermann task, steering, navigation, v1 profile and serial-screen
launcher implementations have been removed at the user's request.
Recover historical source through Git history if needed.

The differential production entry remains `code/main_robocup.py`.
The C10B firmware compatibility backend and shared motor/sensor/safety drivers
remain in use. Optional ROS2 packages retain sensor and map utilities only.
See `docs/LEGACY_CLEANUP.md` for the inventory and deployment transition.