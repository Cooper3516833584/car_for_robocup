# ROS 2 + vision environment

The car runs from `/home/radxa/robocup_ros`, a 4.6 GB tree that is **not**
committed. This directory pins it so a new board can rebuild the same layout.

```text
robocup_ros/
  bin/micromamba        micromamba 2.9.0
  root/                 MAMBA_ROOT_PREFIX
  env/                  conda env, python 3.11.15, ROS 2 Humble, slam_toolbox
  vision_env/           venv --system-site-packages on top of env/
  python_ext/           pyrealsense2 module exposed through PYTHONPATH
```

| File | Content |
|---|---|
| `env-explicit-linux-aarch64.txt` | exact 574-package conda set (`@EXPLICIT`) |
| `env-pip-requirements.txt` | RapidOCR/ONNX/OpenCV/Shapely layer in `env/` |
| `vision-requirements.txt` | torch/torchvision/ultralytics layer in `vision_env/` |
| `build_pyrealsense2.sh` | librealsense v2.53.1 + Python 3.11 patch |
| `librealsense-v2.53.1-py311.patch` | pybind11 2.10.4 + `<iostream>` fixes |
| `create_ros_env.sh` | idempotent rebuild/verify of the whole tree |

## Rebuild order

```sh
sudo bash deploy/ros_env/build_pyrealsense2.sh      # ~20-40 min, root
bash deploy/ros_env/create_ros_env.sh               # ~6 GiB, as radxa
```

Then verify:

```sh
sudo sh deploy/board/install_board_setup.sh
sudo sh deploy/install_car_services.sh
```

`install_car_services.sh` imports `competition_boot`, `main_robocup`, `rclpy`,
`pyrealsense2`, `cv2`, `rapidocr`, `onnxruntime` and `ultralytics` from the
environment and refuses to install the units if any import fails.

## Versions that matter

- micromamba 2.9.0 (`competition-boot.service` isolates its cache per start, see
  `docs/MAMBA_LISTENER_STARTUP.md`).
- Python 3.11.15 in both environments; the conda lock is aarch64-only.
- ROS 2 Humble from `robostack-humble` plus `ros-humble-slam-toolbox`.
- librealsense `v2.53.1`, built with `-DFORCE_RSUSB_BACKEND=ON` and Python
  bindings against `/usr/bin/python3`.
- torch/torchvision CPU wheels from `https://download.pytorch.org/whl/cpu`.

## Why not commit the environment

4.6 GB of binaries, platform-specific, and reproducible from the four files
above. The explicit lock is the strongest guarantee: it names every conda URL
rather than resolving floating dependencies. If a URL disappears upstream,
recreate `env/` with the top-level specs recorded in the lock header and treat
the version drift as a deliberate change.
