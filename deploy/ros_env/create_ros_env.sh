#!/bin/bash
# Recreate the car's ROS 2 + vision environment under /home/radxa/robocup_ros.
#
# The 4.6 GB environment is deliberately not committed.  This script rebuilds
# the same layout from the pinned files in this directory:
#
#   robocup_ros/
#     bin/micromamba           micromamba 2.9.0
#     root/                    MAMBA_ROOT_PREFIX
#     env/                     conda env: python 3.11 + ROS 2 Humble + slam_toolbox
#     vision_env/              venv --system-site-packages on top of env/
#     python_ext/              pyrealsense2 shim used through PYTHONPATH
#
# Run as the service user (radxa), not root, after build_pyrealsense2.sh:
#
#   bash deploy/ros_env/create_ros_env.sh
#
# Everything is idempotent: existing prefixes are reused, not deleted.
set -euo pipefail

DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROS_ROOT=${ROS_ROOT:-/home/radxa/robocup_ros}
MAMBA_VERSION=${MAMBA_VERSION:-2.9.0}
M="$ROS_ROOT/bin/micromamba"

if [ "$(id -u)" -eq 0 ]; then
    echo "run as the service user (for example radxa), not root" >&2
    exit 1
fi
if [ "$(uname -m)" != "aarch64" ]; then
    echo "this lock file is linux-aarch64 only (ROCK 5A); detected $(uname -m)" >&2
    exit 1
fi

available_gib=$(df -Pk "$(dirname "$ROS_ROOT")" | awk 'NR==2 {printf "%d", $4/1048576}')
if [ "$available_gib" -lt 10 ]; then
    echo "warning: only ${available_gib} GiB free; the environment needs about 6 GiB" >&2
fi

mkdir -p "$ROS_ROOT/bin" "$ROS_ROOT/python_ext"
if [ ! -x "$M" ]; then
    echo "== installing micromamba $MAMBA_VERSION =="
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    if ! curl -fsSL "https://micro.mamba.pm/api/micromamba/linux-aarch64/$MAMBA_VERSION" \
        -o "$tmp/micromamba.tar.bz2"; then
        echo "versioned download failed; falling back to latest" >&2
        curl -fsSL "https://micro.mamba.pm/api/micromamba/linux-aarch64/latest" \
            -o "$tmp/micromamba.tar.bz2"
    fi
    tar -xjf "$tmp/micromamba.tar.bz2" -C "$tmp" bin/micromamba
    install -m 0755 "$tmp/bin/micromamba" "$M"
fi
export MAMBA_ROOT_PREFIX="$ROS_ROOT/root"

if [ ! -x "$ROS_ROOT/env/bin/python" ]; then
    echo "== creating $ROS_ROOT/env from the explicit lock =="
    "$M" create -y -p "$ROS_ROOT/env" -f "$DIR/env-explicit-linux-aarch64.txt"
else
    echo "== reusing existing $ROS_ROOT/env =="
fi

echo "== pip layer in env =="
"$ROS_ROOT/env/bin/pip" install -r "$DIR/env-pip-requirements.txt"

if [ ! -x "$ROS_ROOT/vision_env/bin/python" ]; then
    echo "== creating $ROS_ROOT/vision_env =="
    "$ROS_ROOT/env/bin/python" -m venv --system-site-packages "$ROS_ROOT/vision_env"
else
    echo "== reusing existing $ROS_ROOT/vision_env =="
fi

echo "== vision packages =="
"$ROS_ROOT/vision_env/bin/pip" install -r "$DIR/vision-requirements.txt"

# The conda env cannot see Debian's dist-packages, so expose the system
# pyrealsense2 module through the PYTHONPATH entry the service units use.
REALSENSE_SO=$(find /usr/local/lib/python3.11/dist-packages /usr/lib/python3/dist-packages \
    -maxdepth 1 -name 'pyrealsense2*.so' 2>/dev/null | head -n 1 || true)
if [ -n "$REALSENSE_SO" ]; then
    ln -sfn "$REALSENSE_SO" "$ROS_ROOT/python_ext/$(basename "$REALSENSE_SO")"
    echo "pyrealsense2: $ROS_ROOT/python_ext/$(basename "$REALSENSE_SO") -> $REALSENSE_SO"
else
    echo "warning: pyrealsense2 is not installed; run deploy/ros_env/build_pyrealsense2.sh as root" >&2
fi

echo "== import check =="
MAMBA_ROOT_PREFIX="$MAMBA_ROOT_PREFIX" \
PYTHONPATH="$ROS_ROOT/python_ext:${PYTHONPATH:-}" \
"$M" run -p "$ROS_ROOT/env" "$ROS_ROOT/vision_env/bin/python" - <<'PY'
import rclpy, cv2, pyrealsense2, rapidocr, ultralytics, onnxruntime
print("imports ok: rclpy cv2 pyrealsense2 rapidocr ultralytics onnxruntime")
PY

cat <<EOF

environment ready: $ROS_ROOT

next:
  sudo sh deploy/install_car_services.sh
  sudo systemctl enable --now competition-boot.service

If pyrealsense2 was missing, run as root first:
  sudo bash deploy/ros_env/build_pyrealsense2.sh
EOF
