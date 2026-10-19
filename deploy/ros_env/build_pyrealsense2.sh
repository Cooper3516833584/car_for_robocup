#!/bin/bash
# Build and install Intel RealSense SDK 2.53.1 with the Python 3.11 bindings.
#
# The car board has librealsense v2.53.1 installed under /usr/local and the
# pyrealsense2 module under /usr/lib/python3/dist-packages.  Rebuilding it needs
# the local patch in this directory: pybind11 2.6.2 cannot compile against
# Python 3.11, and two wrapper translation units need <iostream>.
#
# Run as root:
#   sudo bash deploy/ros_env/build_pyrealsense2.sh
#
# Takes roughly 20-40 minutes on a ROCK 5A.  Set LIBSRC to build in another
# checkout, and JOBS to change the compiler parallelism.
set -euo pipefail

DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
SRC=${LIBSRC:-/home/radxa/librealsense}
JOBS=${JOBS:-$(nproc)}
VERSION=v2.53.1

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root: sudo bash $0" >&2
    exit 1
fi
if ! command -v cmake >/dev/null; then
    echo "installing build dependencies"
    apt-get update
    apt-get install -y --no-install-recommends \
        build-essential cmake git pkg-config libssl-dev libusb-1.0-0-dev \
        libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
        python3-dev bzip2 ca-certificates curl
fi

if [ ! -d "$SRC/.git" ]; then
    git clone --branch "$VERSION" --depth 1 \
        https://github.com/IntelRealSense/librealsense.git "$SRC"
fi
echo "source: $SRC ($(git -C "$SRC" describe --tags 2>/dev/null || echo unknown))"

if ! grep -q 'v2.10.4' "$SRC/third-party/pybind11/CMakeLists.txt"; then
    echo "applying Python 3.11 patch"
    git -C "$SRC" apply "$DIR/librealsense-v2.53.1-py311.patch"
fi

# FORCE_RSUSB_BACKEND avoids depending on a patched kernel/uvc driver on ARM.
# If the T265 is not detected afterwards, rebuild with -DFORCE_RSUSB_BACKEND=OFF
# and compare with `rs-enumerate-devices`.
cmake -S "$SRC" -B "$SRC/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_PYTHON_BINDINGS=ON \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
cmake --build "$SRC/build" -j "$JOBS"
cmake --install "$SRC/build"
ldconfig

/usr/bin/python3 -c 'import pyrealsense2 as rs; print("pyrealsense2", rs.__version__)'
echo
echo "now rerun deploy/ros_env/create_ros_env.sh as radxa to link python_ext/"
