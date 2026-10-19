#!/bin/sh
# Install the car's project systemd units with paths pointing at this checkout.
#
# The shared alarm and battery units live in code/test/*.service and are the
# single source of truth. Camera home and competition units live in deploy/.
# Older installs hard-coded a different checkout (car_for_ECDC); a stale path
# silently breaks every service, so this script rewrites the paths instead of
# keeping a second copy of each unit in sync.
#
# Run as root, from anywhere:
#     sudo deploy/install_car_services.sh [repo_root]
#
# The camera servo home service is enabled because the boot-time camera position
# must be deterministic. Alarm/battery units remain opt-in. Competition and SLAM
# units are installed but remain operator-enabled. The T265 self-heal unit is
# installed/enabled separately by deploy/install_t265_boot_init_service.sh.
set -eu

REPO_ROOT=${1:-"$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"}
REPO_ROOT=$(CDPATH= cd -- "$REPO_ROOT" && pwd)
CAR_CODE=$REPO_ROOT/code
ROS_ROOT=${REPO_ROOT%/*}/robocup_ros
COMPETITION_PYTHON=$ROS_ROOT/vision_env/bin/python
MICROMAMBA=$ROS_ROOT/bin/micromamba
UNIT_DIR=/etc/systemd/system

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root" >&2
    exit 1
fi

# A checkout may lack the executable bit (git does not track file modes on
# Windows); `sh <script>` always works.
if [ ! -x "$0" ]; then
    echo "note: $0 is not executable here, run it as: sh $0" >&2
fi
if [ ! -x "$COMPETITION_PYTHON" ]; then
    echo "competition Python environment is missing: $COMPETITION_PYTHON" >&2
    exit 1
fi
if [ ! -x "$MICROMAMBA" ] || [ ! -x "$ROS_ROOT/env/bin/ros2" ]; then
    echo "RoboStack micromamba or ros2 executable is missing under $ROS_ROOT" >&2
    exit 1
fi
if [ ! -c /dev/ttyS4 ]; then
    echo "HC-15 device /dev/ttyS4 is not present; competition listener path is unverified" >&2
    exit 1
fi
if ! MAMBA_ROOT_PREFIX="$ROS_ROOT/root" \
    PYTHONPATH="$ROS_ROOT/python_ext:$CAR_CODE${PYTHONPATH:+:$PYTHONPATH}" \
    "$MICROMAMBA" run -p "$ROS_ROOT/env" "$COMPETITION_PYTHON" \
    -c 'import competition_boot, main_robocup; import rclpy, pyrealsense2, cv2, rapidocr, onnxruntime, ultralytics'; then
    echo "competition environment cannot import ROS, T265, OCR or YOLO dependencies" >&2
    exit 1
fi

install_one() {
    template=$1
    unit=$2
    if [ ! -f "$template" ]; then
        echo "missing unit template: $template" >&2
        exit 1
    fi
    # Rewrite the legacy checkout path (if any) to this repository.
    rendered=$(mktemp)
    sed -e "s#/home/radxa/car_for_ECDC/code#${CAR_CODE}#g" \
        -e "s#/home/radxa/car_for_ECDC#${REPO_ROOT}#g" \
        -e "s#/home/radxa/robocup_ros#${ROS_ROOT}#g" \
        -e "s#/home/radxa/car#${REPO_ROOT}#g" \
        "$template" > "$rendered"
    if ! cmp -s "$rendered" "$UNIT_DIR/$unit" 2>/dev/null; then
        install -m 0644 "$rendered" "$UNIT_DIR/$unit"
        echo "installed $UNIT_DIR/$unit -> $CAR_CODE"
    else
        echo "already current: $UNIT_DIR/$unit"
    fi
    rm -f "$rendered"
}

install_one "$CAR_CODE/test/sound-light-alarm.service"        sound-light-alarm.service
install_one "$CAR_CODE/test/battery-voltage-monitor.service"  battery-voltage-monitor.service
install_one "$REPO_ROOT/deploy/servo-pwm7-home.service"        servo-pwm7-home.service
install_one "$REPO_ROOT/deploy/robocup-slam.service"            robocup-slam.service
install_one "$REPO_ROOT/deploy/competition-boot.service"        competition-boot.service

SYSTEMCTL_PATH=$(command -v systemctl)
SUDOERS_TARGET=/etc/sudoers.d/robocup-slam-restart
SUDOERS_RENDERED=$(mktemp)
sed "s|@SYSTEMCTL_PATH@|$SYSTEMCTL_PATH|g" \
    "$REPO_ROOT/deploy/robocup-slam-restart.sudoers.in" > "$SUDOERS_RENDERED"
chmod 0440 "$SUDOERS_RENDERED"
if visudo -cf "$SUDOERS_RENDERED" >/dev/null; then
    if [ -e "$SUDOERS_TARGET" ] && ! cmp -s "$SUDOERS_RENDERED" "$SUDOERS_TARGET"; then
        if ! sudo -n -l -U radxa 2>/dev/null | grep -Fq "NOPASSWD: $SYSTEMCTL_PATH restart robocup-slam.service"; then
            echo "refusing to overwrite existing $SUDOERS_TARGET; inspect it and sudo -n -l -U radxa" >&2
            rm -f "$SUDOERS_RENDERED"
            exit 1
        fi
        echo "existing authorization already grants the exact SLAM restart command"
        rm -f "$SUDOERS_RENDERED"
    elif [ ! -e "$SUDOERS_TARGET" ]; then
        if sudo -n -l -U radxa 2>/dev/null | grep -Fq "NOPASSWD: $SYSTEMCTL_PATH restart robocup-slam.service"; then
            rm -f "$SUDOERS_RENDERED"
            echo "existing authorization already grants the exact SLAM restart command"
        else
            install -m 0440 "$SUDOERS_RENDERED" "$SUDOERS_TARGET"
            rm -f "$SUDOERS_RENDERED"
            visudo -cf "$SUDOERS_TARGET" >/dev/null
            echo "installed $SUDOERS_TARGET for $SYSTEMCTL_PATH"
        fi
    else
        rm -f "$SUDOERS_RENDERED"
        echo "already current: $SUDOERS_TARGET"
    fi
else
    rm -f "$SUDOERS_RENDERED"
    echo "invalid rendered sudoers policy" >&2
    exit 1
fi

systemctl daemon-reload
systemd-analyze verify \
    "$UNIT_DIR/robocup-slam.service" \
    "$UNIT_DIR/competition-boot.service"
systemctl enable --now servo-pwm7-home.service
echo
echo "installed units:"
for unit in sound-light-alarm battery-voltage-monitor robocup-slam competition-boot; do
    printf '  %-32s enabled=%s active=%s\n' \
        "$unit" \
        "$(systemctl is-enabled "$unit.service" 2>&1 || true)" \
        "$(systemctl is-active "$unit.service" 2>&1 || true)"
done
printf '  %-32s enabled=%s active=%s\n' \
    servo-pwm7-home \
    "$(systemctl is-enabled servo-pwm7-home.service 2>&1 || true)" \
    "$(systemctl is-active servo-pwm7-home.service 2>&1 || true)"
echo
echo "enable one with: systemctl enable --now <unit>.service"
echo "robocup-slam.service and competition-boot.service were installed, not enabled or started"
echo
echo "board-level groups/udev/PWM0 helper (once per board):"
echo "  sudo sh $REPO_ROOT/deploy/board/install_board_setup.sh"
