#!/bin/sh
# Provision the OS-level pieces the RoboCup car needs on a ROCK 5A.
#
# The repository already owns the application: code, tests and the deployment
# units under deploy/.  This script installs the few board-level items that used
# to live only on the car's SD card:
#
#   * membership in the dialout/gpio/pwm/i2c/spidev/video/plugdev groups
#   * /usr/local/sbin/rock5a-pwm0-permissions + its oneshot unit (PWM0, pin 23)
#   * the Intel RealSense udev rules (T265 / D500 permissions)
#   * the T265 boot bring-up unit (delegated to install_t265_boot_init_service.sh)
#   * a read-only check that the PWM and UART device-tree nodes actually exist
#
# It does not edit the runtime profile, install the mission services, start a
# motor or reboot the board.  Run deploy/install_car_services.sh afterwards.
#
# Usage (as root, from anywhere):
#     sudo sh deploy/board/install_board_setup.sh [--enable-overlays] [--user NAME]
#
# --enable-overlays additionally appends the overlays listed below to the
#     /boot/extlinux/extlinux.conf fdtoverlays line with tools/board_overlay.py.
#     A reboot is required afterwards.  Without the flag the script only reports
#     which overlays are missing.
# --user NAME selects the account that runs the car services (default: radxa).
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/../.." && pwd)
UNIT_DIR=/etc/systemd/system
UDEV_DIR=/etc/udev/rules.d
PWM_HELPER=/usr/local/sbin/rock5a-pwm0-permissions
CONF=/boot/extlinux/extlinux.conf
SERVICE_USER=radxa
ENABLE_OVERLAYS=0
REBOOT_REQUIRED=0

# Device-tree nodes the runtime opens.  PWM1/6/7/11/14 and UART4-M2 are already
# routed by the vendor base DT on the 6.1.43-20-rk2312 image; the overlays make
# the same routing explicit and are required if a future image ships them
# disabled.  PWM7 (febd0030) is the camera axis and must never be dropped.
OVERLAYS="rk3588-pwm0-m2 rk3588-pwm1-m2 rk3588-pwm11-m1 rk3588-pwm14-m1 rk3588-pwm6-m0 rk3588-pwm7-m0 rk3588-uart6-m1 rk3588-uart4-m2"
PWM_NODES="fd8b0000.pwm fd8b0010.pwm febe0030.pwm febf0020.pwm febd0020.pwm febd0030.pwm"

usage() {
    sed -n '2,24p' "$0"
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --enable-overlays) ENABLE_OVERLAYS=1; shift ;;
        --user=*) SERVICE_USER=${1#*=}; shift ;;
        --user)
            if [ "$#" -lt 2 ]; then
                echo "--user needs a value, for example --user=radxa" >&2
                exit 2
            fi
            SERVICE_USER=$2
            shift 2
            ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root: sudo sh $0" >&2
    exit 1
fi
if [ ! -f "$REPO_ROOT/code/main_robocup.py" ]; then
    echo "$REPO_ROOT does not look like the car repository" >&2
    exit 1
fi
if ! id "$SERVICE_USER" >/dev/null 2>&1; then
    echo "service user does not exist: $SERVICE_USER" >&2
    exit 1
fi

MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || true)
case "$MODEL" in
    *"ROCK 5A"*) echo "board  : ${MODEL:-unknown}" ;;
    *) echo "warning: model is '${MODEL:-unknown}', not ROCK 5A; the pin map and DT node names below are ROCK 5A-specific" >&2 ;;
esac
echo "repo   : $REPO_ROOT"
echo "user   : $SERVICE_USER"

# 1. Serial, GPIO, PWM and camera permissions are group-based on the Radxa image.
for group in dialout gpio pwm i2c spidev video plugdev; do
    getent group "$group" >/dev/null || groupadd "$group"
done
usermod -aG dialout,gpio,pwm,i2c,spidev,video,plugdev "$SERVICE_USER"
echo "groups : $(id -nG "$SERVICE_USER")"
echo "         (log out and back in for a new group set to apply to existing shells)"

# 2. PWM0 (pin 23) has no udev rule; a oneshot unit grants the pwm group access.
install -d /usr/local/sbin
install -m 0755 "$SCRIPT_DIR/rock5a-pwm0-permissions" "$PWM_HELPER"
install -m 0644 "$SCRIPT_DIR/rock5a-pwm0-permissions.service" \
    "$UNIT_DIR/rock5a-pwm0-permissions.service"
systemctl daemon-reload
systemctl enable rock5a-pwm0-permissions.service
if systemctl start rock5a-pwm0-permissions.service; then
    echo "pwm    : installed $PWM_HELPER and rock5a-pwm0-permissions.service"
else
    echo "warning: rock5a-pwm0-permissions.service failed; PWM0/pin 23 is not available yet" >&2
fi

# 3. RealSense USB nodes (T265, D500) are 0666/plugdev; the T265 rule also pulls
#    in the boot bring-up unit when the camera enumerates.
install -m 0644 "$REPO_ROOT/tools/99-realsense-libusb.rules" \
    "$UDEV_DIR/99-realsense-libusb.rules"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --action=add || true
echo "udev   : installed 99-realsense-libusb.rules"

if [ -f "$REPO_ROOT/deploy/install_t265_boot_init_service.sh" ]; then
    sh "$REPO_ROOT/deploy/install_t265_boot_init_service.sh" "$REPO_ROOT"
fi

# 4. Report the boot overlay line, then verify the nodes the kernel actually
#    exposes.  A board whose base DT already routes these pins passes even when
#    the overlay line is shorter; a board that lacks a node gets the exact
#    commands to add it.
echo
echo "overlays in $CONF:"
ACTIVE=""
if [ -r "$CONF" ]; then
    ACTIVE=$(sed -n 's/^[[:space:]]*fdtoverlays[[:space:]]*//p' "$CONF" | tail -n 1)
fi
if [ -z "$ACTIVE" ]; then
    echo "  (no fdtoverlays line found; this is not an extlinux/U-Boot board)"
else
    echo "  $ACTIVE"
fi

overlay_active() {
    case " $ACTIVE " in
        *"/$1.dtbo "*|*"/$1.dtbo") return 0 ;;
        *) return 1 ;;
    esac
}
MISSING_OVERLAYS=""
for overlay in $OVERLAYS; do
    if overlay_active "$overlay"; then
        echo "  active   $overlay"
    else
        echo "  missing  $overlay"
        MISSING_OVERLAYS="$MISSING_OVERLAYS $overlay"
    fi
done

if [ -n "$MISSING_OVERLAYS" ] && [ "$ENABLE_OVERLAYS" -eq 1 ]; then
    for overlay in $MISSING_OVERLAYS; do
        if python3 "$REPO_ROOT/tools/board_overlay.py" enable "$overlay"; then
            REBOOT_REQUIRED=1
        else
            echo "failed to enable $overlay" >&2
        fi
    done
fi

# 5. The decisive check is the live kernel view, not the boot line.
echo
echo "live device nodes:"
MISSING_NODES=""
for node in $PWM_NODES; do
    found=0
    for link in /sys/class/pwm/pwmchip*/device; do
        [ -e "$link" ] || continue
        case "$(readlink -f "$link")" in
            *"$node"*) found=1 ;;
        esac
    done
    if [ "$found" -eq 1 ]; then
        echo "  ok       pwm $node"
    else
        echo "  MISSING  pwm $node" >&2
        MISSING_NODES="$MISSING_NODES $node"
    fi
done
for pair in "/dev/ttyS4:HC-15 radio (UART4-M2)" "/dev/ttyS6:D500 lidar (UART6-M1)"; do
    device=${pair%%:*}
    label=${pair#*:}
    if [ -c "$device" ]; then
        echo "  ok       $device  $label"
    else
        echo "  MISSING  $device  $label" >&2
        MISSING_NODES="$MISSING_NODES $device"
    fi
done
if [ -c /dev/ttyACM0 ]; then
    echo "  ok       /dev/ttyACM0  C10B motor controller"
else
    echo "  absent   /dev/ttyACM0  C10B (fine if the USB adapter is unplugged)"
fi

if [ ! -f "$REPO_ROOT/configs/robocup_diffdrive.toml" ]; then
    echo "warning: runtime profile is missing: $REPO_ROOT/configs/robocup_diffdrive.toml" >&2
fi

echo
if [ -n "$MISSING_NODES" ] && [ "$ENABLE_OVERLAYS" -eq 0 ]; then
    echo "ACTION REQUIRED: missing devices:$MISSING_NODES"
    echo "Enable the overlays and reboot:"
    echo "  sudo sh $0 --enable-overlays"
    echo "  sudo reboot"
    echo "or edit $CONF by hand with tools/board_overlay.py."
    exit 2
fi
if [ "$REBOOT_REQUIRED" -eq 1 ]; then
    echo "Overlays were added; reboot, then re-run this script to verify: sudo reboot"
    exit 0
fi
if [ -n "$MISSING_NODES" ]; then
    echo "Overlays were enabled but the nodes are still missing; reboot first, then re-run."
    exit 2
fi
echo "board setup OK. Next:"
echo "  sudo sh $REPO_ROOT/deploy/install_car_services.sh"
echo "  sudo systemctl enable --now competition-boot.service"
echo
echo "The runtime profile is $REPO_ROOT/configs/robocup_diffdrive.toml;"
echo "change ports/pins there (see docs/BOARD_PORTING.md)."
