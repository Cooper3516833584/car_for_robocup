#!/bin/sh
# Install the T265 boot bring-up / self-heal unit on the car (ROCK 5A).
#
# Run as root.  The deploy directory is substituted into the service so the
# unit always points at the checked-out repository it was installed from.
set -eu

REPO_ROOT=${1:-"$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"}
DEPLOY_DIR=$(CDPATH= cd -- "$REPO_ROOT/deploy" && pwd)
PROGRAM=$DEPLOY_DIR/t265_boot_init.py
SERVICE_TEMPLATE=$DEPLOY_DIR/t265-boot-init.service.in
TIMER_TEMPLATE=$DEPLOY_DIR/t265-boot-init.timer.in
UDEV_TEMPLATE=$DEPLOY_DIR/99-t265-boot-init.rules
SERVICE_TARGET=/etc/systemd/system/t265-boot-init.service
TIMER_TARGET=/etc/systemd/system/t265-boot-init.timer
UDEV_TARGET=/etc/udev/rules.d/99-t265-boot-init.rules

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root" >&2
    exit 1
fi

for required in "$PROGRAM" "$SERVICE_TEMPLATE" "$TIMER_TEMPLATE" "$UDEV_TEMPLATE"; do
    if [ ! -f "$required" ]; then
        echo "missing deployment input: $required" >&2
        exit 1
    fi
done

# 先做语法检查，避免装出一个起不来的服务
/usr/bin/python3 -c 'import ast,sys; ast.parse(open(sys.argv[1], encoding="utf-8").read())' "$PROGRAM"

if ! /usr/bin/python3 -c 'import pyrealsense2' >/dev/null 2>&1; then
    echo "warning: pyrealsense2 is not importable, pose verification will fail" >&2
fi

sed "s#@DEPLOY_DIR@#$DEPLOY_DIR#g" "$SERVICE_TEMPLATE" > "$SERVICE_TARGET"
install -m 0644 "$TIMER_TEMPLATE" "$TIMER_TARGET"
install -m 0644 "$UDEV_TEMPLATE" "$UDEV_TARGET"
udevadm control --reload-rules
systemctl daemon-reload
systemctl enable t265-boot-init.service t265-boot-init.timer
echo "installed $SERVICE_TARGET, $TIMER_TARGET and $UDEV_TARGET"
echo "run it now with: systemctl start t265-boot-init.service"
