#!/bin/sh
# Install the car's project systemd units with paths pointing at this checkout.
#
# The units live in the repository (code/test/*.service and
# code/mission-screen-launcher.service) and are the single source of truth.
# Older installs hard-coded a different checkout (car_for_ECDC); a stale path
# silently breaks every service, so this script rewrites the paths instead of
# keeping a second copy of each unit in sync.
#
# Run as root, from anywhere:
#     sudo deploy/install_car_services.sh [repo_root]
#
# Nothing is enabled: the timer-disabled units stay opt-in.  Only the T265
# self-heal unit is installed/enabled separately by
# deploy/install_t265_boot_init_service.sh.
set -eu

REPO_ROOT=${1:-"$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"}
CAR_CODE=$REPO_ROOT/code
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

install_one() {
    template=$1
    unit=$2
    if [ ! -f "$template" ]; then
        echo "missing unit template: $template" >&2
        exit 1
    fi
    # Rewrite the legacy checkout path (if any) to this repository.
    sed -e "s#/home/radxa/car_for_ECDC/code#${CAR_CODE}#g" \
        -e "s#/home/radxa/car/code#${CAR_CODE}#g" \
        "$template" > "$UNIT_DIR/$unit"
    chmod 0644 "$UNIT_DIR/$unit"
    echo "installed $UNIT_DIR/$unit -> $CAR_CODE"
}

install_one "$CAR_CODE/test/sound-light-alarm.service"        sound-light-alarm.service
install_one "$CAR_CODE/test/battery-voltage-monitor.service"  battery-voltage-monitor.service
install_one "$CAR_CODE/test/rock5a-pwm0-permissions.service"  rock5a-pwm0-permissions.service
install_one "$CAR_CODE/mission-screen-launcher.service"       mission-screen-launcher.service

systemctl daemon-reload
echo
echo "installed units (none enabled by this script):"
for unit in sound-light-alarm battery-voltage-monitor rock5a-pwm0-permissions mission-screen-launcher; do
    printf '  %-32s enabled=%s active=%s\n' \
        "$unit" \
        "$(systemctl is-enabled "$unit.service" 2>&1 || true)" \
        "$(systemctl is-active "$unit.service" 2>&1 || true)"
done
echo
echo "enable one with: systemctl enable --now <unit>.service"
