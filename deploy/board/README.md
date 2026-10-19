# Board-level setup bundle (ROCK 5A)

These files are the OS-level pieces the car needs that used to exist only on the
car's SD card. The application itself is already in this repository; this
directory makes a fresh board reproducible.

| File | Installed to | Purpose |
|---|---|---|
| `install_board_setup.sh` | run in place | groups, PWM0 helper, udev, T265 unit, device-node check |
| `rock5a-pwm0-permissions` | `/usr/local/sbin/` | export PWM0 and grant the `pwm` group |
| `rock5a-pwm0-permissions.service` | `/etc/systemd/system/` | run the helper at boot |

## Usage

```sh
sudo sh deploy/board/install_board_setup.sh
sudo sh deploy/install_car_services.sh
```

If the script reports missing PWM/UART nodes:

```sh
sudo sh deploy/board/install_board_setup.sh --enable-overlays
sudo reboot
sudo sh deploy/board/install_board_setup.sh     # verify
```

`--enable-overlays` edits `/boot/extlinux/extlinux.conf` through
`tools/board_overlay.py` (which backs up the file first). It does not reboot for
you. A board whose base device tree already routes those pins passes the node
check without changing the overlay line — the live `/sys/class/pwm` and
`/dev/ttyS*` view is authoritative, not the boot line.

The script is idempotent and does not install the mission services, start a
motor, or touch `configs/robocup_diffdrive.toml`. Full pin/service map and the
new-board checklist: [`docs/BOARD_PORTING.md`](../../docs/BOARD_PORTING.md).

## Why these files were separate

Radxa's image creates `pwm`, `gpio`, `i2c` and `spidev` groups, but PWM0 has no
udev rule, so it needs a oneshot unit. The RealSense rules live in `tools/`; the
T265 boot unit has its own installer under `deploy/`; this bundle just wires them
together in the right order for a new board.
