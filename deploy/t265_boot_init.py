#!/usr/bin/env python3
"""Boot-time bring-up and health check for the on-board RealSense T265.

Verified failure mode on this car (ROCK 5A, librealsense 2.53.1, T265
``230322111433`` / fw ``0.2.0.951``): the device enumerates correctly as
``8087:0b37``, but the *first* ``rs.pipeline.start()`` after a cold boot can
fail with ``RuntimeError: No device connected``.  Re-reading the device list
and starting the pipeline against the same context then succeeds.  There is no
automatic recovery, so a mission that starts too early loses odometry.

This program replaces the manual retry:

* If the device is absent nothing happens: it waits for the udev ``add`` event
  or the next periodic run, so plugging the camera in later is enough.
* If the device is present the pose stream is opened with bounded retries,
  recreating the context between attempts, which is what recovers the state.
* If the device never yields a pose stream and nothing else holds it, the USB
  port is reset to force re-enumeration, then verification is retried.
* Once the T265 is verified the process exits.

The process is one-shot: it never stays resident and never holds the camera, so
a T265 that a mission is using is left untouched.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import sys
import time

STATE_DIR = Path("/var/log/t265-boot-init")
LOG_PATH = STATE_DIR / "run.log"
STATE_PATH = STATE_DIR / "state.json"
LOCK_PATH = Path("/run/t265-boot-init/lock")

LOG_MAX_BYTES = 1024 * 1024
LOG_BACKUPS = 2

T265_IDS = ("8087", "0b37")
VPU_IDS = ("03e7", "2150")
USB_SYSFS = Path("/sys/bus/usb/devices")
USB_DEVS = Path("/dev/bus/usb")

POLL_SECONDS = 2.0
REENUMERATE_TIMEOUT = 12.0
RESET_COOLDOWN_SECONDS = 60.0

POSE_TIMEOUT_MS = 5000
POSE_MAX_SECONDS = 15.0
POSE_MIN_FRAMES = 15
POSE_MIN_CONFIDENT = 8
CONFIDENT_LEVEL = 2  # rs.tracker_confidence MEDIUM
QUATERNION_TOLERANCE = 0.02

# How many times to rebuild the context before falling back to a port reset.
# The car's cold-boot race needs one retry; three gives margin.
POSE_ATTEMPTS = 3
# Opening through librealsense is what actually boots the firmware; a plain USB
# reset only resets the port in place.
RESET_METHODS = ("librealsense_kick", "usbdevfs_reset")
USBDEVFS_RESET = 0x5514  # _IO('U', 20)


def _rotate_log() -> None:
    try:
        if LOG_PATH.stat().st_size <= LOG_MAX_BYTES:
            return
    except OSError:
        return
    for index in range(LOG_BACKUPS, 1, -1):
        older = LOG_PATH.with_suffix(f".log.{index}")
        newer = LOG_PATH.with_suffix(f".log.{index - 1}")
        if newer.exists():
            older.unlink(missing_ok=True)
            newer.rename(older)
    LOG_PATH.with_suffix(".log.1").unlink(missing_ok=True)
    LOG_PATH.rename(LOG_PATH.with_suffix(".log.1"))


def log(message: str) -> None:
    line = "%s %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), message)
    try:
        _rotate_log()
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        with LOG_PATH.open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")
    except OSError:
        pass


def log_best_effort(message: str) -> None:
    """Write to the file log and to stdout (which goes to the journal)."""

    log(message)
    print(message, flush=True)


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _boot_id() -> str:
    return _read_text(Path("/proc/sys/kernel/random/boot_id"))


def read_state() -> dict:
    try:
        return json.loads(STATE_PATH.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def write_state(**fields) -> None:
    try:
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        STATE_PATH.write_text(
            json.dumps(fields, ensure_ascii=False, indent=1) + "\n", encoding="utf-8"
        )
    except OSError as exc:
        log("状态文件写入失败: %s" % exc)


def find_devices() -> list[dict]:
    """Return every sysfs USB device whose VID:PID is the T265 or its VPU ROM."""

    found = []
    try:
        entries = sorted(USB_SYSFS.iterdir())
    except OSError:
        return found
    for entry in entries:
        vendor = _read_text(entry / "idVendor")
        product = _read_text(entry / "idProduct")
        if not vendor or not product:
            continue
        ids = (vendor, product)
        if ids not in (T265_IDS, VPU_IDS):
            continue
        kind = "t265" if ids == T265_IDS else "vpu"
        found.append(
            {
                "kind": kind,
                "name": entry.name,
                "path": entry,
                "ids": "%s:%s" % ids,
                "serial": _read_text(entry / "serial"),
                "busnum": _read_text(entry / "busnum"),
                "devnum": _read_text(entry / "devnum"),
            }
        )
    return found


def find_device() -> tuple[str, dict | None]:
    """Collapse the device list into one verdict, preferring the booted T265."""

    devices = find_devices()
    for device in devices:
        if device["kind"] == "t265":
            return "t265", device
    if devices:
        return "vpu", devices[0]
    return "absent", None


def device_node(device: dict) -> Path | None:
    try:
        return USB_DEVS / ("%03d" % int(device["busnum"])) / ("%03d" % int(device["devnum"]))
    except (KeyError, TypeError, ValueError):
        return None


def holders(device: dict) -> list[int]:
    """PIDs that currently have the USB device node open."""

    node = device_node(device)
    if node is None:
        return []
    target = str(node)
    pids = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        fd_dir = "/proc/%s/fd" % name
        try:
            descriptors = os.listdir(fd_dir)
        except OSError:
            continue
        for descriptor in descriptors:
            try:
                if os.readlink("%s/%s" % (fd_dir, descriptor)) == target:
                    pids.append(int(name))
                    break
            except OSError:
                continue
    return pids


def _reset_usbdevfs(device: dict) -> None:
    node = device_node(device)
    if node is None or not node.exists():
        raise RuntimeError("没有可用的设备节点: %s" % node)
    descriptor = os.open(str(node), os.O_RDWR | os.O_NONBLOCK)
    try:
        fcntl.ioctl(descriptor, USBDEVFS_RESET, 0)
    finally:
        os.close(descriptor)


_KICK_SOURCE = """
import pyrealsense2 as rs

ctx = rs.context()
devices = list(ctx.devices)
print("context devices: %d" % len(devices))
for device in devices:
    try:
        print("  %s %s" % (device.get_info(rs.camera_info.name),
                           device.get_info(rs.camera_info.serial_number)))
    except Exception as exc:
        print("  info error: %s" % exc)
"""


def _kick_librealsense(device: dict) -> None:
    """Open the device through librealsense, which makes the firmware boot.

    This has to happen in a separate process: a live ``rs.context()`` keeps the
    device claimed, and the pose check afterwards needs to open it again.
    """

    import subprocess

    result = subprocess.run(
        [sys.executable, "-c", _KICK_SOURCE],
        capture_output=True,
        text=True,
        timeout=30,
    )
    detail = (result.stdout + result.stderr).strip().replace("\n", " | ")
    log("  librealsense 引导（%s）: %s" % (device["ids"], detail[:400] or "(无输出)"))


_RESETS = {
    "librealsense_kick": _kick_librealsense,
    "usbdevfs_reset": _reset_usbdevfs,
}


def wait_for_t265(timeout: float) -> tuple[str, dict | None]:
    deadline = time.monotonic() + timeout
    kind, device = find_device()
    while kind != "t265" and time.monotonic() < deadline:
        time.sleep(POLL_SECONDS)
        kind, device = find_device()
    return kind, device


def _verify_pose_once() -> tuple[str, str]:
    """Open one pose stream and check it looks like a working tracker.

    Returns ``(verdict, detail)`` where verdict is ``"ok"``, ``"busy"`` or
    ``"failed"``.  The pipeline is always stopped before returning.
    """

    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        return "failed", "无法导入 pyrealsense2: %s" % exc

    import math

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.pose)
    config.enable_stream(rs.stream.accel)
    try:
        pipeline.start(config)
    except Exception as exc:  # noqa: BLE001 - realsense raises RuntimeError subclasses
        text = str(exc)
        if "No device connected" in text or "busy" in text.lower():
            return "busy", text
        return "failed", "无法启动 pose 流: %s" % text

    frames = 0
    confidence = []
    norms = []
    accel = []
    started = time.monotonic()
    try:
        while time.monotonic() - started < POSE_MAX_SECONDS:
            try:
                bundle = pipeline.wait_for_frames(POSE_TIMEOUT_MS)
            except Exception as exc:  # noqa: BLE001
                return "failed", "等待帧失败: %s" % exc
            for frame in bundle:
                stream = frame.get_profile().stream_type()
                if stream == rs.stream.pose:
                    data = frame.as_pose_frame().get_pose_data()
                    frames += 1
                    confidence.append(data.tracker_confidence)
                    rotation = data.rotation
                    norms.append(
                        math.sqrt(
                            rotation.x ** 2
                            + rotation.y ** 2
                            + rotation.z ** 2
                            + rotation.w ** 2
                        )
                    )
                elif stream == rs.stream.accel:
                    motion = frame.as_motion_frame().get_motion_data()
                    accel.append(math.sqrt(motion.x ** 2 + motion.y ** 2 + motion.z ** 2))
            if frames >= 2 * POSE_MIN_FRAMES and confidence[-1] >= CONFIDENT_LEVEL:
                break
    finally:
        try:
            pipeline.stop()
        except Exception:  # noqa: BLE001
            pass

    if frames < POSE_MIN_FRAMES:
        return "failed", "只取到 %d 帧位姿（需要 %d）" % (frames, POSE_MIN_FRAMES)
    tail = confidence[-10:]
    if sum(1 for value in tail if value >= CONFIDENT_LEVEL) < POSE_MIN_CONFIDENT:
        return "failed", "最后 10 帧置信度不足: %s" % tail
    worst = max(abs(norm - 1.0) for norm in norms)
    if worst > QUATERNION_TOLERANCE:
        return "failed", "四元数模长偏差 %.4f 超限" % worst
    detail = "%d 帧, 置信度 %s, 模长偏差 %.4f" % (frames, tail, worst)
    if accel:
        detail += ", |a|≈%.2f m/s^2" % (sum(accel) / len(accel))
    return "ok", detail


def verify_pose() -> tuple[str, str]:
    """Verify the pose stream, retrying the context rebuild that fixes cold boot.

    A first ``pipeline.start()`` returning ``No device connected`` immediately
    after boot is the measured failure; a fresh context fixes it.  Only if every
    attempt fails is the device treated as unhealthy.
    """

    last = ("failed", "尚未尝试")
    for attempt in range(1, POSE_ATTEMPTS + 1):
        verdict, detail = _verify_pose_once()
        if verdict == "ok":
            if attempt > 1:
                detail += "（第 %d 次尝试成功）" % attempt
            return "ok", detail
        last = (verdict, detail)
        if verdict == "busy":
            return last
        log("  位姿验证第 %d/%d 次失败: %s" % (attempt, POSE_ATTEMPTS, detail))
        if attempt < POSE_ATTEMPTS:
            time.sleep(POLL_SECONDS)
    return last


def cooldown_active(state: dict) -> bool:
    last = state.get("reset_cycle_at")
    if not isinstance(last, (int, float)):
        return False
    return (time.time() - last) < RESET_COOLDOWN_SECONDS


def _reset_cycle() -> tuple[str, dict | None]:
    """Try each reset method until the device re-enumerates as a T265."""

    kind, device = find_device()
    for method in RESET_METHODS:
        kind, device = find_device()
        if kind == "t265":
            break
        if kind == "absent":
            log("设备在复位过程中消失，停止")
            break
        used_by = holders(device)
        if used_by:
            log("设备被 PID %s 占用，跳过复位" % used_by)
            return kind, device
        log("尝试复位方式 %s（设备 %s, %s）" % (method, device["name"], device["ids"]))
        try:
            _RESETS[method](device)
        except Exception as exc:  # noqa: BLE001 - report and try the next method
            log("  复位方式 %s 失败: %s" % (method, exc))
            continue
        kind, device = wait_for_t265(REENUMERATE_TIMEOUT)
        log("  复位后状态=%s" % kind)
        if kind == "t265":
            log("  复位方式 %s 生效，已识别为 8087:0b37" % method)
            break
    return kind, device


def run(wait_seconds: float) -> int:
    if wait_seconds > 0:
        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline and find_device()[0] == "absent":
            time.sleep(POLL_SECONDS)

    kind, device = find_device()
    log_best_effort(
        "启动检查: 设备状态=%s%s" % (kind, "" if device is None else " (%s)" % device["ids"])
    )

    if kind == "absent":
        log("T265 未插入；等待 udev 插入事件或下一次周期检查")
        write_state(last="absent", at=time.time())
        return 0

    if kind == "vpu":
        state = read_state()
        if cooldown_active(state):
            log("仍是 VPU 状态，距上次复位不足 %.0f 秒，跳过本次" % RESET_COOLDOWN_SECONDS)
            return 0
        kind, device = _reset_cycle()
        write_state(last="reset_cycle", reset_cycle_at=time.time(), at=time.time())
        if kind != "t265":
            log(
                "%s 都未能把设备带成 T265（当前 %s），%.0f 秒后重试；"
                "若持续如此需要人工重新插拔，或检查线缆与 USB 端口"
                % ("、".join(RESET_METHODS), kind, RESET_COOLDOWN_SECONDS)
            )
            return 1

    if kind != "t265":
        log("设备状态=%s，本次不处理" % kind)
        return 0

    assert device is not None
    boot = _boot_id()
    state = read_state()
    if (
        state.get("verified")
        and state.get("serial") == device["serial"]
        and state.get("boot_id") == boot
    ):
        log("T265 %s 已在本 boot 内验证通过，无需操作" % device["serial"])
        return 0

    verdict, detail = verify_pose()

    if verdict == "busy":
        # Either a mission holds the camera (fine) or the cold-boot race hit
        # every attempt.  Distinguish by checking for a real holder, then try
        # one port reset so the next timer tick can succeed.
        used_by = holders(device)
        if used_by:
            log("T265 已被 PID %s 占用，视为可用: %s" % (used_by, detail))
            write_state(
                last="t265",
                serial=device["serial"],
                boot_id=boot,
                verified=True,
                via="in-use",
                at=time.time(),
            )
            return 0
        state = read_state()
        if cooldown_active(state):
            log("位姿流不可用且距上次复位不足 %.0f 秒，跳过本次" % RESET_COOLDOWN_SECONDS)
            return 0
        log_best_effort("T265 %s 位姿流不可用且无人占用，执行 USB 复位: %s" % (device["serial"], detail))
        _reset_cycle()
        write_state(last="t265", reset_cycle_at=time.time(), verified=False, at=time.time())
        return 1

    if verdict == "ok":
        log_best_effort("T265 %s 位姿验证通过: %s" % (device["serial"], detail))
        write_state(
            last="t265",
            serial=device["serial"],
            boot_id=boot,
            verified=True,
            via="pose",
            detail=detail,
            at=time.time(),
        )
        return 0

    # Verification failed although the device is enumerated: reset the port so
    # the firmware gets another boot, respecting the cooldown.
    state = read_state()
    if cooldown_active(state):
        log("位姿验证失败且距上次复位不足 %.0f 秒，跳过本次" % RESET_COOLDOWN_SECONDS)
        return 1
    log_best_effort("T265 %s 位姿验证失败，执行 USB 复位: %s" % (device["serial"], detail))
    _reset_cycle()
    write_state(
        last="t265",
        serial=device["serial"],
        boot_id=boot,
        verified=False,
        detail=detail,
        reset_cycle_at=time.time(),
        at=time.time(),
    )
    return 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--wait",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="先等待设备出现（默认 0；手动排障时可用）",
    )
    args = parser.parse_args()

    try:
        LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
        lock = LOCK_PATH.open("a+")
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print("另一个 t265-boot-init 实例正在运行，退出", file=sys.stderr)
        return 0

    try:
        return run(args.wait)
    finally:
        fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
