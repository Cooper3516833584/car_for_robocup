#!/usr/bin/env python3
"""Bounded unattended T265/D500/C10B diagnosis; importing opens no devices.

Run the existing robocup_slam.launch.py sidecar separately. Active TOML is
never overwritten: evidence-backed candidates are written under --output.
Long movements are not split. The operator waived the original eight-second
limit; a thirty-second deadline and all localization/drive guards remain.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import statistics
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "code")]
from components.diagnostics_log import JsonlEventLogger
from components.t265_driver import T265RawPose
from components.t265_pose_adapter import T265PoseAdapter, _native_pose_to_robot
from config.relative_slam_profile import accepted_relative_slam_profile
from config.v2_factory import build_differential_drive
from config.v2_loader import DEFAULT_V2_CONFIG, load_v2_config
from config.v2_models import SensorMount3DConfig
from config.v2_runtime import RuntimeMode, validate_runtime_readiness
from core.frames import normalize_angle_rad as wrap
from core.types import Twist2D
from robocup_runtime import build_runtime
from tools.drive_calibration import accepted_fused_sample

TEST_MAX_LINEAR_M_S = 0.06
TEST_MAX_ANGULAR_RAD_S = 0.20
SEGMENT_MAX_S = 30.0
MAX_RADIUS_FROM_ROUND_START_M = 0.60
MAX_SINGLE_POSE_STEP_M = 0.08
MAX_SINGLE_YAW_STEP_RAD = math.radians(15)
MAX_STRAIGHT_YAW_DEVIATION_RAD = math.radians(15)
MIN_T265_TRACKER_CONFIDENCE = 2
INTER_SEGMENT_STOP_S = 1.0
SETTLE_WINDOW_S = 0.40
SETTLE_MAX_YAW_SPAN_RAD = math.radians(0.30)
SETTLE_MAX_WAIT_S = 1.50
FINAL_YAW_TARGET_RAD = math.radians(1)
CORRECTION_MAX_OMEGA_RAD_S = 0.15
MAX_CORRECTIONS = 2
PERIOD_S = 0.05
LEFT_MOUNT = SensorMount3DConfig(-0.00910, 0.17375, 0.03225, 0., 0., math.pi / 2)
ZERO = Twist2D(0., 0.)


class DiagAbort(RuntimeError):
    def __init__(self, reason, verdict="FAILED_LOCALIZATION"):
        super().__init__(reason)
        self.verdict = verdict


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def git(*args):
    p = subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError(p.stderr.strip())
    return p.stdout.strip()


def raw_from_event(e):
    return T265RawPose(tuple(e["raw_translation_xyz"]), tuple(e["raw_quaternion_xyzw"]),
        None if e.get("raw_velocity_xyz") is None else tuple(e["raw_velocity_xyz"]),
        None if e.get("raw_angular_velocity_xyz") is None else tuple(e["raw_angular_velocity_xyz"]),
        int(e.get("tracker_confidence", e.get("confidence", 0))), e.get("mapper_confidence"),
        e.get("t265_device_timestamp_ms"), e["t265_received_time"], e.get("t265_measurement_time"))


class Capture:
    """Mirror runtime events for same-session analysis while retaining its sink."""
    def __init__(self, path):
        self.sink = JsonlEventLogger(path)
        self.events = []
        self.latest = {}

    def emit(self, event, *, priority=False):
        e = dict(event)
        e["host_monotonic_s"] = time.monotonic()
        self.events.append(e)
        self.latest[e["type"]] = e
        return self.sink.emit(e, priority=priority)

    @property
    def dropped_events(self):
        return self.sink.dropped_events

    @property
    def write_error(self):
        return self.sink.write_error

    def close(self):
        self.sink.close()


def yaw_delta(events, kind, key="yaw_rad"):
    rows = [e for e in events if e["type"] == kind and e.get(key) is not None]
    if len(rows) < 2:
        return None
    return sum(wrap(b[key] - a[key]) for a, b in zip(rows, rows[1:]))


def native_yaw(e):
    m = _native_pose_to_robot(raw_from_event(e)).rotation_matrix
    return math.atan2(m[1][0], m[0][0])


def segment_metrics(events):
    result = {}
    for kind in ("t265_pose", "fused_pose", "slam_anchor"):
        d = yaw_delta(events, kind)
        result[kind + "_yaw_delta_deg"] = None if d is None else math.degrees(d)
    raw = [e for e in events if e["type"] == "t265_pose"]
    if len(raw) >= 2:
        result["native_quaternion_yaw_delta_deg"] = math.degrees(sum(
            wrap(native_yaw(b) - native_yaw(a)) for a, b in zip(raw, raw[1:])))
        # Robotics vertical is native +Y; also record the axis-independent norm.
        gyro = [e for e in raw if e.get("raw_angular_velocity_xyz") is not None]
        result["raw_angular_vertical_integral_rad"] = sum(
            a["raw_angular_velocity_xyz"][1] * (b["t265_received_time"] - a["t265_received_time"])
            for a, b in zip(gyro, gyro[1:]))
        result["raw_angular_norm_median_rad_s"] = statistics.median([
            math.sqrt(sum(v*v for v in e["raw_angular_velocity_xyz"])) for e in gyro]) if gyro else None
    commands = [e for e in events if e["type"] == "diag_drive_command"]
    result["drive_commands"] = commands[-1] if commands else None
    return result


def pose_metrics(a, b):
    dx, dy = b.x_m-a.x_m, b.y_m-a.y_m
    return {"longitudinal_m": dx*math.cos(a.yaw_rad)+dy*math.sin(a.yaw_rad),
            "lateral_m": -dx*math.sin(a.yaw_rad)+dy*math.cos(a.yaw_rad),
            "center_shift_m": math.hypot(dx, dy),
            "yaw_delta_deg": math.degrees(wrap(b.yaw_rad-a.yaw_rad))}


class Runner:
    def __init__(self, config, capture, output):
        self.config, self.log, self.output = config, capture, output
        self.runtime = build_runtime(config, RuntimeMode.HARDWARE_PROBE,
                                     sensor_only=True, event_logger=capture)
        self.drive = None
        self.aborted = False
        self.previous = None
        self.origin = None
        self.radius = MAX_RADIUS_FROM_ROUND_START_M
        self.low_since = None
        self.composition_since = None
        self.segments = []
        self.closures = []
        self.phase = "A"
        self.started = time.monotonic()

    def stop(self):
        if self.drive is not None:
            self.drive.stop()

    def tick(self, *, strict=True, straight_start=None):
        if self.aborted:
            raise DiagAbort("signal/SSH session interrupted")
        if time.monotonic()-self.started > 900:
            raise DiagAbort("whole-round watchdog deadline")
        step = self.runtime.step()
        now = time.monotonic()
        if step.error or step.mission_state.value in {"error", "safe_stop"}:
            raise DiagAbort("runtime: " + str(step.error or step.mission_state.value))
        raw = max((self.log.latest.get(k, {}) for k in ("t265_pose", "t265_rejected")),
                  key=lambda e: e.get("host_monotonic_s", -1))
        conf = raw.get("tracker_confidence", raw.get("confidence", 0))
        if conf < MIN_T265_TRACKER_CONFIDENCE:
            self.low_since = self.low_since or now
            if strict and now-self.low_since > 0.3:
                raise DiagAbort("T265 confidence <2 for >0.3s")
        else:
            self.low_since = None
        if self.log.dropped_events or self.log.write_error:
            raise DiagAbort("event log drop/write failure")
        sample = accepted_fused_sample(step.estimate, self.config, now, allow_pending=True)
        if strict and sample is None:
            raise DiagAbort("fresh T265+SLAM fused pose unavailable: " + step.estimate.state.value)
        if self.drive is not None:
            if self.drive.watchdog_stop_count:
                raise DiagAbort("drive watchdog triggered", "FAILED_DRIVE_HARDWARE")
            if not self.drive.backend.rear_driver.is_running:
                raise DiagAbort("C10B sender failed", "FAILED_DRIVE_HARDWARE")
        if sample is not None:
            if self.previous is not None and (
                math.hypot(sample.x_m-self.previous.x_m, sample.y_m-self.previous.y_m)>MAX_SINGLE_POSE_STEP_M
                or abs(wrap(sample.yaw_rad-self.previous.yaw_rad))>MAX_SINGLE_YAW_STEP_RAD):
                raise DiagAbort("fused pose single-step jump")
            if self.origin is not None and math.hypot(sample.x_m-self.origin.x_m, sample.y_m-self.origin.y_m)>self.radius:
                raise DiagAbort("radius from round start exceeded")
            if straight_start is not None and abs(wrap(sample.yaw_rad-straight_start.yaw_rad))>MAX_STRAIGHT_YAW_DEVIATION_RAD:
                raise DiagAbort("straight yaw deviation >15deg", "FAILED_T265_MOUNT")
            self.previous = sample
        anchor = self.runtime.fusion.map_T_t265_odom
        odom = self.runtime.fusion.continuous_t265_pose
        if anchor is not None and odom is not None and step.estimate.pose is not None:
            err = abs(wrap(step.estimate.pose.yaw_rad-anchor.yaw_rad-odom.yaw_rad))
            self.log.emit({"type":"diag_composition", "error_deg":math.degrees(err),
                "accepted_anchor_yaw_rad":anchor.yaw_rad, "continuous_odom_yaw_rad":odom.yaw_rad,
                "adapter_yaw_rad":self.log.latest.get("t265_pose",{}).get("yaw_rad")})
            self.composition_since = (self.composition_since or now) if err>math.radians(.5) else None
            if strict and self.composition_since and now-self.composition_since>1.:
                raise DiagAbort("FUSION_COMPOSITION_INCONSISTENCY")
        return step, sample

    def hold(self, seconds, *, strict=True):
        self.stop()
        deadline = time.monotonic()+seconds
        while time.monotonic()<deadline:
            self.tick(strict=strict)
            time.sleep(PERIOD_S)

    def analyze_while_stopped(self, analyze):
        """Keep feeding existing localization while a finite offline job runs."""
        self.stop()
        result = {}
        def work():
            try:
                result["value"] = analyze()
            except BaseException as error:
                result["error"] = error
        worker = threading.Thread(target=work,name="motion-diag-offline",daemon=True)
        worker.start()
        deadline = time.monotonic()+5.
        while worker.is_alive():
            if time.monotonic()>deadline:
                raise DiagAbort("offline calculation exceeded five seconds")
            self.tick()
            time.sleep(PERIOD_S)
        if "error" in result:
            raise result["error"]
        return result["value"]

    def preflight(self):
        # Startup is bounded and stationary, separate from the ten-second test.
        deadline = time.monotonic()+30
        while time.monotonic()<deadline:
            _, sample = self.tick(strict=False)
            if sample is not None and not self.runtime.fusion.slam_consensus_pending(time.monotonic()):
                break
            time.sleep(PERIOD_S)
        else:
            raise DiagAbort("stationary sensor startup exceeded 30s")
        begin = len(self.log.events)
        samples = [sample]
        start_metrics = self.runtime.slam_bridge.metrics()
        started = time.monotonic()
        while time.monotonic()-started<10:
            _, current = self.tick()
            if self.runtime.fusion.slam_consensus_pending(time.monotonic()):
                raise DiagAbort("stationary SLAM consensus pending")
            samples.append(current)
            time.sleep(PERIOD_S)
        elapsed = time.monotonic()-started
        metrics = self.runtime.slam_bridge.metrics()
        drift = max(math.hypot(p.x_m-sample.x_m,p.y_m-sample.y_m) for p in samples)
        yaw = max(abs(wrap(p.yaw_rad-sample.yaw_rad)) for p in samples)
        rate = (metrics["slam.scan_publish_count"]-start_metrics["slam.scan_publish_count"])/elapsed
        rows = self.log.events[begin:]
        result = {"passed":drift<=.01 and yaw<=math.radians(1) and rate>=3.5,
            "duration_s":elapsed,"drift_m":drift,"yaw_drift_deg":math.degrees(yaw),
            "scan_publish_hz":rate,"anchor_hz":sum(e["type"]=="slam_anchor" for e in rows)/elapsed,
            "t265_samples":sum(e["type"]=="t265_pose" for e in rows),
            "min_tracker_confidence":min(e["tracker_confidence"] for e in rows if e["type"]=="t265_pose"),
            "causal_clamp_count":sum(bool(e.get("measurement_time_clamped_to_receive")) for e in rows),
            "slam_metrics":metrics}
        self.origin = self.previous
        if not result["passed"]:
            raise DiagAbort("stationary drift/scan-rate check failed: " + json.dumps(result))
        return result

    def open_drive(self, config):
        if validate_runtime_readiness(config, RuntimeMode.HARDWARE_MISSION):
            raise DiagAbort("motion readiness failed", "FAILED_DRIVE_PROTOCOL")
        if self.drive is not None:
            self.drive.close()
        self.drive = build_differential_drive(config, fake=False)
        self.drive.start()
        self.drive.stop()

    def send(self, twist):
        if abs(twist.linear_x_m_s)>TEST_MAX_LINEAR_M_S+1e-9 or abs(twist.angular_z_rad_s)>TEST_MAX_ANGULAR_RAD_S+1e-9:
            raise DiagAbort("test speed cap violated", "FAILED_DRIVE_HARDWARE")
        self.drive.command(twist, now_s=time.monotonic())
        limited = self.drive.last_limited_twist
        wheels = self.drive.kinematics.twist_to_wheels(limited)
        encoded = self.drive.backend.last_encoded_wheel_speeds_m_s
        command = self.drive.backend.last_c10b_chassis_command
        self.log.emit({"type":"diag_drive_command", "phase":self.phase,
            "requested_v_m_s":twist.linear_x_m_s,"requested_omega_rad_s":twist.angular_z_rad_s,
            "limited_v_m_s":limited.linear_x_m_s,"limited_omega_rad_s":limited.angular_z_rad_s,
            "left_target_m_s":wheels.left_m_s,"right_target_m_s":wheels.right_m_s,
            "c10b_encoded_left_m_s":encoded[0],"c10b_encoded_right_m_s":encoded[1],
            "c10b_vx_mm_s":command.linear_mm_s,"c10b_vz_mrad_s":command.angular_mrad_s,
            "protocol_mode":self.drive.backend.protocol_mode.value})

    def motion(self, name, v, omega, *, duration=None, integral=None, controller=None, stop_s=1.):
        self.phase = name
        _, start = self.tick()
        begin = len(self.log.events)
        segment = {"name":name,"requested_v_m_s":v,"requested_omega_rad_s":omega,
            "start_host_s":time.monotonic(),"start_event_index":begin,"complete":False,
            "start_pose":asdict(start),"integral_target":integral}
        self.segments.append(segment)
        print("START " + name, flush=True)
        self.log.emit({"type":"diag_segment_start", **segment}, priority=True)
        started = last = time.monotonic()
        integrated = 0.
        previous_limited = ZERO
        try:
            while True:
                now = time.monotonic()
                dt = now-last
                integrated += (previous_limited.linear_x_m_s if v else previous_limited.angular_z_rad_s)*dt
                last = now
                _, current = self.tick(straight_start=start if v and not omega else None)
                if duration is not None and now-started>=duration:
                    break
                if integral is not None and abs(integrated)>=abs(integral):
                    break
                requested = Twist2D(v, omega)
                if controller is not None:
                    output = controller.step(self.runtime.fusion.estimate(time.monotonic()).pose,
                                             now_s=time.monotonic(), pose_state="ok")
                    if output.state.value=="succeeded":
                        break
                    if output.state.value!="running":
                        raise DiagAbort("controller " + output.state.value,"FAILED_DRIVE_HARDWARE")
                    requested = output.command
                if now-started>=SEGMENT_MAX_S:
                    raise DiagAbort("thirty-second movement deadline: " + name,"FAILED_DRIVE_HARDWARE")
                if v and omega and controller is None:
                    # Maintain the requested arc radius during acceleration.
                    # Independent v/w slew would initially request radius .248m
                    # in compat firmware coordinates, below its .350m gate.
                    # Shape test input only; retain existing acceleration limits.
                    ramp_v = min(abs(v), abs(previous_limited.linear_x_m_s)
                                 + self.config.drive.max_linear_accel_m_s2*max(0.,dt))
                    requested = Twist2D(math.copysign(ramp_v,v),omega*ramp_v/abs(v))
                self.send(requested)
                previous_limited = self.drive.last_limited_twist
                time.sleep(PERIOD_S)
            segment["complete"] = True
        finally:
            # Record the stop instant before stop-frame flushing can block.
            segment["stop_cmd_host_s"] = time.monotonic()
            self.log.emit({"type":"diag_stop_command","name":name},priority=True)
            self.stop()
            segment["movement_duration_s"] = segment["stop_cmd_host_s"]-started
            segment["command_integral"] = integrated
        self.hold(stop_s)
        segment["end_event_index"] = len(self.log.events)
        segment["end_host_s"] = time.monotonic()
        segment["metrics"] = {**segment_metrics(self.log.events[begin:]),**pose_metrics(start,self.previous)}
        segment["end_pose"] = asdict(self.previous)
        self.log.emit({"type":"diag_segment_end", **segment}, priority=True)
        print("STOP " + name + " " + json.dumps(segment["metrics"]), flush=True)
        return segment

    def arcs(self, prefix):
        return [self.motion(prefix+str(i),v,w,duration=1.5) for i,(v,w) in enumerate(
            ((.06,.12),(-.06,.12),(.06,-.12),(-.06,-.12)),1)]

    def pair(self, name, target, rotate=False, controller=False):
        a = self.previous
        for sign in (1,-1):
            signed = sign*target
            if controller:
                if rotate:
                    self.runtime.motion.rotate(signed)
                else:
                    self.runtime.motion.drive_distance(signed)
            self.motion(name+("+" if sign>0 else "-"),
                0 if rotate else sign*.06, sign*.20 if rotate else 0,
                integral=None if controller else signed,
                controller=self.runtime.motion if controller else None,stop_s=3 if rotate else 1)
        result = {"name":name,"target":target,"rotation":rotate,
                  "pair_position_closure_m":math.hypot(self.previous.x_m-a.x_m,self.previous.y_m-a.y_m),
                  "pair_yaw_closure_deg":math.degrees(wrap(self.previous.yaw_rad-a.yaw_rad))}
        self.closures.append(result)
        return result

    def settle(self, stopped):
        deadline = time.monotonic()+SETTLE_MAX_WAIT_S
        history = []
        while time.monotonic()<deadline:
            self.tick()
            now = time.monotonic()
            raw = self.log.latest.get("t265_pose",{})
            angular = raw.get("raw_angular_velocity_xyz")
            if angular is not None and now-raw["host_monotonic_s"]<=.15:
                history.append((now,self.previous.yaw_rad,math.sqrt(sum(v*v for v in angular))))
            history = [p for p in history if now-p[0]<=SETTLE_WINDOW_S+.05]
            anchor = self.log.latest.get("slam_anchor",{})
            if len(history)>=6 and history[-1][0]-history[0][0]>=SETTLE_WINDOW_S:
                ys = [wrap(p[1]-history[0][1]) for p in history]
                if (max(ys)-min(ys)<=SETTLE_MAX_YAW_SPAN_RAD
                        and statistics.median(p[2] for p in history)<math.radians(2)
                        and anchor.get("host_monotonic_s",0)>stopped
                        and now-anchor["host_monotonic_s"]<=.5):
                    return self.previous,now-stopped
            time.sleep(PERIOD_S)
        raise DiagAbort("fused/IMU/new-anchor did not settle within 1.5s","FAILED_ROTATION_SETTLING")

    def precise_rotate(self, name, angle, *, corrections):
        start = self.previous
        target = wrap(start.yaw_rad+angle)
        self.runtime.motion.rotate(angle)
        segment = self.motion(name+"_coarse",0,math.copysign(.2,angle),
                              controller=self.runtime.motion,stop_s=0)
        initial_error = wrap(target-self.previous.yaw_rad)
        stopped = segment["stop_cmd_host_s"]
        stable,waited = self.settle(stopped)
        settled_error = wrap(target-stable.yaw_rad)
        count = 0
        signs = []
        while corrections and abs(wrap(target-stable.yaw_rad))>FINAL_YAW_TARGET_RAD and count<MAX_CORRECTIONS:
            error = wrap(target-stable.yaw_rad)
            signs.append(math.copysign(1,error))
            if len(signs)>1 and signs[-1]!=signs[-2]:
                raise DiagAbort("opposing consecutive yaw corrections","FAILED_ROTATION_SETTLING")
            count += 1
            started = time.monotonic()
            try:
                while time.monotonic()-started<1.:
                    self.tick()
                    error = wrap(target-self.previous.yaw_rad)
                    if abs(error)<math.radians(.8):
                        break
                    self.send(Twist2D(0,max(-CORRECTION_MAX_OMEGA_RAD_S,min(CORRECTION_MAX_OMEGA_RAD_S,1.2*error))))
                    time.sleep(PERIOD_S)
            finally:
                stopped = time.monotonic()
                self.log.emit({"type":"diag_stop_command","name":name+"_correction"},priority=True)
                self.stop()
            stable,extra = self.settle(stopped)
            waited += extra
        final_error = wrap(target-stable.yaw_rad)
        result = {"name":name,"first_stop_error_deg":math.degrees(initial_error),
            "post_settle_error_deg":math.degrees(settled_error),"correction_count":count,
            "final_error_deg":math.degrees(final_error),"time_to_stable_s":waited,
            "corrections_enabled":corrections,"passed":abs(final_error)<=FINAL_YAW_TARGET_RAD,
            "post_stop":None}
        segment["end_host_s"] = time.monotonic()
        result["post_stop"] = rotation_cause(list(self.log.events),segment)
        self.log.emit({"type":"diag_precision_rotation",**result},priority=True)
        self.hold(INTER_SEGMENT_STOP_S)
        return result


def sign_verified(segment, reverse=False):
    m = segment.get("metrics",{})
    expected = segment["requested_omega_rad_s"]*(-1 if reverse else 1)
    names = ("t265_pose_yaw_delta_deg","native_quaternion_yaw_delta_deg","fused_pose_yaw_delta_deg")
    # Unmeasurable motion is unknown, never a sign pass.
    return all(m.get(k) is not None and abs(m[k])>=1 and m[k]*expected>0 for k in names)


def candidate(path, original, changes):
    text = original
    for section, values in changes.items():
        pattern = r"(?ms)(^\["+re.escape(section)+r"\]\s*\n)(.*?)(?=^\[|\Z)"
        def update(match):
            body = match[2]
            for key,value in values.items():
                body,n = re.subn(r"(?m)^"+re.escape(key)+r"\s*=\s*[^\n]*", key+" = "+value,body)
                if n!=1:
                    raise ValueError("candidate key missing/duplicated: " + key)
            return match[1]+body
        text,n = re.subn(pattern,update,text)
        if n!=1:
            raise ValueError("candidate section missing/duplicated: " + section)
    path.write_text(text,encoding="utf-8")
    return load_v2_config(path)


def mount_ab(events, segments, mount):
    adapter = T265PoseAdapter(mount)
    poses = []
    for i,e in enumerate(events):
        if e["type"]!="t265_pose":
            continue
        raw = raw_from_event(e)
        update = adapter.adapt(raw,now_s=raw.received_monotonic_s)
        if update.pose is not None:
            poses.append((i,update.pose))
    result = {}
    for segment in segments:
        selected = [p for i,p in poses if segment["start_event_index"]<=i<segment.get("end_event_index",len(events))]
        if len(selected)>=2:
            result[segment["name"]] = pose_metrics(selected[0],selected[-1])
            result[segment["name"]]["max_center_shift_m"] = max(math.hypot(p.x_m-selected[0].x_m,p.y_m-selected[0].y_m) for p in selected)
    return result


def rotation_cause(events, segment):
    stopped = segment["stop_cmd_host_s"]
    rows = [e for e in events if stopped<=e["host_monotonic_s"]<=segment.get("end_host_s",stopped+3)]
    raw = [e for e in rows if e["type"]=="t265_pose" and e.get("raw_angular_velocity_xyz")]
    still = None
    for e in raw:
        window = [r for r in raw if e["host_monotonic_s"]-.3<=r["host_monotonic_s"]<=e["host_monotonic_s"]]
        if len(window)>=4 and window[-1]["host_monotonic_s"]-window[0]["host_monotonic_s"]>=.25:
            norms = [math.sqrt(sum(v*v for v in r["raw_angular_velocity_xyz"])) for r in window]
            if statistics.median(norms)<math.radians(2):
                still = e["host_monotonic_s"]
                break
    after = rows if still is None else [e for e in rows if e["host_monotonic_s"]>=still]
    result = {"name":segment["name"],"t_imu_still_after_stop_s":None if still is None else still-stopped,
              "post_stop":segment_metrics(rows),"post_imu_still":segment_metrics(after)}
    initial = [r for r in raw if r["host_monotonic_s"]<=stopped+.3]
    initial_norm = statistics.median([math.sqrt(sum(v*v for v in r["raw_angular_velocity_xyz"]))
                                     for r in initial]) if initial else None
    result["initial_gyro_norm_median_deg_s"] = None if initial_norm is None else math.degrees(initial_norm)
    m = result["post_imu_still"]
    native = abs(m.get("native_quaternion_yaw_delta_deg") or 0)
    adapter = abs(m.get("t265_pose_yaw_delta_deg") or 0)
    fused = abs(m.get("fused_pose_yaw_delta_deg") or 0)
    post = result["post_stop"]
    changes = [post.get(k) for k in ("native_quaternion_yaw_delta_deg","t265_pose_yaw_delta_deg","fused_pose_yaw_delta_deg")]
    moving_after_stop = (initial_norm is not None and initial_norm>math.radians(2)
        and all(v is not None and abs(v)>.5 for v in changes)
        and all(v*changes[0]>0 for v in changes))
    if still is not None and 1<=native<=5 and 1<=adapter<=5:
        cause = "t265_pose_settling_after_physical_stop"
    elif moving_after_stop:
        # Eventual stopping does not erase real movement before t_imu_still.
        cause = "physical_or_drive_motion_after_stop"
    elif native<.3 and adapter>1:
        cause = "adapter_or_time_or_rebase"
    elif native<.3 and adapter<.3 and fused>1:
        cause = "fusion_or_slam_anchor"
    else:
        cause = "inconclusive_or_no_significant_settling"
    result["cause"] = cause
    return result


def reanalyze(run):
    report = json.loads((run/"report.json").read_text(encoding="utf-8"))
    manifest = json.loads((run/"manifest.json").read_text(encoding="utf-8"))
    events = sorted([json.loads(line) for line in (run/"events.jsonl").read_text(encoding="utf-8").splitlines()],
                    key=lambda e:e["host_monotonic_s"])
    rotations = [s for s in report["segments"] if s["name"].startswith("D_")]
    report["rotation"] = [rotation_cause(events,s) for s in rotations]
    if any(r["cause"]=="physical_or_drive_motion_after_stop" for r in report["rotation"]):
        report["acquisition_error"] = report.get("acquisition_error",report.get("error"))
        report["verdict"] = "FAILED_DRIVE_HARDWARE"
        report["error"] = "STOP followed by nonzero raw angular velocity and aligned native/adapter/fused yaw; stable-IMU yaw drift is small"
    report["analysis_commit"] = git("rev-parse","HEAD")
    report["analysis_source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    write_json(run/"report.json",report)
    (run/"report.md").write_text(render_report(report,manifest),encoding="utf-8")
    print("REPORT " + str(run/"report.md"))


def render_report(report, manifest):
    headings = [("Baseline",{k:manifest[k] for k in ("commit","config_sha256","base_config","effective_config","t265_serial")}),
        ("Stationary health",report.get("stationary_health")),("C10B protocol diagnosis",report.get("protocol")),
        ("T265 mount diagnosis",report.get("mount")),("Rotation stop diagnosis",report.get("rotation")),
        ("Experimental settle correction",report.get("settle")),("Linear regression",report.get("linear")),
        ("Round-trip closure",report.get("closures")),("Changes made",report.get("changes",[]))]
    lines = ["# Automatic motion diagnosis", "", "Final verdict: **"+report["verdict"]+"**", "",
        "Error: "+str(report.get("error")),"", "No external chassis truth. All measurements are internal T265 + D500/SLAM consistency.", "",
        "Limits: v <= 0.06 m/s, omega <= 0.20 rad/s, movement <= 30.0 s, radius <= 0.60 m (0.75 m only for 50 cm).", "",
        "Operator explicitly waived the original eight-second limit and requested unsplit long movements. A finite thirty-second deadline remains.", ""]
    for title,value in headings:
        lines += ["## "+title,"","```json",json.dumps(value if value is not None else {"status":"NOT_RUN"},ensure_ascii=False,indent=2),"```",""]
    lines += ["## Completed/aborted segments","", "| Segment | Complete | Time (s) | Fused yaw (deg) |", "|---|---|---|---|"]
    for s in report["segments"]:
        lines.append("| %s | %s | %.3f | %s |"%(s["name"],s["complete"],s.get("movement_duration_s",0),s.get("metrics",{}).get("fused_pose_yaw_delta_deg")))
    lines += ["", "## Remaining uncertainty","", "Unrun phases are unverified. No physical position or yaw accuracy claim is made."]
    return "\n".join(lines)+"\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config")
    parser.add_argument("--output")
    parser.add_argument("--analyze-run",help="reanalyze existing JSONL without hardware")
    parser.add_argument("--confirm-unattended-low-speed",action="store_true")
    parser.add_argument("--confirm-area-clear",action="store_true")
    parser.add_argument("--mount-retest-from",help="prior run with verified protocol and a better left mount")
    parser.add_argument("--rotation-from",help="reuse completed physical mount retest and begin at phase D")
    args = parser.parse_args(argv)
    if args.analyze_run:
        reanalyze(Path(args.analyze_run).resolve())
        return 0
    if not args.config or not args.output:
        parser.error("live diagnosis requires --config and --output")
    if sys.platform!="linux" or not(args.confirm_unattended_low_speed and args.confirm_area_clear):
        parser.error("Linux and explicit unattended low-speed/clear-area authorization required")
    path,output = Path(args.config).resolve(),Path(args.output).resolve()
    if path==DEFAULT_V2_CONFIG.resolve() or output.exists() or output==ROOT or ROOT in output.parents:
        parser.error("use the actual profile and a new output directory outside the checkout")
    if git("diff","--name-only") or git("diff","--cached","--name-only"):
        parser.error("tracked worktree must be clean")
    base = load_v2_config(path)
    if base.relay.enabled:
        parser.error("payload relay must be disabled")
    config = accepted_relative_slam_profile(base)
    config = replace(config,drive=replace(config.drive,
        max_linear_speed_m_s=min(config.drive.max_linear_speed_m_s,TEST_MAX_LINEAR_M_S),
        max_angular_speed_rad_s=min(config.drive.max_angular_speed_rad_s,TEST_MAX_ANGULAR_RAD_S)))
    original = path.read_text(encoding="utf-8")
    output.mkdir(parents=True)
    prior = None
    mount_prior = None
    if args.mount_retest_from:
        prior_dir = Path(args.mount_retest_from).resolve()
        prior = json.loads((prior_dir/"report.json").read_text(encoding="utf-8"))
        prior_manifest = json.loads((prior_dir/"manifest.json").read_text(encoding="utf-8"))
        arcs = [s for s in prior["segments"] if s["name"].startswith(("B2a","B2b"))]
        if (prior_manifest["config_sha256"]!=hashlib.sha256(path.read_bytes()).hexdigest()
                or len(arcs)!=8 or not all(sign_verified(s) for s in arcs)
                or prior.get("mount",{}).get("conclusion")!="left_mount_candidate_better"):
            parser.error("prior run does not authorize this protocol/mount candidate")
        combined = candidate(output/"candidate_t265_mount.toml",original,{
            "calibration":{"c10b_diff_firmware_verified":"true"},
            "vehicle.drive":{"protocol_mode":'"differential_vx_vz"'},
            "sensors.t265.mount":{k:str(v) for k,v in asdict(LEFT_MOUNT).items()}})
        config = replace(accepted_relative_slam_profile(combined),drive=replace(config.drive,protocol_mode="differential_vx_vz"),t265_mount=LEFT_MOUNT)
        config = replace(config,calibration=combined.calibration)
        if args.rotation_from:
            mount_dir = Path(args.rotation_from).resolve()
            mount_prior = json.loads((mount_dir/"report.json").read_text(encoding="utf-8"))
            mount_manifest = json.loads((mount_dir/"manifest.json").read_text(encoding="utf-8"))
            if (mount_manifest["config_sha256"]!=prior_manifest["config_sha256"]
                    or mount_prior.get("dropped_events") or mount_prior.get("log_write_error")
                    or mount_prior.get("mount",{}).get("conclusion")!="left_mount_verified_by_ab_and_motion"
                    or mount_manifest["effective_config"]["t265_mount"]!=asdict(LEFT_MOUNT)):
                parser.error("prior physical mount retest is not valid evidence")
    elif args.rotation_from:
        parser.error("--rotation-from also requires --mount-retest-from")
    manifest = {"commit":git("rev-parse","HEAD"),"git_status":git("status","--short"),
        "config_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),"config_path":str(path),
        "base_config":asdict(base),"effective_config":asdict(config),"t265_serial":"unknown",
        "operator_evidence":"area clear; unattended low-speed authorized; T265 on left side",
        "operator_deadline_override":"Do not split movements; no eight-second limit. Retain finite 30s deadline.",
        "python":sys.version,"started_utc":time.strftime("%Y-%m-%dT%H:%M:%SZ",time.gmtime()),
        "critical_source_sha256":{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [Path(__file__),ROOT/"code/robocup_runtime.py",ROOT/"code/components/pose_fusion.py",
                      ROOT/"code/components/t265_pose_adapter.py",ROOT/"code/components/differential_drive.py",ROOT/"code/components/c10b_diff_backend.py"]}}
    write_json(output/"manifest.json",manifest)
    capture = Capture(output/"events.jsonl")
    runner = None
    handlers = {}
    report = {"verdict":"FAILED_LOCALIZATION","error":None,"changes":[],"segments":[],"closures":[]}
    try:
        runner = Runner(config,capture,output)
        def abort(signum,frame):
            runner.aborted = True
            runner.stop()
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            handlers[sig] = signal.signal(sig,abort)
        runner.runtime.start()
        manifest["t265_serial"] = runner.runtime.t265_source.serial
        write_json(output/"manifest.json",manifest)
        report["stationary_health"] = runner.preflight()
        print("PREFLIGHT PASS " + json.dumps(report["stationary_health"]),flush=True)
        runner.open_drive(config)
        arcs = runner.arcs("B") if prior is None else []
        reverse_bug = bool(arcs) and all(sign_verified(arcs[i],reverse=i in (1,3)) for i in range(4))
        report["protocol"] = dict(prior["protocol"]) if prior else {"baseline_arcs":[s["metrics"] for s in arcs],
            "conclusion":"LIKELY_DIFF_CAR_REVERSE_VZ_BEHAVIOR" if reverse_bug else "UNKNOWN",
            "config_changed":False}
        if reverse_bug:
            proposed = candidate(output/"candidate_diff_vx_vz.toml",original,{
                "calibration":{"c10b_diff_firmware_verified":"true"},
                "vehicle.drive":{"protocol_mode":'"differential_vx_vz"'}})
            proposed = replace(config,calibration=proposed.calibration,drive=replace(config.drive,protocol_mode="differential_vx_vz"))
            runner.open_drive(proposed)
            repeated = runner.arcs("B2a")+runner.arcs("B2b")
            report["protocol"]["candidate_arcs"] = [s["metrics"] for s in repeated]
            if all(sign_verified(s) for s in repeated):
                report["protocol"]["conclusion"] = "differential_vx_vz_verified_by_motion"
                config = proposed
                report["changes"].append("candidate_diff_vx_vz.toml verified; active config awaits PC deployment")
            else:
                runner.open_drive(config)
                report["protocol"]["conclusion"] = "UNKNOWN_CANDIDATE_FAILED"
        elif arcs and all(sign_verified(s) for s in arcs):
            report["protocol"]["conclusion"] = "current_protocol_signs_consistent"
        cstart = len(runner.segments)
        if mount_prior is None:
            runner.pair("C_straight20",.20)
            runner.pair("C_rotate90",math.pi/2,rotate=True)
        csegments = runner.segments[cstart:]
        snapshot = list(capture.events)
        a,b = ({},{}) if mount_prior else runner.analyze_while_stopped(lambda:(
            mount_ab(snapshot,csegments,base.t265_mount),mount_ab(snapshot,csegments,LEFT_MOUNT)))
        improves = all(b[s["name"]]["max_center_shift_m"]<=.6*a[s["name"]]["max_center_shift_m"] for s in csegments if "rotate" in s["name"])
        lateral_ok = all(abs(b[s["name"]]["lateral_m"])<=abs(a[s["name"]]["lateral_m"])+.01 for s in csegments if "straight" in s["name"])
        report["mount"] = dict(mount_prior["mount"]) if mount_prior else {"current":a,"left_candidate":b,"conclusion":"left_mount_candidate_better" if improves and lateral_ok else "inconclusive", "config_changed":False}
        if mount_prior is not None:
            report["mount"]["physical_retest_run"] = args.rotation_from
        elif prior is not None:
            prior_a = prior["mount"]["current"]
            retest_ok = all(b[s["name"]]["max_center_shift_m"]<=.6*prior_a[s["name"]]["max_center_shift_m"]
                           for s in csegments if "rotate" in s["name"])
            retest_ok = retest_ok and all(abs(b[s["name"]]["lateral_m"])<=abs(prior_a[s["name"]]["lateral_m"])+.01
                           for s in csegments if "straight" in s["name"])
            report["mount"]["physical_retest_passed"] = retest_ok
            report["mount"]["baseline_run"] = args.mount_retest_from
            if not retest_ok:
                raise DiagAbort("left mount physical retest did not retain the A/B improvement","FAILED_T265_MOUNT")
            report["mount"]["conclusion"] = "left_mount_verified_by_ab_and_motion"
        elif improves and lateral_ok:
            candidate(output/"candidate_t265_mount.toml",original,{"sensors.t265.mount":{k:str(v) for k,v in asdict(LEFT_MOUNT).items()}})
            # Retesting with a new mount requires a fresh adapter/SLAM session.
            raise DiagAbort("left mount candidate requires fresh-session physical retest before later phases","FAILED_T265_MOUNT")
        dstart = len(runner.segments)
        runner.pair("D_rotate30",math.pi/6,rotate=True)
        runner.pair("D_rotate90",math.pi/2,rotate=True)
        report["rotation"] = [rotation_cause(capture.events,s) for s in runner.segments[dstart:]]
        if report["protocol"]["conclusion"].startswith("UNKNOWN"):
            raise DiagAbort("protocol sign evidence inconclusive; precision phases blocked","FAILED_DRIVE_PROTOCOL")
        if any(r["cause"]=="physical_or_drive_motion_after_stop" for r in report["rotation"]):
            raise DiagAbort("IMU/native/adapter/fused evidence of continued motion after STOP","FAILED_DRIVE_HARDWARE")
        if any(r["cause"] in {"adapter_or_time_or_rebase","fusion_or_slam_anchor"} for r in report["rotation"]):
            raise DiagAbort("rotation attribution requires adapter/fusion investigation before controller changes")
        use_corrections = any(r["cause"]=="t265_pose_settling_after_physical_stop" for r in report["rotation"])
        report["settle"] = {"enabled_by_diagnosis":use_corrections,"runs":[]}
        for i,sign in enumerate((1,-1,1,-1,1,-1,1,-1)):
            result = runner.precise_rotate("E%d"%i,sign*math.pi/2,corrections=use_corrections)
            report["settle"]["runs"].append(result)
            if not result["passed"]:
                raise DiagAbort("FAIL_FINAL_YAW: "+json.dumps(result),"FAILED_ROTATION_SETTLING")
        if report["mount"]["conclusion"]!="left_mount_verified_by_ab_and_motion":
            raise DiagAbort("mount not verified; position precision tests blocked","FAILED_T265_MOUNT")
        precision_text = (output/"candidate_t265_mount.toml").read_text(encoding="utf-8")
        candidate(output/"candidate_precision.toml",precision_text,{"navigation":{
            "position_tolerance_m":"0.01","slowdown_distance_m":"0.15"}})
        config = replace(config,navigation=replace(config.navigation,position_tolerance_m=.01,slowdown_distance_m=.15))
        runner.runtime.motion.navigation = config.navigation
        runner.runtime.motion.navigator.navigation = config.navigation
        report["linear"] = []
        for distance,repeats in ((.2,1),(.5,3)):
            runner.radius = .75 if distance==.5 else .60
            for i in range(repeats):
                index = len(runner.segments)
                closure = runner.pair("F_%dcm_%d"%(distance*100,i),distance,controller=True)
                measurements = []
                for s in runner.segments[index:]:
                    metrics = s["metrics"]
                    target = math.copysign(distance,s["requested_v_m_s"])
                    passed = abs(metrics["longitudinal_m"]-target)<=.01 and abs(metrics["lateral_m"])<=.01
                    measurements.append({"name":s["name"],"error_m":metrics["longitudinal_m"]-target,
                        "lateral_m":metrics["lateral_m"],"passed":passed})
                report["linear"].append({"actions":measurements,"closure":closure})
                if not all(m["passed"] for m in measurements):
                    raise DiagAbort("linear precision regression failed","FAILED_DRIVE_HARDWARE")
        # Final matrix retains this same T265/SLAM session.
        runner.radius = .60
        report["final_stationary_health"] = runner.preflight()
        final_arcs = runner.arcs("FINAL_B")
        if not all(sign_verified(s) for s in final_arcs):
            raise DiagAbort("final protocol direction matrix failed","FAILED_DRIVE_PROTOCOL")
        runner.pair("FINAL_small20",.2,controller=True)
        for sign in (1,-1):
            runner.precise_rotate("FINAL_30_%d"%sign,sign*math.pi/6,corrections=use_corrections)
        for i,sign in enumerate((1,-1,1,-1,1,-1)):
            result = runner.precise_rotate("FINAL_90_%d"%i,sign*math.pi/2,corrections=use_corrections)
            if not result["passed"]:
                raise DiagAbort("final settled rotation failed","FAILED_ROTATION_SETTLING")
        for distance in (.2,.5):
            runner.radius = .75 if distance==.5 else .60
            for i in range(3):
                index = len(runner.segments)
                closure = runner.pair("FINAL_%dcm_%d"%(distance*100,i),distance,controller=True)
                if closure["pair_position_closure_m"]>(.015 if distance==.2 else .020):
                    raise DiagAbort("final position closure failed","FAILED_DRIVE_HARDWARE")
                for s in runner.segments[index:]:
                    m = s["metrics"]
                    if abs(m["longitudinal_m"]-math.copysign(distance,s["requested_v_m_s"]))>.01 or abs(m["lateral_m"])>.01:
                        raise DiagAbort("final longitudinal/lateral error >1cm","FAILED_DRIVE_HARDWARE")
        report["verdict"] = "INTERNAL_CLOSED_LOOP_PASS"
    except BaseException as exc:
        report["error"] = type(exc).__name__+": "+str(exc)
        report["verdict"] = getattr(exc,"verdict","FAILED_LOCALIZATION")
        print("ABORT " + report["error"],flush=True)
    finally:
        if runner is not None:
            try:
                runner.stop()
            finally:
                try:
                    if runner.drive is not None:
                        runner.drive.close()
                finally:
                    runner.runtime.close()
            report["segments"] = runner.segments
            report["closures"] = runner.closures
            # Same raw-data mount comparison remains useful after an early abort.
            relevant = [s for s in runner.segments if s["name"].startswith("C_")]
            if relevant and "mount" not in report:
                report["mount"] = {"current":mount_ab(capture.events,relevant,base.t265_mount),
                    "left_candidate":mount_ab(capture.events,relevant,LEFT_MOUNT),"conclusion":"incomplete_collection","config_changed":False}
            if "stationary_health" not in report:
                report["stationary_health"] = {"passed":False,"reason":report["error"],
                    "last_fused":capture.latest.get("fused_pose"),"last_slam_status":capture.latest.get("slam_status"),
                    "last_t265":capture.latest.get("t265_pose"),"last_rejected":capture.latest.get("t265_rejected")}
        capture.close()
        for sig,handler in handlers.items():
            signal.signal(sig,handler)
        report["dropped_events"] = capture.dropped_events
        report["log_write_error"] = capture.write_error
        report["event_counts"] = dict(Counter(e["type"] for e in capture.events))
        report["active_config_unchanged"] = hashlib.sha256(path.read_bytes()).hexdigest()==manifest["config_sha256"]
        write_json(output/"segments.json",report["segments"])
        write_json(output/"report.json",report)
        (output/"report.md").write_text(render_report(report,manifest),encoding="utf-8")
        print("REPORT " + str(output/"report.md"),flush=True)
    return 0 if report["verdict"]=="INTERNAL_CLOSED_LOOP_PASS" else 1


if __name__=="__main__":
    raise SystemExit(main())
