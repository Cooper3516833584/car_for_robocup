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
import os
from pathlib import Path
import re
import signal
import select
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
from tools.c10b_telemetry_probe import _aligned_frames, telemetry_row
from components.battery_voltage_monitor import _open_c10b_telemetry_port, decode_stock_c10b_voltage_v

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
    if manifest.get('stage')=='rotation_speed_sweep':
        telemetry=[json.loads(line) for line in (run/'c10b_telemetry.jsonl').read_text(encoding='utf-8').splitlines()]
        report['rotation_speed_sweep']=analyze_speed_sweep(events,report['segments'],telemetry,manifest['base_config']['geometry']['drive_track_width_m'])
        if not report.get('error'):
            report['verdict']=report['rotation_speed_sweep']['verdict']
        report['analysis_commit']=git('rev-parse','HEAD')
        write_json(run/'report.json',report)
        (run/'report.md').write_text(render_speed_sweep(report,manifest),encoding='utf-8')
        print('REPORT '+str(run/'report.md'))
        return
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


SWEEP_SPEEDS = (.15, .20, .25, .30, .35, .40)


def sustained(rows, predicate, duration, *, time_key, max_gap=.12):
    first = previous = None
    for row in rows:
        now = row[time_key]
        if not predicate(row):
            first = previous = None
            continue
        if first is None or now-previous > max_gap:
            first = now
        previous = now
        if now-first >= duration:
            return first, now
    return None, None


def analyze_speed_sweep(events, segments, telemetry, track):
    """Internal-device response, with missing samples kept unknown rather than zero."""
    raw = sorted((e for e in events if e['type']=='t265_pose' and
                  e.get('raw_angular_velocity_xyz') is not None), key=lambda e:e['t265_received_time'])
    norm = lambda e:math.sqrt(sum(v*v for v in e['raw_angular_velocity_xyz']))
    first_command = min((e['command_call_host_s'] for e in events if e['type']=='diag_drive_command'),default=math.inf)
    baseline = [r for r in telemetry if r['t_mono'] < first_command]
    med = lambda values:statistics.median(values) if values else None
    def correlation(pairs):
        if len(pairs)<6:
            return None
        xm,ym=statistics.mean(x for x,y in pairs),statistics.mean(y for x,y in pairs)
        denominator=math.sqrt(sum((x-xm)**2 for x,y in pairs)*sum((y-ym)**2 for x,y in pairs))
        return sum((x-xm)*(y-ym) for x,y in pairs)/denominator if denominator else None
    center = [med([r['int16_be_fields'][i] for r in baseline]) for i in range(9)]
    noise = [med([abs(r['int16_be_fields'][i]-center[i]) for r in baseline]) if baseline else None for i in range(9)]
    rows = []
    for speed in SWEEP_SPEEDS:
        for sign in (1,-1):
            omega = speed*sign
            row = {'requested_omega_rad_s':omega,'wheel_mm_s':speed*track*500,
                   'direction':'CCW' if sign>0 else 'CW','verdict':'NOT_RUN'}
            segment = next((s for s in segments if s['requested_omega_rad_s']==omega),None)
            if segment is None:
                rows.append(row)
                continue
            commands = [e for e in events if e['type']=='diag_drive_command' and e.get('phase')==segment['name']]
            if not commands:
                rows.append(row)
                continue
            start, stop = commands[0]['command_call_host_s'],segment['stop_cmd_host_s']
            end = min(segment.get('end_host_s',stop+2),stop+2)
            motion = [e for e in raw if start <= e['t265_received_time'] <= stop]
            steady = [e for e in motion if 1.25 <= e['t265_received_time']-start <= 1.75]
            after = [e for e in raw if stop <= e['t265_received_time'] <= end]
            onset, confirmation = sustained(motion,lambda e:norm(e)>math.radians(5),.10,time_key='t265_received_time')
            measured = med([norm(e) for e in steady])
            vertical = med([e['raw_angular_velocity_xyz'][1] for e in steady])
            signed = math.copysign(measured,vertical) if measured is not None and vertical else None
            ratio = signed/omega if signed is not None else None
            intermittent = sum(norm(e)<.25*speed for e in steady)/len(steady) if steady else None
            steady_coverage = len(steady)>=6 and steady[-1]['t265_received_time']-steady[0]['t265_received_time']>=.35 and max((b['t265_received_time']-a['t265_received_time'] for a,b in zip(steady,steady[1:])),default=1)<.12
            still = None
            for e in after:
                t = e['t265_received_time']
                # Include the sample bracketing the left boundary. Requiring
                # samples strictly inside the window to span .30 s incorrectly
                # rejects a healthy ~18 Hz stream whose timestamps are offset.
                before=[a for a in after if a['t265_received_time']<=t-.30]
                window=before[-1:]+[a for a in after if t-.30<a['t265_received_time']<=t]
                if before and len(window)>=6 and max((b['t265_received_time']-a['t265_received_time'] for a,b in zip(window,window[1:])),default=1)<.12 and med([norm(a) for a in window])<math.radians(2):
                    still = t
                    break
            post_still = [e for e in after if still is not None and e['t265_received_time']>=still]
            native_delta = lambda es:math.degrees(sum(wrap(native_yaw(b)-native_yaw(a)) for a,b in zip(es,es[1:]))) if len(es)>=2 else None
            zero_set = next((e for e in events if e['type']=='diag_c10b_command_set' and e['host_monotonic_s']>=stop and e['zero']),None)
            zero_write = next((e for e in events if e['type']=='diag_c10b_frame_write' and e['write_begin_host_s']>=stop and e['zero']),None)
            tele_steady = [r for r in telemetry if start+1.25<=r['t_mono']<=start+1.75]
            row.update(t_command_start=start,t_motion_start=onset,t_motion_confirmed=confirmation,t_stop_cmd=stop,
                movement_duration_s=stop-start,start_delay_s=None if onset is None else onset-start,
                measured_steady_omega_rad_s=signed,steady_norm_rad_s=measured,response_ratio=ratio,
                steady_vertical_rad_s=vertical,steady_native_yaw_deg=native_delta(steady),
                intermittent_fraction=intermittent,steady_coverage=steady_coverage,
                steady_sample_count=len(steady),t_t265_still=still,
                t265_stop_delay_s=None if still is None else still-stop,
                yaw_after_t265_still_deg=native_delta(post_still),
                software_zero_delay_s=None if zero_set is None else zero_set['host_monotonic_s']-stop,
                first_zero_write_delay_s=None if zero_write is None else zero_write['write_end_host_s']-stop,
                commanded_integral_deg=math.degrees(segment['command_integral']),
                native_yaw_deg=native_delta(motion),adapter_yaw_deg=None,fused_yaw_deg=None,
                telemetry_steady_fields=[med([r['int16_be_fields'][i] for r in tele_steady]) for i in range(9)],
                telemetry_steady_samples=len(tele_steady),complete=segment['complete'])
            subset = [e for e in events if start<=e['host_monotonic_s']<=stop]
            for kind,key in (('t265_pose','adapter_yaw_deg'),('fused_pose','fused_yaw_deg')):
                delta=yaw_delta(subset,kind)
                row[key]=None if delta is None else math.degrees(delta)
            reject = any(r['motor_disabled'] for r in tele_steady)
            reliable = (segment['complete'] and onset is not None and onset-start<=.50 and
                        steady_coverage and ratio is not None and ratio>=.70 and
                        intermittent is not None and intermittent<=.20 and not reject and len(tele_steady)>=3)
            row['motor_disabled_in_steady']=reject
            row['verdict']='RELIABLE' if reliable else 'START_FAILED' if onset is None else 'UNRELIABLE'
            rows.append(row)
    # Firmware field semantics are not assumed. Identify command-correlated
    # int16 patterns across the sweep, then compare with the stationary pattern.
    correlations=[]
    aligned=[]; raw_index=0
    for frame in sorted(telemetry,key=lambda r:r['t_mono']):
        if not raw:
            break
        while raw_index+1<len(raw) and abs(raw[raw_index+1]['t265_received_time']-frame['t_mono'])<=abs(raw[raw_index]['t265_received_time']-frame['t_mono']):
            raw_index+=1
        if abs(raw[raw_index]['t265_received_time']-frame['t_mono'])<=.12:
            aligned.append((frame,raw[raw_index]['raw_angular_velocity_xyz'][1]))
    for i in range(9):
        pairs=[(r['requested_omega_rad_s'],r['telemetry_steady_fields'][i]) for r in rows if r.get('telemetry_steady_samples',0)>=3]
        corr=correlation(pairs)
        sensor_corr=correlation([(vertical,frame['int16_be_fields'][i]) for frame,vertical in aligned])
        signal=med([abs(frame['int16_be_fields'][i]-center[i]) for frame,vertical in aligned if abs(vertical)>math.radians(5)]) if baseline else None
        # Slow starts can leave the prescribed steady window almost stationary.
        # Also validate the full time-aligned movement pattern against T265;
        # this identifies a field without inventing its units or semantics.
        selected=((corr is not None and abs(corr)>=.80) or (sensor_corr is not None and abs(sensor_corr)>=.80)) and signal is not None and signal>max(20,6*noise[i])
        correlations.append({'byte_offset':2+2*i,'correlation_with_requested_omega':corr,'stationary_median':center[i],
                             'correlation_with_t265_vertical':sensor_corr,'stationary_mad':noise[i],'median_motion_deviation':signal,'selected':selected})
    selected=[i for i,c in enumerate(correlations) if c['selected']]
    for row in rows:
        if 't_stop_cmd' not in row:
            continue
        stop=row['t_stop_cmd']
        after=[r for r in telemetry if stop<=r['t_mono']<=stop+2]
        thresholds={i:max(5,3*noise[i],.10*correlations[i]['median_motion_deviation']) for i in selected}
        before=[r for r in telemetry if row['t_command_start']<=r['t_mono']<=stop]
        distinguishable=bool(selected) and any(abs(r['int16_be_fields'][i]-center[i])>thresholds[i] for r in before for i in selected)
        zero,confirm=sustained(after,lambda r:all(abs(r['int16_be_fields'][i]-center[i])<=thresholds[i] for i in selected),.10,time_key='t_mono',max_gap=.15) if selected else (None,None)
        row.update(t_c10b_zero=zero,t_c10b_zero_confirmed=confirm,c10b_stop_delay_s=None if zero is None else zero-stop,
                   telemetry_stop_samples=len(after),telemetry_stop_pattern_observable=distinguishable)
        # Require valid, dense evidence through the end before flagging a
        # persistent motion pattern. Missing telemetry is explicitly unknown.
        coverage=len(after)>=10 and after[-1]['t_mono']>=stop+1.8 and max((b['t_mono']-a['t_mono'] for a,b in zip(after,after[1:])),default=2)<=.20
        delay=row['c10b_stop_delay_s']
        row['stop_class']='UNKNOWN_TELEMETRY_PATTERN'
        if selected and not distinguishable and zero is not None:
            row['stop_class']='ALREADY_STATIONARY_PATTERN'
        elif distinguishable and zero is None and coverage:
            row['stop_class']='C10B_STOP_RESPONSE_ABNORMAL'
        elif zero is not None:
            yaw=row['yaw_after_t265_still_deg']
            if yaw is not None and abs(yaw)>1:
                row['stop_class']='T265_POST_STOP_SETTLING'
            elif delay<=.25 and row['software_zero_delay_s'] is not None and row['software_zero_delay_s']<=.02 and any(e['t265_received_time']>=zero+.10 and norm(e)>math.radians(5) for e in raw if stop<=e['t265_received_time']<=stop+2):
                row['stop_class']='MECHANICAL_OR_MOTOR_COAST'
            else:
                row['stop_class']='STOP_PATTERN_RETURNED'
    minima={direction:min((abs(r['requested_omega_rad_s']) for r in rows if r['direction']==direction and r['verdict']=='RELIABLE'),default=None) for direction in ('CCW','CW')}
    asym=[]
    for i in range(0,12,2):
        a,b=rows[i:i+2]
        ds=abs(a['start_delay_s']-b['start_delay_s']) if a.get('start_delay_s') is not None and b.get('start_delay_s') is not None else None
        dr=abs(a['response_ratio']-b['response_ratio']) if a.get('response_ratio') is not None and b.get('response_ratio') is not None else None
        if (ds is not None and ds>.30) or (dr is not None and dr>.20):
            asym.append({'omega_rad_s':abs(a['requested_omega_rad_s']),'start_delay_difference_s':ds,'response_ratio_difference':dr,
                         'worse_direction':a['direction'] if a.get('response_ratio',0)<b.get('response_ratio',0) else b['direction'],
                         'ccw_stop_delay_s':a.get('t265_stop_delay_s'),'cw_stop_delay_s':b.get('t265_stop_delay_s')})
    maximum=lambda key,absolute=False:max((abs(r[key]) if absolute else r[key] for r in rows if r.get(key) is not None),default=None)
    summary={'minimum_reliable_ccw_omega':minima['CCW'],'minimum_reliable_cw_omega':minima['CW'],
             'minimum_reliable_rotation_omega':max(minima.values()) if all(v is not None for v in minima.values()) else None,
             'worst_start_delay_s':maximum('start_delay_s'),'worst_c10b_stop_delay_s':maximum('c10b_stop_delay_s'),
             'worst_t265_stop_delay_s':maximum('t265_stop_delay_s'),'max_post_still_yaw_deg':maximum('yaw_after_t265_still_deg',True),
             'worst_software_zero_delay_s':maximum('software_zero_delay_s'),'worst_first_zero_write_delay_s':maximum('first_zero_write_delay_s'),
             'direction_asymmetry':asym,'telemetry_field_correlations':correlations,'baseline_telemetry_samples':len(baseline)}
    high=[r for r in rows if abs(r['requested_omega_rad_s'])>=.30]
    if any(r.get('stop_class')=='C10B_STOP_RESPONSE_ABNORMAL' for r in rows):
        verdict='C10B_STOP_RESPONSE_ABNORMAL'
    elif not all(r.get('complete') for r in rows) or all(r['verdict']!='RELIABLE' for r in high):
        verdict='FAILED_DRIVE_ACTUATION'
    elif asym:
        verdict='DRIVE_DIRECTION_ASYMMETRY'
    elif all(r['verdict']=='RELIABLE' for r in rows):
        verdict='ROTATION_RESPONSE_HEALTHY'
    elif summary['minimum_reliable_rotation_omega'] is not None:
        verdict='DRIVE_ROTATION_DEADZONE_CONFIRMED'
    else:
        verdict='FAILED_DRIVE_ACTUATION'
    return {'rows':rows,'summary':summary,'verdict':verdict,
            'measurement_note':'T265 pose angular velocity and quaternion are internal device evidence, not independent raw IMU or external chassis truth. C10B fields are empirical patterns, not verified wheel/firmware-speed units. Still timestamp confirms a 0.30 s median window; onset/telemetry timestamps mark the start of a confirmed interval. Serial receive times include buffering and competing battery-service reads.'}


def render_speed_sweep(report,manifest):
    sweep=report['rotation_speed_sweep']; summary=sweep['summary']
    fmt=lambda x:'未知' if x is None else f'{x:.3f}'
    base=manifest['base_config']
    lines=['# Rotation low-speed response report','', '## Baseline','',
           '- commit: '+manifest['commit'],'- config sha: '+manifest['config_sha256'],
           '- protocol: '+base['drive']['protocol_mode'],
           '- track width: '+str(base['geometry']['drive_track_width_m'])+' m; firmware '+str(base['drive']['firmware_track_width_m'])+' m',
           '- battery: '+fmt(manifest.get('battery_voltage_v'))+' V',
           '- T265 serial: '+manifest['t265_serial']+'; angular acceleration: '+str(base['drive']['max_angular_accel_rad_s2'])+' rad/s²','',
           '## Stationary preflight','', 'PASS' if report.get('stationary_health',{}).get('passed') else 'FAIL','',
           '## Rotation response sweep','',
           '| requested omega | wheel mm/s | direction | start delay | steady omega | response ratio | stop delay | post-still yaw | verdict |',
           '|---:|---:|---|---:|---:|---:|---:|---:|---|']
    for r in sweep['rows']:
        lines.append('| '+ ' | '.join([fmt(r['requested_omega_rad_s']),fmt(r['wheel_mm_s']),r['direction'],fmt(r.get('start_delay_s')),fmt(r.get('measured_steady_omega_rad_s')),fmt(r.get('response_ratio')),fmt(r.get('t265_stop_delay_s')),fmt(r.get('yaw_after_t265_still_deg')),r['verdict']])+' |')
    lines+=['','Stop delay column: T265 still confirmation, seconds; angular speed rad/s; yaw degrees.','',
            '## Minimum reliable speed','', '- CCW: '+fmt(summary['minimum_reliable_ccw_omega']),'- CW: '+fmt(summary['minimum_reliable_cw_omega']),'- selected: '+fmt(summary['minimum_reliable_rotation_omega'])+' rad/s','',
            '## STOP response','', '- software zero: '+fmt(summary['worst_software_zero_delay_s'])+' s; first zero write '+fmt(summary['worst_first_zero_write_delay_s'])+' s',
            '- C10B zero delay: '+fmt(summary['worst_c10b_stop_delay_s'])+' s (stationary-pattern return)',
            '- T265 still delay: '+fmt(summary['worst_t265_stop_delay_s'])+' s',
            '- post-still yaw: '+fmt(summary['max_post_still_yaw_deg'])+'°','',
            '## Direction symmetry','', '- result: '+json.dumps(summary['direction_asymmetry'],ensure_ascii=False),'',
            '## Diagnosis','', '- verdict: '+report['verdict'], '- evidence: '+sweep['measurement_note'],
            '- worst start delay: '+fmt(summary['worst_start_delay_s'])+' s',
            '- STOP classes: '+json.dumps(dict(Counter(r.get('stop_class','NOT_RUN') for r in sweep['rows'])),ensure_ascii=False),
            '- error: '+str(report.get('error')),'', '## Changes','', '- code: existing auto_motion_diag.py rotation_speed_sweep only',
            '- config: unchanged; no controller or protocol changes','', '## Next recommended change','',
            ('下一轮仅评估旋转最小有效角速度 '+fmt(summary['minimum_reliable_rotation_omega'])+' rad/s。' if report['verdict']=='DRIVE_ROTATION_DEADZONE_CONFIRMED' else '下一轮仅同步核对 C10B 遥测与两侧电机实际响应。')]
    return '\n'.join(lines)+'\n'




def main(argv=None):
    parser=argparse.ArgumentParser(description="Read-only historical motion log analysis")
    parser.add_argument("--analyze-run")
    args=parser.parse_args(argv)
    if args.analyze_run:
        reanalyze(Path(args.analyze_run).resolve())
        return 0
    print("Live legacy diagnostics retired; use tools/closed_loop_motion.py",file=sys.stderr)
    return 1


if __name__=="__main__":
    raise SystemExit(main())
