#!/usr/bin/env python3
"""Read-only SSH summary of wall turns and payload failures; never opens hardware."""
import argparse
import json
from pathlib import Path
import subprocess


REMOTE = r'''
import json, math
from collections import deque
from pathlib import Path
events = []
for line in Path('/home/radxa/car/logs/competition/events.jsonl').read_text().splitlines():
    try:
        events.append(json.loads(line))
    except ValueError:
        pass
starts = [i for i, e in enumerate(events) if e.get('type') == 'competition_motion_profile']
events = events[starts[-1]:] if starts else events
important = {'wall_runtime_started', 'competition_target_centered', 'payload_release_failed',
             'wall_action_start', 'wall_action_done', 'competition_route_leg_start',
             'competition_route_leg_done', 'payload_detour_failed', 'payload_drop',
             'payload_reference_locked', 'task_board_counts'}
result = {'events': [e for e in events if e.get('type') in important], 'turns': {},
          'first_terminal': {}, 'follow_end': {}}
follow_end = {}
for e in events:
    if e.get('type') == 'wall_control' and e.get('action') == 'follow_right_wall':
        phase = e.get('phase', '')
        follow_end.setdefault(phase, deque(maxlen=60)).append(e)
        if e.get('terminal_visible') and phase not in result['first_terminal']:
            result['first_terminal'][phase] = e
    if e.get('type') != 'wall_control' or e.get('action') != 'turn_to_wall_bearing':
        continue
    phase = e.get('phase', '')
    result['turns'].setdefault(phase, []).append([
        round(e['t'], 3),
        round(math.degrees(e['error_rad']), 2) if 'error_rad' in e else None,
        round(e['reference_rho_base_m'], 3) if 'reference_rho_base_m' in e else None,
        e.get('scan_seq'), e.get('stable_count'), e.get('reason')])
result['follow_end'] = {k: list(v) for k, v in follow_end.items()}
print(json.dumps(result, ensure_ascii=True))
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='radxa_iqoo', help='SSH config alias')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        completed = subprocess.run(
            ['ssh', '-T', '-C', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
             '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2',
             '-o', 'IPQoS=none', args.host, 'python3 -'],
            input=REMOTE, text=True, capture_output=True, timeout=45, check=True)
    except subprocess.CalledProcessError as exc:
        raise SystemExit(exc.stderr.strip() or f'SSH failed: {exc.returncode}') from exc
    except subprocess.TimeoutExpired as exc:
        raise SystemExit('SSH summary timed out after 45 seconds') from exc
    report = json.loads(completed.stdout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({'output': str(args.output.resolve()),
                      'events': report['events'],
                      'turn_samples': {k: len(v) for k, v in report['turns'].items()}}, indent=2))


if __name__ == '__main__':
    main()
