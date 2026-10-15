#!/usr/bin/env python3
"""Remove only an obsolete relay table from a deployed local TOML profile."""
from datetime import datetime, timezone
from pathlib import Path
import re
import shutil
import sys
import tomllib

ROOT = Path(__file__).resolve().parents[1]


def without_relay(source):
    before = tomllib.loads(source)
    if 'relay' not in before.get('devices', {}):
        return source
    lines = source.splitlines(keepends=True)
    start = next(i for i, line in enumerate(lines)
                 if re.fullmatch(r'\s*\[devices\.relay\]\s*(?:#.*)?', line.strip()))
    end = next((i for i in range(start + 1, len(lines))
                if lines[i].lstrip().startswith('[')), len(lines))
    # Keep comments belonging to the next section, and all unrelated content.
    while end > start + 1 and (not lines[end - 1].strip() or
                              lines[end - 1].lstrip().startswith('#')):
        end -= 1
    result = ''.join(lines[:start] + lines[end:])
    del before['devices']['relay']
    if tomllib.loads(result) != before:
        raise ValueError('refusing migration: unrelated profile settings would change')
    return result


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    path = Path(args[0]) if args else ROOT / 'configs/robocup_diffdrive.toml'
    source = path.read_text(encoding='utf-8-sig')
    updated = without_relay(source)
    if source == updated:
        print('profile already has no retired relay table:', path)
        return 0
    backups = ROOT / 'logs/config-backups'
    backups.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S-%f')
    backup = backups / (path.name + '.before-sg90-' + stamp + '.bak')
    shutil.copy2(path, backup)
    if backup.read_bytes() != path.read_bytes():
        raise RuntimeError('profile backup verification failed')
    path.write_text(updated, encoding='utf-8')
    print('retired relay table removed; other settings preserved:', path)
    print('verified backup:', backup)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
