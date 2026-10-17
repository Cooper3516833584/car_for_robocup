#!/usr/bin/env python3
"""Give one micromamba run a fresh process cache, preserving the child's cache.

The systemd RuntimeDirectory owns cleanup, including after a crash or reboot.
Never inspect or repair a shared process registry on the listener startup path:
micromamba 2.9 aborts when any old proc/*.json is empty or malformed.
"""

import argparse
import os
from pathlib import Path
import tempfile


def prepare_launch(cache_root, mamba, prefix, command, *, environ=None):
    if not command:
        raise ValueError('a child command is required')
    environment = dict(os.environ if environ is None else environ)
    root = Path(cache_root)
    root.mkdir(mode=0o700, parents=True, exist_ok=True)
    # A new directory on EVERY start also survives a failed prior launch even
    # if RuntimeDirectory cleanup has not happened yet. No old JSON is parsed.
    cache = tempfile.mkdtemp(prefix='launch-', dir=root)
    original_cache = environment.get('XDG_CACHE_HOME')
    environment['XDG_CACHE_HOME'] = cache
    restore_cache = (['-u', 'XDG_CACHE_HOME'] if original_cache is None
                     else ['XDG_CACHE_HOME=' + original_cache])
    argv = [str(mamba), 'run', '-p', str(prefix), '/usr/bin/env',
            *restore_cache, *command]
    return argv, environment


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache-root', required=True)
    parser.add_argument('mamba')
    parser.add_argument('prefix')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    if not args.command:
        parser.error('a child command is required')
    command, environment = prepare_launch(args.cache_root, args.mamba,
                                          args.prefix, args.command)
    print('micromamba isolated process cache: ' + environment['XDG_CACHE_HOME'], flush=True)
    # Replace this launcher: systemd still supervises micromamba directly and
    # KillMode=control-group continues to cover the exact same child process.
    os.execvpe(command[0], command, environment)


if __name__ == '__main__':
    main()
