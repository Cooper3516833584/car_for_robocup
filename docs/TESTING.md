# Testing Guide

> Status: documentation only; no test was executed for this change. Commands below
> are the ones documented in `AGENTS.md`.

## Test classes

| Class | Purpose | Where | Safe without hardware? |
|---|---|---|---|
| Unit tests | Kinematics, frames, config, protocol, watchdog | `code/test/test_*.py` | Yes, with `unittest` |
| Dry-run runtime | Fake sensors + fake drive, synthetic field goal | `code/main_robocup.py --mode dry-run` | Yes |
| Replay | Log replay and diagnostics | `code/components/pose_log_replay.py` | Yes (no motors) |
| Hardware probe | Sensors only, no autonomous navigation | `--mode hardware-probe` | Requires vehicle |
| Hardware mission | Full mission | `--mode hardware-mission` | **No** |

## Documented commands

```powershell
py -3 -m compileall -q code tools
py -3 -m unittest discover -s code/test -p "test_*.py"
git diff --check
```

Dry run (no vehicle):

```powershell
py -3 code\main_robocup.py --config configs\robocup_diffdrive.example.toml --mode dry-run
```

## What was not run here

- `unittest` suite: **NOT RUN** — this documentation host lacks the runtime
  dependencies and the checkout is documentation-only.
- `dry-run`: **NOT RUN** — would still import the full runtime stack.
- Any `hardware-*` mode: **NOT RUN** by policy.

## Rules

- Hardware tests are never part of default discovery.
- A passing unit test is not evidence of vehicle behaviour; record measured runs in
  `docs/HARDWARE_MEASUREMENTS.md` or dated test logs.
- Do not add a `--force` bypass for readiness checks (see `AGENTS.md`).
