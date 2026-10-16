"""Run offline unit tests from the repository root, Windows-friendly.

``python -m unittest discover -s code/test`` inserts ``code/test`` lazily while
importing, so a single-file run has to be launched as ``discover -p NAME``.
Unittest's pattern is one glob (no braces, no comma lists), so several files are
run as several discoveries and the results are aggregated here.

Usage:
    py -3 tools/run_offline_tests.py                            # whole suite
    py -3 tools/run_offline_tests.py test_wall_observer.py      # one file
    py -3 tools/run_offline_tests.py test_wall_observer.py test_wall_navigation.py
"""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys

REPO = Path(__file__).resolve().parents[1]
TEST_DIR = REPO / "code" / "test"
FAILURE = re.compile(r"^(?:ERROR|FAIL): (\S+)")


def run_one(pattern, verbose):
    command = [sys.executable, "-m", "unittest", "discover", "-s", "code/test",
               "-p", pattern]
    if verbose:
        command.append("-v")
    completed = subprocess.run(command, cwd=REPO, capture_output=True, text=True,
                               errors="replace")
    return completed.stdout + completed.stderr


def main(argv):
    verbose = "-v" in argv
    names = [arg for arg in argv if arg != "-v"]
    if not names:
        # One discovery over the whole suite.
        names = None
    else:
        names = [name if name.endswith(".py") else name + ".py" for name in names]
        for name in names:
            if not (TEST_DIR / name).is_file():
                print(f"no such test module: {name}", file=sys.stderr)
                return 2

    patterns = [name for name in names] if names else ["test_*.py"]
    failures, ran, failed_files = [], 0, []
    for pattern in patterns:
        output = run_one(pattern, verbose)
        if verbose:
            print(output, end="" if output.endswith("\n") else "\n")
        for line in output.splitlines():
            match = FAILURE.match(line.strip())
            if match:
                failures.append(match.group(1))
        summary = [line for line in output.splitlines() if line.startswith("Ran ")]
        if summary:
            ran += int(summary[-1].split()[1])
        # unittest prints "OK" only when a run is clean; anything else that is
        # not a clean OK (including a discovery that found nothing) is a failure.
        clean = any(line.strip().startswith("OK") for line in output.splitlines())
        if clean and "FAILED" not in output:
            print(f"[ ok ] {pattern} ({summary[-1] if summary else 'no summary'})")
        else:
            failed_files.append(pattern)
            print(f"[FAIL] {pattern} ({summary[-1] if summary else 'no summary'})")

    print(f"\nTotal tests run: {ran}")
    if failures:
        print("Distinct failing tests: %d" % len(set(failures)))
        for item in sorted(set(failures)):
            print("  -", item)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
