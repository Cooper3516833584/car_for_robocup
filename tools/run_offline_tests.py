"""Run offline unit tests from the repository root, Windows-friendly.

``python -m unittest discover -s code/test`` inserts ``code/test`` lazily while
importing, so a single-file run has to be launched as ``discover -p NAME``.
This helper runs one discovery per requested file (or the whole suite) and
prints a compact failure summary, so targeted iteration is one command.

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


def run(module_name, verbose):
    command = [sys.executable, "-m", "unittest", "discover", "-s", "code/test",
               "-p", module_name]
    if verbose:
        command.append("-v")
    completed = subprocess.run(command, cwd=REPO, capture_output=True, text=True,
                               errors="replace")
    return completed.returncode, completed.stdout + completed.stderr


def main(argv):
    verbose = "-v" in argv
    names = [arg for arg in argv if arg != "-v"]
    if not names:
        names = sorted(path.name for path in TEST_DIR.glob("test_*.py"))
    names = [name if name.endswith(".py") else name + ".py" for name in names]
    for name in names:
        if not (TEST_DIR / name).is_file():
            print(f"no such test module: {name}", file=sys.stderr)
            return 2

    failures, ran = [], 0
    for name in names:
        code, output = run(name, verbose)
        if verbose:
            print(output, end="" if output.endswith("\n") else "\n")
        for line in output.splitlines():
            match = FAILURE.match(line.strip())
            if match:
                failures.append(match.group(1))
        summary = [line for line in output.splitlines() if line.startswith("Ran ")]
        ran += int(summary[-1].split()[1]) if summary else 0
        if code != 0:
            print(f"[FAIL] {name}")
        else:
            print(f"[ ok ] {name} ({summary[-1] if summary else 'no summary'})")
    print(f"\nTotal tests run: {ran}")
    if failures:
        print("Distinct failing tests: %d" % len(set(failures)))
        for item in sorted(set(failures)):
            print("  -", item)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
