from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import unittest


class ProductionImportBoundaryTests(unittest.TestCase):
    def test_robocup_entry_does_not_import_ackermann_or_steering(self) -> None:
        root = Path(__file__).resolve().parents[2]
        script = (
            "import sys; "
            f"sys.path.insert(0, {str(root / 'code')!r}); "
            "import main_robocup; "
            "assert 'components.ackermann_drive' not in sys.modules; "
            "assert 'components.steering_servo' not in sys.modules; "
            "assert 'config.factory' not in sys.modules"
        )
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=root,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
