from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import unittest


class ProductionImportBoundaryTests(unittest.TestCase):
    def test_shared_package_exports_resolve_after_legacy_removal(self) -> None:
        root = Path(__file__).resolve().parents[2]
        script = (
            f"import sys; sys.path.insert(0, {str(root / 'code')!r}); "
            "import components, config; "
            "[getattr(components, name) for name in components.__all__]; "
            "[getattr(config, name) for name in config.__all__]; "
            "assert not hasattr(components, 'AckermannDrive'); "
            "assert not hasattr(components, 'FrontSteeringServo'); "
            "assert not hasattr(config, 'load_car_config'); "
            "assert config.NavigationConfig.__module__ == 'config.v2_models'"
        )
        result = subprocess.run([sys.executable, "-c", script], cwd=root,
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stderr)

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
