"""Hardware-free tests for the Linux sysfs PWM HAL using a fake sysfs tree.

Never touches /sys/class/pwm on the host.
"""

from __future__ import annotations

from pathlib import Path
import os
import shutil
import sys
import unittest
import unittest.mock
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hal.pwm import LinuxSysfsPWMOutput, PWMBackendError, resolve_chip


class LinuxSysfsPWMOutputTests(unittest.TestCase):
    def setUp(self) -> None:
        self.root = (
            Path(__file__).resolve().parent / f"_hal_tmp_{uuid.uuid4().hex}"
        )
        self.root.mkdir()
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)

    def _make_chip(self, name: str = "pwmchip0") -> Path:
        chip = self.root / name
        chip.mkdir()
        (chip / "export").write_text("", encoding="ascii")
        return chip

    def _make_pwm(self, chip: Path) -> Path:
        pwm = chip / "pwm0"
        pwm.mkdir()
        (pwm / "enable").write_text("0\n", encoding="ascii")
        (pwm / "period").write_text("0\n", encoding="ascii")
        (pwm / "polarity").write_text("normal\n", encoding="ascii")
        (pwm / "duty_cycle").write_text("0\n", encoding="ascii")
        return pwm

    def test_start_configures_period_and_polarity_and_export(self) -> None:
        chip = self._make_chip()
        pwm = self._make_pwm(chip)
        (chip / "export").write_text("", encoding="ascii")

        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
            period_ns=20_000_000,
            polarity="normal",
        )
        output.start()

        self.assertEqual((pwm / "period").read_text().strip(), "20000000")
        self.assertEqual((pwm / "polarity").read_text().strip(), "normal")
        self.assertTrue(output.is_running)

    def test_set_pulse_us_writes_ns_duty_and_enables(self) -> None:
        chip = self._make_chip()
        pwm = self._make_pwm(chip)
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
            period_ns=20_000_000,
            polarity="normal",
        )
        output.start()

        output.set_pulse_us(1580)

        self.assertEqual((pwm / "duty_cycle").read_text().strip(), "1580000")
        self.assertEqual((pwm / "enable").read_text().strip(), "1")

    def test_disable_writes_enable_zero(self) -> None:
        chip = self._make_chip()
        pwm = self._make_pwm(chip)
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
        )
        output.start()
        output.set_pulse_us(1500)
        output.disable()

        self.assertEqual((pwm / "enable").read_text().strip(), "0")

    def test_export_creates_missing_pwm_channel(self) -> None:
        chip = self._make_chip()
        (chip / "export").write_text("", encoding="ascii")
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
        )
        with unittest.mock.patch("hal.pwm.time.sleep"):
            with self.assertRaisesRegex(PWMBackendError, "export"):
                output.start()

        # The export request was written even though the fake sysfs never
        # creates the channel directory.
        self.assertEqual((chip / "export").read_text().strip(), "0")

    def test_operations_before_start_raise(self) -> None:
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="pwmchip0",
            channel=0,
        )
        with self.assertRaises(PWMBackendError):
            output.set_pulse_us(1500)
        with self.assertRaises(PWMBackendError):
            output.disable()

    def test_missing_chip_raises_clear_error(self) -> None:
        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root,
            chip_device_match="does-not-exist",
            channel=0,
        )
        with self.assertRaisesRegex(PWMBackendError, "does-not-exist"):
            output.start()


class ResolveChipTests(unittest.TestCase):
    """Chip selection must survive probe-order renumbering on real boards."""

    def setUp(self) -> None:
        self.root = (
            Path(__file__).resolve().parent / f"_hal_tmp_{uuid.uuid4().hex}"
        )
        self.root.mkdir()
        self.addCleanup(shutil.rmtree, self.root, ignore_errors=True)

    def _make_chip(self, name: str, node: str | None = None) -> Path:
        """Create ``pwmchipN``; ``node`` adds the ``device`` symlink target."""

        chip = self.root / name
        chip.mkdir()
        (chip / "export").write_text("", encoding="ascii")
        if node is not None:
            device = self.root / node
            device.mkdir()
            try:
                (chip / "device").symlink_to(device, target_is_directory=True)
            except (OSError, NotImplementedError) as exc:
                # Windows needs Developer Mode or admin rights to create
                # symlinks; these cases only exist on real Linux boards.
                self.skipTest(f"cannot create a device symlink here: {exc}")
        return chip

    def test_literal_chip_name_wins(self) -> None:
        self._make_chip("pwmchip0", "fd8b0000.pwm")
        self._make_chip("pwmchip3", "febd0030.pwm")

        # Selection normalises through the ``device`` symlink, so compare the
        # resolved path: on some Python versions ``Path.resolve`` keeps the last
        # symlink component, on others it expands it.
        self.assertEqual(
            resolve_chip(self.root, "pwmchip3").resolve(),
            (self.root / "pwmchip3").resolve(),
        )

    def test_device_tree_node_name_is_probe_order_independent(self) -> None:
        # Two controllers can collapse onto the same sysfs class name, so only
        # the device-tree address identifies the pin. Resolve it either way.
        self._make_chip("pwmchip1", "febd0030.pwm")
        self._make_chip("pwmchip0", "fd8b0000.pwm")

        selected = resolve_chip(self.root, "febd0030.pwm")
        self.assertEqual(selected.name, "pwmchip1")
        self.assertEqual(os.readlink(selected / "device"), str(self.root / "febd0030.pwm"))

    def test_node_name_matches_through_symlink_not_substring(self) -> None:
        # A substring search would accept "pwmchip0" here because its device
        # target path contains "febd0030.pwm"; the node name must match exactly,
        # so a child of that node directory must not be selected.
        chip = self._make_chip("pwmchip0", "febd0030.pwm")
        (self.root / "febd0030.pwm" / "child").mkdir()
        (chip / "device").unlink()
        try:
            (chip / "device").symlink_to(
                self.root / "febd0030.pwm" / "child", target_is_directory=True
            )
        except (OSError, NotImplementedError) as exc:
            self.skipTest(f"cannot create a device symlink here: {exc}")

        with self.assertRaises(PWMBackendError):
            resolve_chip(self.root, "febd0030.pwm")

    def test_first_matching_selector_in_a_fallback_list_is_used(self) -> None:
        self._make_chip("pwmchip0", "fd8b0000.pwm")
        self._make_chip("pwmchip2", "febd0030.pwm")

        self.assertEqual(
            resolve_chip(self.root, ("pwmchip9", "febd0030.pwm", "pwmchip0")).name,
            "pwmchip2",
        )
        self.assertEqual(
            resolve_chip(self.root, ("pwmchip0", "febd0030.pwm")).name,
            "pwmchip0",
        )

    def test_substring_selector_still_supports_unknown_boards(self) -> None:
        # The last-resort selector matches a component of the resolved device
        # path, for boards whose node names are not known in advance.
        self._make_chip("pwmchip1", "some-vendor-pwm.pwm")

        self.assertEqual(resolve_chip(self.root, "some-vendor").name, "pwmchip1")

    def test_empty_selector_list_is_rejected(self) -> None:
        self._make_chip("pwmchip0")
        with self.assertRaises(PWMBackendError):
            resolve_chip(self.root, ())

    def test_absent_node_name_does_not_fall_back_to_a_numeric_chip(self) -> None:
        # Real-trap regression: on the ROCK 5A, pwmchip0 is D500's fd8b0000.pwm.
        # Asking only for the unavailable febd0030.pwm must fail instead of
        # silently binding that unrelated controller.
        self._make_chip("pwmchip0", "fd8b0000.pwm")

        with self.assertRaises(PWMBackendError):
            resolve_chip(self.root, "febd0030.pwm")

    def test_start_exports_on_the_resolved_chip(self) -> None:
        chip = self._make_chip("pwmchip3", "febd0030.pwm")
        pwm = chip / "pwm0"
        pwm.mkdir()
        (pwm / "enable").write_text("0\n", encoding="ascii")
        (pwm / "period").write_text("0\n", encoding="ascii")
        (pwm / "polarity").write_text("normal\n", encoding="ascii")
        (pwm / "duty_cycle").write_text("0\n", encoding="ascii")

        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root, chip_device_match="febd0030.pwm", channel=0
        )
        output.start()
        output.set_pulse_us(1500)

        self.assertEqual((chip / "pwm0" / "duty_cycle").read_text().strip(), "1500000")
        self.assertEqual((chip / "pwm0" / "period").read_text().strip(), "20000000")

    def test_explicit_pwm_path_skips_chip_lookup(self) -> None:
        chip = self.root / "anywhere"
        chip.mkdir()
        pwm = chip / "pwm2"
        pwm.mkdir()
        (pwm / "enable").write_text("0\n", encoding="ascii")
        (pwm / "period").write_text("0\n", encoding="ascii")
        (pwm / "polarity").write_text("normal\n", encoding="ascii")
        (pwm / "duty_cycle").write_text("0\n", encoding="ascii")

        output = LinuxSysfsPWMOutput(
            sysfs_root=self.root / "no-such-root",
            chip_device_match="nothing-matches",
            pwm_path=pwm,
        )
        output.start()
        output.set_pulse_us(1500)

        self.assertEqual((pwm / "duty_cycle").read_text().strip(), "1500000")


if __name__ == "__main__":
    unittest.main()
