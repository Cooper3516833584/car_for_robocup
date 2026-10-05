"""Verify the optional ROS2 launch cannot start the retired chassis stack."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType
import unittest
from unittest.mock import patch


class SensorOnlyLaunchTests(unittest.TestCase):
    def test_launch_only_contains_d500_and_hardware_is_opt_in(self):
        launch = ModuleType("launch")
        launch.LaunchDescription = lambda actions: actions
        actions = ModuleType("launch.actions")
        actions.DeclareLaunchArgument = lambda name, **kwargs: {"argument": name, **kwargs}
        substitutions = ModuleType("launch.substitutions")
        substitutions.LaunchConfiguration = lambda name: name
        ros_actions = ModuleType("launch_ros.actions")
        ros_actions.Node = lambda **kwargs: kwargs
        modules = {"launch": launch, "launch.actions": actions,
                   "launch.substitutions": substitutions, "launch_ros": ModuleType("launch_ros"),
                   "launch_ros.actions": ros_actions}
        path = Path(__file__).resolve().parents[1] / "ros2_ws/src/car_nav_bringup/launch/hardware.launch.py"
        spec = importlib.util.spec_from_file_location("sensor_launch_under_test", path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
            description = module.generate_launch_description()
        nodes = [action for action in description if "executable" in action]
        self.assertEqual([node["executable"] for node in nodes], ["d500_localization_node"])
        hardware = next(action for action in description if action.get("argument") == "use_hardware")
        self.assertEqual(hardware["default_value"], "false")


if __name__ == "__main__":
    unittest.main()
