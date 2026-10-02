"""Run only the optional SLAM sidecar; the robot runtime is started separately."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(__file__).resolve().parents[1] / "configs" / "slam_toolbox_rock5a.yaml"
    return LaunchDescription([
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[str(config)],
        ),
    ])
