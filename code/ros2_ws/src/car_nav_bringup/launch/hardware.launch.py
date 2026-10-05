"""Optional D500 sensor bridge. Chassis control belongs to main_robocup.py."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_hardware", default_value="false"),
        DeclareLaunchArgument("radar_x_m"), DeclareLaunchArgument("radar_y_m"), DeclareLaunchArgument("radar_yaw_rad"),
        Node(package="car_ros_bridge", executable="d500_localization_node", parameters=[{"use_hardware": LaunchConfiguration("use_hardware"), "radar_x_m": LaunchConfiguration("radar_x_m"), "radar_y_m": LaunchConfiguration("radar_y_m"), "radar_yaw_rad": LaunchConfiguration("radar_yaw_rad")}]),
    ])
