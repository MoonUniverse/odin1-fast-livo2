import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("odin_ros_driver")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(package_dir, "config", "control_command_imu_only.yaml"),
        description="Path to the Odin config YAML file for IMU-only calibration",
    )

    host_sdk_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
    )

    return LaunchDescription([
        config_file_arg,
        host_sdk_node,
    ])
