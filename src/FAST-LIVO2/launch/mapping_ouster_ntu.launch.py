import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as yaml_file:
        return yaml.safe_load(yaml_file)


def generate_launch_description():
    pkg_share = get_package_share_directory("fast_livo")
    rviz = LaunchConfiguration("rviz")

    config = os.path.join(pkg_share, "config", "NTU_VIRAL.yaml")
    camera = os.path.join(pkg_share, "config", "camera_NTU_VIRAL.yaml")
    rviz_config = os.path.join(pkg_share, "rviz_cfg", "ntu_viral.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            output="screen",
            parameters=[load_yaml(config), load_yaml(camera)],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
