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
    topic_report = LaunchConfiguration("topic_report")
    topic_report_dir = LaunchConfiguration("topic_report_dir")
    pcd_save = LaunchConfiguration("pcd_save")
    image_save = LaunchConfiguration("image_save")

    config = os.path.join(pkg_share, "config", "odin.yaml")
    camera = os.path.join(pkg_share, "config", "camera_odin.yaml")
    rviz_config = os.path.join(pkg_share, "rviz_cfg", "fast_livo2.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("topic_report", default_value="false"),
        DeclareLaunchArgument("topic_report_dir", default_value="/tmp/fast_livo_topic_reports"),
        DeclareLaunchArgument("pcd_save", default_value="false"),
        DeclareLaunchArgument("image_save", default_value="false"),
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            output="screen",
            parameters=[
                load_yaml(config),
                load_yaml(camera),
                {
                    "pcd_save.pcd_save_en": pcd_save,
                    "image_save.img_save_en": image_save,
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            condition=IfCondition(rviz),
        ),
        Node(
            package="topic_monitor",
            executable="topic_report",
            name="topic_report",
            output="screen",
            parameters=[{"output_dir": topic_report_dir}],
            condition=IfCondition(topic_report),
        ),
    ])
