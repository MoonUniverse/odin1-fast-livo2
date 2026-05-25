import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    output_run_dir = LaunchConfiguration("output_run_dir")
    save_translation_m = LaunchConfiguration("save_translation_m")
    save_rotation_deg = LaunchConfiguration("save_rotation_deg")

    config = os.path.join(pkg_share, "config", "odin.yaml")
    camera = os.path.join(pkg_share, "config", "camera_odin.yaml")
    rviz_config = os.path.join(pkg_share, "rviz_cfg", "fast_livo2.rviz")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("topic_report", default_value="false"),
        DeclareLaunchArgument("topic_report_dir", default_value="/tmp/fast_livo_topic_reports"),
        DeclareLaunchArgument("pcd_save", default_value="false"),
        DeclareLaunchArgument("image_save", default_value="false"),
        DeclareLaunchArgument("output_run_dir", default_value=""),
        DeclareLaunchArgument("save_translation_m", default_value="0.2"),
        DeclareLaunchArgument("save_rotation_deg", default_value="10.0"),
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            output="screen",
            parameters=[
                load_yaml(config),
                load_yaml(camera),
                {
                    "common.output_run_dir": output_run_dir,
                    "pcd_save.pcd_save_en": pcd_save,
                    "image_save.img_save_en": image_save,
                    "save_pose_gate.translation_m": ParameterValue(save_translation_m, value_type=float),
                    "save_pose_gate.rotation_deg": ParameterValue(save_rotation_deg, value_type=float),
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
