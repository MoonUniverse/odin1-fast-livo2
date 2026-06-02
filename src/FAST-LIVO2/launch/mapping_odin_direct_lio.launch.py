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
    fast_livo_share = get_package_share_directory("fast_livo")
    odin_share = get_package_share_directory("odin_ros_driver")

    rviz = LaunchConfiguration("rviz")
    topic_report = LaunchConfiguration("topic_report")
    topic_report_dir = LaunchConfiguration("topic_report_dir")
    pcd_save = LaunchConfiguration("pcd_save")
    final_map_save = LaunchConfiguration("final_map_save")
    pcd_async_save = LaunchConfiguration("pcd_async_save")
    pcd_async_queue_size = LaunchConfiguration("pcd_async_queue_size")
    output_run_dir = LaunchConfiguration("output_run_dir")
    save_translation_m = LaunchConfiguration("save_translation_m")
    save_rotation_deg = LaunchConfiguration("save_rotation_deg")
    odin_config = LaunchConfiguration("odin_config")
    recorddata = LaunchConfiguration("recorddata")
    recorddata_dir = LaunchConfiguration("recorddata_dir")
    publish_debug_topics = LaunchConfiguration("publish_debug_topics")

    config = os.path.join(fast_livo_share, "config", "odin.yaml")
    camera = os.path.join(fast_livo_share, "config", "camera_odin.yaml")
    rviz_config = os.path.join(fast_livo_share, "rviz_cfg", "fast_livo2.rviz")
    default_odin_config = os.path.join(odin_share, "config", "control_command_fast_livo.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("topic_report", default_value="false"),
        DeclareLaunchArgument("topic_report_dir", default_value="/tmp/fast_livo_lio_topic_reports"),
        DeclareLaunchArgument("pcd_save", default_value="false"),
        DeclareLaunchArgument("final_map_save", default_value="false"),
        DeclareLaunchArgument("pcd_async_save", default_value="true"),
        DeclareLaunchArgument("pcd_async_queue_size", default_value="8"),
        DeclareLaunchArgument("output_run_dir", default_value=""),
        DeclareLaunchArgument("save_translation_m", default_value="0.2"),
        DeclareLaunchArgument("save_rotation_deg", default_value="10.0"),
        DeclareLaunchArgument("odin_config", default_value=default_odin_config),
        DeclareLaunchArgument("recorddata", default_value="false"),
        DeclareLaunchArgument("recorddata_dir", default_value=""),
        DeclareLaunchArgument("publish_debug_topics", default_value="false"),
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            output="screen",
            parameters=[
                load_yaml(config),
                load_yaml(camera),
                {
                    "common.input_source": "odin_direct",
                    "common.img_en": 0,
                    "common.output_run_dir": output_run_dir,
                    "pcd_save.pcd_save_en": pcd_save,
                    "pcd_save.final_map_save_en": final_map_save,
                    "pcd_save.async_save_en": pcd_async_save,
                    "pcd_save.async_queue_size": ParameterValue(pcd_async_queue_size, value_type=int),
                    "image_save.img_save_en": False,
                    "save_pose_gate.translation_m": ParameterValue(save_translation_m, value_type=float),
                    "save_pose_gate.rotation_deg": ParameterValue(save_rotation_deg, value_type=float),
                    "odin_direct.config_file": odin_config,
                    "odin_direct.recorddata": recorddata,
                    "odin_direct.recorddata_dir": recorddata_dir,
                    "odin_direct.publish_debug_topics": publish_debug_topics,
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
