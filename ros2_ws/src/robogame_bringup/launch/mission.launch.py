"""Launch the full RoboGame 2026 mission on the real robot.

The mission node owns the whole core stack; cameras are expected from
their drivers (e.g. usb_cam / v4l2 nodes). Run from the repository
checkout so `config_dir` resolves:

  cd /home/pi/robogame
  ros2 launch robogame_bringup mission.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = LaunchConfiguration("config_dir")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_dir", default_value=".",
            description="repository root containing config/"),
        DeclareLaunchArgument(
            "field_file", default_value="real_field.yaml",
            description="field layout file inside config/"),
        DeclareLaunchArgument(
            "localization_camera_topic",
            default_value="/camera_localization/image_raw"),
        DeclareLaunchArgument(
            "block_camera_topic",
            default_value="/camera_block/image_raw"),
        DeclareLaunchArgument(
            "control_period", default_value="0.02"),

        Node(
            package="robogame_bringup",
            executable="mission_node",
            name="robogame_mission",
            output="screen",
            parameters=[{
                "config_dir": config_dir,
                "field_file": LaunchConfiguration("field_file"),
                "localization_camera_topic": LaunchConfiguration(
                    "localization_camera_topic"),
                "block_camera_topic": LaunchConfiguration(
                    "block_camera_topic"),
                "control_period": LaunchConfiguration("control_period"),
            }],
        ),
    ])
