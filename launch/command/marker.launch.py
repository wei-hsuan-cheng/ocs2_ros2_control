import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

common_dir = Path(__file__).resolve().parent.parent / "common"
if str(common_dir) not in sys.path:
    sys.path.insert(0, str(common_dir))

from config_utils import load_common_params, load_default_frame_from_robot_config  # noqa: E402


def _bool_to_str(value):
    return "true" if value else "false"


def generate_launch_description():
    params = load_common_params()
    marker_defaults = params.get("command", {}).get("marker", {})
    default_global_frame = load_default_frame_from_robot_config()

    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value="", description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value="", description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value="", description="URDF passed to visualization nodes."),
        DeclareLaunchArgument("initialPoseFile", default_value="", description="Initial pose YAML for marker/visualization."),
        DeclareLaunchArgument(
            "markerPublishRate",
            default_value=str(marker_defaults.get("publish_rate", 100.0)),
            description="Interactive marker publish rate.",
        ),
        DeclareLaunchArgument(
            "enableJoystick",
            default_value=_bool_to_str(marker_defaults.get("enable_joystick", False)),
            description="Enable joystick control for the marker.",
        ),
        DeclareLaunchArgument(
            "enableAutoPosition",
            default_value=_bool_to_str(marker_defaults.get("enable_auto_position", False)),
            description="Enable automatic marker repositioning.",
        ),
        DeclareLaunchArgument(
            "markerGlobalFrame",
            default_value=str(default_global_frame),
            description="Frame used for the interactive marker.",
        ),
    ]

    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")
    initial_pose_file = LaunchConfiguration("initialPoseFile")

    marker_node = Node(
        package="ocs2_ros2_control",
        executable="mobile_manipulator_marker_target",
        name="mobile_manipulator_marker_target",
        parameters=[{
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
            "initialPoseFile": initial_pose_file,
            "markerPublishRate": LaunchConfiguration("markerPublishRate"),
            "enableJoystick": LaunchConfiguration("enableJoystick"),
            "enableAutoPosition": LaunchConfiguration("enableAutoPosition"),
            "markerGlobalFrame": LaunchConfiguration("markerGlobalFrame"),
        }],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [marker_node])
