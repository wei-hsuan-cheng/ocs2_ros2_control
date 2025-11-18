import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _load_common_params():
    params_path = os.path.join(
        get_package_share_directory("ocs2_ros2_control"),
        "config",
        "common_params.yaml",
    )
    with open(params_path, "r") as f:
        return yaml.safe_load(f)


def _load_default_frame_from_robot_config():
    cfg_share = get_package_share_directory("ocs2_ros2_control")
    cfg_path = os.path.join(cfg_share, "config", "ridgeback_ur5", "ridgeback_ur5_ros2_control.yaml")
    default_frame = "odom"
    try:
        with open(cfg_path, "r") as f:
            diff_cfg = yaml.safe_load(f) or {}
        default_frame = (
            diff_cfg.get("ridgeback_base_controller", {})
            .get("ros__parameters", {})
            .get("odom_frame_id", default_frame)
        )
    except Exception:
        default_frame = "odom"
    return default_frame


def _bool_to_str(value):
    return "true" if value else "false"


def generate_launch_description():
    params = _load_common_params()
    marker_defaults = params.get("command", {}).get("marker", {})
    default_frame = _load_default_frame_from_robot_config()

    package_share = FindPackageShare("ocs2_ros2_control")

    initial_pose_default = PathJoinSubstitution([
        package_share,
        "config",
        "ridgeback_ur5",
        "ridgeback_ur5_initial_pose.yaml",
    ])

    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value="", description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value="", description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value="", description="URDF passed to visualization nodes."),
        DeclareLaunchArgument("initialPoseFile", default_value=initial_pose_default, description="Initial pose YAML for marker/visualization."),
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
            "markerFrame",
            default_value=str(default_frame),
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
            "markerFrame": LaunchConfiguration("markerFrame"),
        }],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [marker_node])
