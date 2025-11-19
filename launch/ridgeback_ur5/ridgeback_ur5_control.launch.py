import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
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


def generate_launch_description():
    params = _load_common_params()
    control_defaults = params.get("control", {})
    default_global_frame = _load_default_frame_from_robot_config()

    package_share = FindPackageShare("ocs2_ros2_control")

    controller_config = PathJoinSubstitution([
        package_share,
        "config",
        "ridgeback_ur5",
        "ridgeback_ur5_ros2_control.yaml",
    ])

    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value="", description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value="", description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value="", description="URDF passed to visualization nodes."),
        DeclareLaunchArgument(
            "futureTimeOffset",
            default_value=str(control_defaults.get("future_time_offset", 0.02)),
            description="Lookahead time (s) when evaluating MPC policy.",
        ),
        DeclareLaunchArgument(
            "commandSmoothingAlpha",
            default_value=str(control_defaults.get("command_smoothing_alpha", 1.0)),
            description="Blending factor for consecutive MPC commands.",
        ),
        DeclareLaunchArgument(
            "globalFrame",
            default_value=str(default_global_frame),
            description="Global frame used for odometry and OCS2 visualization.",
        ),
    ]

    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")
    future_time_offset = LaunchConfiguration("futureTimeOffset")
    command_smoothing_alpha = LaunchConfiguration("commandSmoothingAlpha")
    global_frame = LaunchConfiguration("globalFrame")

    robot_description_content = Command([
        FindExecutable(name="cat"),
        " ",
        urdf_file,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, 
                    controller_config, 
                    {
                    "taskFile": task_file,
                    "libFolder": lib_folder,
                    "urdfFile": urdf_file,
                    "futureTimeOffset": future_time_offset,
                    "commandSmoothingAlpha": command_smoothing_alpha,
                    "globalFrame": global_frame,
                    }],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/ridgeback_base_controller/cmd_vel", "/cmd_vel"),
            ("/ridgeback_base_controller/odom", "/odom"),
        ],
    )

    ocs2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ocs2_controller", "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ridgeback_base_controller", "--controller-manager", "/controller_manager"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                joint_state_broadcaster,
                diff_drive_spawner,
                ocs2_controller_spawner,
            ],
        )
    )

    return LaunchDescription(declared_arguments + [
        control_node,
        load_controller,
    ])
