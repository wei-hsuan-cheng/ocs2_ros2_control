import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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


def generate_launch_description():
    params = _load_common_params()
    control_defaults = params.get("control", {})

    package_share = FindPackageShare("ocs2_ros2_control")

    control_urdf = PathJoinSubstitution([
        package_share,
        "description",
        "urdf",
        "ridgeback_ur5_ros2_control.urdf",
    ])

    robot_description_content = Command([
        FindExecutable(name="cat"),
        " ",
        control_urdf,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

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
    ]

    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")
    future_time_offset = LaunchConfiguration("futureTimeOffset")
    command_smoothing_alpha = LaunchConfiguration("commandSmoothingAlpha")

    ocs2_params = {
        "controller_manager.ros__parameters.ocs2_controller.ros__parameters.task_file": task_file,
        "controller_manager.ros__parameters.ocs2_controller.ros__parameters.lib_folder": lib_folder,
        "controller_manager.ros__parameters.ocs2_controller.ros__parameters.urdf_file": urdf_file,
        "controller_manager.ros__parameters.ocs2_controller.ros__parameters.future_time_offset": future_time_offset,
        "controller_manager.ros__parameters.ocs2_controller.ros__parameters.command_smoothing_alpha": command_smoothing_alpha,
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, ocs2_params],
        output="both",
    )

    ocs2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ocs2_controller", "--controller-manager", "/controller_manager"],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(declared_arguments + [
        control_node,
        joint_state_broadcaster,
        ocs2_controller_spawner,
    ])
