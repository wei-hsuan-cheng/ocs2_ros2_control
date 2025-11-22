import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart

common_dir = Path(__file__).resolve().parent.parent / "common"
if str(common_dir) not in sys.path:
    sys.path.insert(0, str(common_dir))

from config_utils import load_common_params, load_default_frame_from_robot_config  # noqa: E402


def generate_launch_description():
    params = load_common_params()
    control_defaults = params.get("control", {})
    default_global_frame = load_default_frame_from_robot_config()

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
