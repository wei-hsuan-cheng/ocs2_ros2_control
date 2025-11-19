import os
import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _urdf_from_xacro_path():
    """Generate ridgeback_ur5.urdf from the xacro into the config folder."""
    share_dir = get_package_share_directory("ocs2_ros2_control")
    xacro_in = os.path.join(share_dir, "description", "ridgeback_ur5", "urdf", "ridgeback_ur5.urdf.xacro")
    urdf_out = os.path.join(share_dir, "description", "ridgeback_ur5", "urdf", "ridgeback_ur5.urdf")
    initial_pose = os.path.join(share_dir, "config", "ridgeback_ur5", "ridgeback_ur5_initial_pose.yaml")

    try:
        subprocess.run(
            [
                "xacro",
                xacro_in,
                f"initial_pose_file:={initial_pose}",
                "ros2_control_mode:=true",
                "-o",
                urdf_out,
            ],
            check=True,
        )
        print(f"[ridgeback_ur5_bringup] Generate URDF from xacro")

    except Exception as exc:  # noqa: BLE001
        print(f"[ridgeback_ur5_bringup] Failed to generate URDF from xacro: {exc}")

    return urdf_out


def generate_launch_description():
    package_share = FindPackageShare("ocs2_ros2_control")

    task_default = PathJoinSubstitution([
        package_share,
        "config",
        "ridgeback_ur5",
        "task.info",
    ])

    lib_default = PathJoinSubstitution([
        package_share,
        "auto_generated",
        "ridgeback_ur5",
    ])

    urdf_default = _urdf_from_xacro_path()
    
    initial_pose_default = PathJoinSubstitution([
        package_share,
        "config",
        "ridgeback_ur5",
        "ridgeback_ur5_initial_pose.yaml",
    ])

    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value=task_default, description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value=lib_default, description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value=urdf_default, description="URDF passed to visualization nodes."),
        DeclareLaunchArgument("debug", default_value="false", description="Skip MPC node when true."),
        DeclareLaunchArgument("commandType", default_value="marker", description="Command interface to start: marker, twist, or trajectory."),
        DeclareLaunchArgument("initialPoseFile", default_value=initial_pose_default, description="Initial pose YAML for marker/visualization."),
        DeclareLaunchArgument("rviz", default_value="true", description="Enable RViz and visualization nodes."),
    ]

    # Common launch configurations
    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")
    debug_flag = LaunchConfiguration("debug")
    command_type = LaunchConfiguration("commandType")
    initial_pose_file = LaunchConfiguration("initialPoseFile")
    rviz_flag = LaunchConfiguration("rviz")

    # ROS 2 controllers
    control_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "ridgeback_ur5", "ridgeback_ur5_control.launch.py"])
        ),
        launch_arguments={
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
        }.items(),
    )

    # MPC
    mpc_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "ridgeback_ur5", "ridgeback_ur5_mpc.launch.py"])
        ),
        launch_arguments={
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
            "debug": debug_flag,
        }.items(),
    )

    # Command (target) for MPC
    command_dir = PathJoinSubstitution([package_share, "launch", "command"])
    command_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([command_dir, PythonExpression(["'", command_type, "'", " + '.launch.py'"])])
        ),
        launch_arguments={
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
            "initialPoseFile": initial_pose_file,
        }.items(),
    )

    # Visualization
    rviz_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "visualization", "visualize.launch.py"])
        ),
        condition=IfCondition(rviz_flag),
        launch_arguments={
            "urdfFile": urdf_file,
            "rviz": rviz_flag,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [
        control_include,
        mpc_include,
        command_include,
        rviz_include,
    ])
