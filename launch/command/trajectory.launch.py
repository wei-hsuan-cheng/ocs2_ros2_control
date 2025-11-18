import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    trajectory_defaults = params.get("command", {}).get("trajectory", {})
    axis_defaults = trajectory_defaults.get("axis", {})

    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value="", description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value="", description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value="", description="URDF passed to visualization nodes."),
        
        DeclareLaunchArgument("trajectoryPublishRate", default_value=str(trajectory_defaults.get("publish_rate", 100.0)), description="Trajectory target publish rate."),
        DeclareLaunchArgument("trajectoryHorizon", default_value=str(trajectory_defaults.get("horizon", 50.0)), description="Trajectory horizon (s)."),
        DeclareLaunchArgument("trajectoryDt", default_value=str(trajectory_defaults.get("hodtrizon", 0.01)), description="Trajectory discretization step (s)."),
        DeclareLaunchArgument("trajectoryAmplitude", default_value=str(trajectory_defaults.get("amplitude", 0.2)), description="Eight-shape amplitude (m)."),
        DeclareLaunchArgument("trajectoryFrequency", default_value=str(trajectory_defaults.get("frequency", 0.2)), description="Eight-shape frequency (Hz)."),
        DeclareLaunchArgument("trajectoryAxisX", default_value=str(axis_defaults.get("x", 1.0))),
        DeclareLaunchArgument("trajectoryAxisY", default_value=str(axis_defaults.get("y", 0.0))),
        DeclareLaunchArgument("trajectoryAxisZ", default_value=str(axis_defaults.get("z", 1.0))),
    ]

    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")

    trajectory_node = Node(
        package="ocs2_ros2_control",
        executable="mobile_manipulator_trajectory_target",
        name="mobile_manipulator_trajectory_target",
        parameters=[{
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
            "publishRate": LaunchConfiguration("trajectoryPublishRate"),
            "horizon": LaunchConfiguration("trajectoryHorizon"),
            "dt": LaunchConfiguration("trajectoryDt"),
            "amplitude": LaunchConfiguration("trajectoryAmplitude"),
            "frequency": LaunchConfiguration("trajectoryFrequency"),
            "axisX": LaunchConfiguration("trajectoryAxisX"),
            "axisY": LaunchConfiguration("trajectoryAxisY"),
            "axisZ": LaunchConfiguration("trajectoryAxisZ"),
        }],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [trajectory_node])
