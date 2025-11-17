from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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

    urdf_default = PathJoinSubstitution([
        package_share,
        "description",
        "urdf",
        "ridgeback_ur5.urdf",
    ])

    declared_arguments = [
        DeclareLaunchArgument("taskFile", default_value=task_default, description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value=lib_default, description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value=urdf_default, description="URDF passed to visualization nodes."),
        DeclareLaunchArgument("trajectoryPublishRate", default_value="100.0", description="Trajectory target publish rate."),
        DeclareLaunchArgument("trajectoryHorizon", default_value="5.0", description="Trajectory horizon (s)."),
        DeclareLaunchArgument("trajectoryDt", default_value="0.01", description="Trajectory discretization step (s)."),
        DeclareLaunchArgument("trajectoryAmplitude", default_value="0.2", description="Eight-shape amplitude (m)."),
        DeclareLaunchArgument("trajectoryFrequency", default_value="0.2", description="Eight-shape frequency (Hz)."),
        DeclareLaunchArgument("trajectoryAxisX", default_value="1.0"),
        DeclareLaunchArgument("trajectoryAxisY", default_value="0.0"),
        DeclareLaunchArgument("trajectoryAxisZ", default_value="1.0"),
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
