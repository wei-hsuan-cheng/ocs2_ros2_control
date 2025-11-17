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
        DeclareLaunchArgument("twistPublishRate", default_value="100.0", description="Twist target publish rate."),
        DeclareLaunchArgument("twistHorizon", default_value="1.0", description="Twist target horizon (s)."),
        DeclareLaunchArgument("twistDt", default_value="0.01", description="Twist discretization step (s)."),
        DeclareLaunchArgument("twistInWorld", default_value="false", description="Interpret twist in world frame."),
        DeclareLaunchArgument("twistVelX", default_value="0.0"),
        DeclareLaunchArgument("twistVelY", default_value="0.0"),
        DeclareLaunchArgument("twistVelZ", default_value="0.05"),
        DeclareLaunchArgument("twistOmegaX", default_value="0.0"),
        DeclareLaunchArgument("twistOmegaY", default_value="0.0"),
        DeclareLaunchArgument("twistOmegaZ", default_value="0.0"),
    ]

    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")

    twist_node = Node(
        package="ocs2_ros2_control",
        executable="mobile_manipulator_twist_target",
        name="mobile_manipulator_twist_target",
        parameters=[{
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
            "publishRate": LaunchConfiguration("twistPublishRate"),
            "horizon": LaunchConfiguration("twistHorizon"),
            "dt": LaunchConfiguration("twistDt"),
            "twistInWorld": LaunchConfiguration("twistInWorld"),
            "vx": LaunchConfiguration("twistVelX"),
            "vy": LaunchConfiguration("twistVelY"),
            "vz": LaunchConfiguration("twistVelZ"),
            "wx": LaunchConfiguration("twistOmegaX"),
            "wy": LaunchConfiguration("twistOmegaY"),
            "wz": LaunchConfiguration("twistOmegaZ"),
        }],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [twist_node])
