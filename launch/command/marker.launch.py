from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("ocs2_ros2_control")

    urdf_default = PathJoinSubstitution([
        package_share,
        "description",
        "urdf",
        "ridgeback_ur5.urdf",
    ])

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
        DeclareLaunchArgument("initialPoseFile", default_value=initial_pose_default, description="Initial pose YAML for marker/visualization."),
        DeclareLaunchArgument("markerPublishRate", default_value="100.0", description="Interactive marker publish rate."),
        DeclareLaunchArgument("enableJoystick", default_value="false", description="Enable joystick control for the marker."),
        DeclareLaunchArgument("enableAutoPosition", default_value="false", description="Enable automatic marker repositioning."),
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
        }],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [marker_node])
