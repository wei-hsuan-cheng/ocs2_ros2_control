from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument("debug", default_value="false", description="Skip MPC node when true."),
        DeclareLaunchArgument("taskFile", default_value="", description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value="", description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value="", description="URDF passed to visualization nodes."),
    ]

    # Common launch configurations
    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")
    debug_flag = LaunchConfiguration("debug")

    # MPC controller node
    mpc_node = Node(
        package="ocs2_ros2_control",
        executable="mobile_manipulator_mpc_node",
        name="mobile_manipulator_mpc",
        condition=UnlessCondition(debug_flag),
        parameters=[{
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
        }],
        output="screen",
    )

    return LaunchDescription(declared_arguments + [
        mpc_node,
    ])
