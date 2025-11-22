import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

common_dir = Path(__file__).resolve().parent.parent / "common"
if str(common_dir) not in sys.path:
    sys.path.insert(0, str(common_dir))

from urdf_utils import generate_urdf_from_xacro  # noqa: E402


def generate_launch_description():
    package_share = FindPackageShare("ocs2_ros2_control")
    share_dir = Path(get_package_share_directory("ocs2_ros2_control"))

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

    xacro_path = share_dir / "description" / "ridgeback_ur5" / "urdf" / "ridgeback_ur5.urdf.xacro"
    urdf_path = share_dir / "description" / "ridgeback_ur5" / "urdf" / "ridgeback_ur5.urdf"
    initial_pose_path = share_dir / "config" / "ridgeback_ur5" / "ridgeback_ur5_initial_pose.yaml"

    urdf_default = generate_urdf_from_xacro(str(xacro_path), str(urdf_path), str(initial_pose_path))
    
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
