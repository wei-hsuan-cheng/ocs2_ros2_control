from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
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
        DeclareLaunchArgument("rviz", default_value="true", description="Enable RViz and visualization nodes."),
        DeclareLaunchArgument("debug", default_value="false", description="Skip MPC node when true."),
        DeclareLaunchArgument("taskFile", default_value=task_default, description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value=lib_default, description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value=urdf_default, description="URDF passed to visualization nodes."),
        
        DeclareLaunchArgument("futureTimeOffset", default_value="0.02", description="Lookahead time (s) when evaluating MPC policy."),
        DeclareLaunchArgument("commandSmoothingAlpha", default_value="1.0", description="Blending factor for consecutive MPC commands."),
        DeclareLaunchArgument("commandType", default_value="marker", description="Command interface to start: marker, twist, or trajectory."),
        DeclareLaunchArgument("initialPoseFile", default_value=initial_pose_default, description="Initial pose YAML for marker/visualization."),
    ]

    # Common launch configurations
    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")
    future_time_offset = LaunchConfiguration("futureTimeOffset")
    command_smoothing_alpha = LaunchConfiguration("commandSmoothingAlpha")
    initial_pose_file = LaunchConfiguration("initialPoseFile")
    command_type = LaunchConfiguration("commandType")
    rviz_flag = LaunchConfiguration("rviz")
    debug_flag = LaunchConfiguration("debug")

    # Include the core controller bringup
    core_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "ridgeback_ur5", "ridgeback_ur5_core.launch.py"])
        ),
        launch_arguments={
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
            "futureTimeOffset": future_time_offset,
            "commandSmoothingAlpha": command_smoothing_alpha,
        }.items(),
    )

    # Visualization
    rviz_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "include", "visualize.launch.py"])
        ),
        condition=IfCondition(rviz_flag),
        launch_arguments={
            "urdfFile": urdf_file,
            "rviz": rviz_flag,
        }.items(),
    )

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

    command_dir = PathJoinSubstitution([package_share, "launch", "command"])

    marker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([command_dir, "marker.launch.py"])
        ),
        condition=IfCondition(PythonExpression(["'", command_type, "'", " == 'marker'"])),
        launch_arguments={
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
            "initialPoseFile": initial_pose_file,
        }.items(),
    )

    twist_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([command_dir, "twist.launch.py"])
        ),
        condition=IfCondition(PythonExpression(["'", command_type, "'", " == 'twist'"])),
        launch_arguments={
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
        }.items(),
    )

    trajectory_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([command_dir, "trajectory.launch.py"])
        ),
        condition=IfCondition(PythonExpression(["'", command_type, "'", " == 'trajectory'"])),
        launch_arguments={
            "taskFile": task_file,
            "libFolder": lib_folder,
            "urdfFile": urdf_file,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [
        core_include,
        mpc_node,
        marker_launch,
        twist_launch,
        trajectory_launch,
        rviz_include,
    ])
