from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    urdf_default = PathJoinSubstitution([
        FindPackageShare("ocs2_ros2_control"),
        "description",
        "urdf",
        "ridgeback_ur5.urdf",
    ])

    control_urdf = PathJoinSubstitution([
        FindPackageShare("ocs2_ros2_control"),
        "description",
        "urdf",
        "ridgeback_ur5_ros2_control.urdf",
    ])

    task_default = PathJoinSubstitution([
        FindPackageShare("ocs2_ros2_control"),
        "config",
        "ridgeback_ur5",
        "task.info",
    ])

    lib_default = PathJoinSubstitution([
        FindPackageShare("ocs2_ros2_control"),
        "auto_generated",
        "ridgeback_ur5",
    ])
    initial_pose_default = PathJoinSubstitution([
        FindPackageShare("ocs2_ros2_control"),
        "config",
        "ridgeback_ur5",
        "ridgeback_ur5_initial_pose.yaml",
    ])

    declared_arguments = [
        DeclareLaunchArgument("rviz", default_value="true", description="Enable RViz and interactive marker."),
        DeclareLaunchArgument("debug", default_value="false", description="Run the legacy mobile_manipulator_mpc executable."),
        DeclareLaunchArgument("taskFile", default_value=task_default, description="Path to the OCS2 task file."),
        DeclareLaunchArgument("libFolder", default_value=lib_default, description="Folder for auto-generated OCS2 libraries."),
        DeclareLaunchArgument("urdfFile", default_value=urdf_default, description="URDF passed to visualization nodes."),
        DeclareLaunchArgument("futureTimeOffset", default_value="0.02", description="Lookahead time (s) when evaluating MPC policy."),
        DeclareLaunchArgument("commandSmoothingAlpha", default_value="1.0", description="Blending factor for consecutive MPC commands."),
        DeclareLaunchArgument("markerPublishRate", default_value="100.0", description="Interactive marker publish rate."),
        DeclareLaunchArgument("enableJoystick", default_value="false", description="Enable joystick control for the marker."),
        DeclareLaunchArgument("enableAutoPosition", default_value="false", description="Enable automatic marker repositioning."),
        DeclareLaunchArgument("initialPoseFile", default_value=initial_pose_default, description="Initial pose YAML for marker and hardware alignment."),
    ]

    robot_description_content = Command([
        FindExecutable(name="cat"),
        " ",
        control_urdf,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controller_config = PathJoinSubstitution([
        FindPackageShare("ocs2_ros2_control"),
        "config",
        "ridgeback_ur5",
        "ridgeback_ur5_ros2_control.yaml",
    ])

    rviz_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ocs2_ros2_control"),
                "launch",
                "include",
                "visualize.launch.py",
            ])
        ),
        launch_arguments={
            "urdfFile": LaunchConfiguration("urdfFile"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    task_file = LaunchConfiguration("taskFile")
    lib_folder = LaunchConfiguration("libFolder")
    urdf_file = LaunchConfiguration("urdfFile")
    future_time_offset = LaunchConfiguration("futureTimeOffset")
    command_smoothing_alpha = LaunchConfiguration("commandSmoothingAlpha")
    initial_pose_file = LaunchConfiguration("initialPoseFile")

    ocs2_params = {
        "controller_manager.ros__parameters.ocs2_controller.ros__parameters.task_file": LaunchConfiguration("taskFile"),
        "controller_manager.ros__parameters.ocs2_controller.ros__parameters.lib_folder": LaunchConfiguration("libFolder"),
        "controller_manager.ros__parameters.ocs2_controller.ros__parameters.urdf_file": LaunchConfiguration("urdfFile"),
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

    mpc_node = Node(
        package="ocs2_ros2_control",
        executable="mobile_manipulator_mpc_node",
        name="mobile_manipulator_mpc",
        condition=UnlessCondition(LaunchConfiguration("debug")),
        parameters=[{"taskFile": task_file, "libFolder": lib_folder, "urdfFile": urdf_file}],
        output="screen",
    )

    marker_node = Node(
        # package="ocs2_ros2_control",
        package="ocs2_mobile_manipulator_ros",
        executable="mobile_manipulator_marker_target",
        name="mobile_manipulator_marker_target",
        condition=IfCondition(LaunchConfiguration("rviz")),
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

    nodes = [
        control_node,
        joint_state_broadcaster,
        ocs2_controller_spawner,
        mpc_node,
        marker_node,
        rviz_include,
    ]

    return LaunchDescription(declared_arguments + nodes)
