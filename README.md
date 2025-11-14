# ocs2_ros2_control

Minimal ROS 2 control package that embeds the [OCS2](https://leggedrobotics.github.io/ocs2/) mobile manipulator MPC stack in a `ros2_control` workflow. It provides:

- A fake hardware plugin (`RidgebackUr5Hardware`) that simulates **Ridgeback + UR5 mobile manipulator** (differential-drive mobile robot + 6-axis arm).

- An `Ocs2Ros2Controller` which bridges the hardware state to the OCS2 MRT/MPC interface and streams back the optimal commands.

The package is intentionally self‑contained (`urdf`/`xacro`, `task` and `rviz` config), but it **depends on the upstream OCS2 ROS 2 repositories** for the solver libraries and marker/MPC nodes.

## Prerequisites

1. Install ROS 2 Humble (or a newer distro with the same APIs).
2. Clone and build [`wei-hsuan-cheng/ocs2_ros2`](https://github.com/wei-hsuan-cheng/ocs2_ros2). That repository provides the `ocs2_mobile_manipulator` libraries and the ROS nodes (`mobile_manipulator_mpc_node`, marker target) that this package launches.

Make sure the `ocs2_ros2` workspace is sourced before building this package.

## Build and Run Demo

```bash
# Build and install
cd ~/ros2_ws
colcon build --symlink-install --packages-select ocs2_ros2_control && . install/setup.bash
# Run ridgeback_ur5 demo (marker pose tracking)
ros2 launch ocs2_ros2_control ridgeback_ur5_bringup.launch.py
```

If you built `ocs2_ros2` in another workspace, source it **before** running the commands above (so their messages and plugins are discoverable).

The demo launch file starts:

- `ros2_control_node` with the fake Ridgeback + UR5 hardware and the
  `Ocs2Ros2Controller`.
- The `joint_state_broadcaster` that publsihes `joint_states` from hardware.
- OCS2 MPC and marker nodes from `ocs2_mobile_manipulator_ros`.
- RViz (unless `rviz:=false`) with the copied visualization config.

Useful arguments:

| Argument            | Default | Description                                      |
|---------------------|---------|--------------------------------------------------|
| `rviz`              | `true`  | Enable/disable RViz + interactive marker.        |
| `taskFile`          | local   | Path to the OCS2 `task.info` file.               |
| `libFolder`         | local   | Folder containing the auto-generated OCS2 libs.  |
| `urdfFile`          | local   | URDF used by visualization nodes.                |
| `markerPublishRate` | `100.0` | Interactive marker publish rate (Hz).            |

## Folder layout

```bash
auto_generated/   -> pre-built OCS2 libraries (CppAD) for Ridgeback + UR5
config/           -> ros2_control YAML + task files
description/      -> xacro/urdf assets (ros2_control settings included)
include/          -> Header files
rviz/             -> RViz configuration copied from OCS2 examples
src/
  control/        -> ros2_control controllers (OCS2 bridge)
  hardware/       -> Ridgeback + UR5 fake_hardware (SystemInterface)
  visualization/  -> Marker, trajectory, and pose visualizations in RViz2
```

## Notes

- The fake hardware integrates velocities directly; it is intended for
  controller bring-up and visualization only.
- `auto_generated/ridgeback_ur5` is copied from the OCS2 examples for
  convenience. Regenerate those libraries if you modify the task model.
- The OCS2 MPC core process is provided by `ocs2_mobile_manipulator_ros`, make sure that package is present (from the upstream repo mentioned above).

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)