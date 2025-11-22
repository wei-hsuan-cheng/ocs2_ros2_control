import subprocess

def generate_urdf_from_xacro(xacro_in: str, urdf_out: str, initial_pose: str) -> str:
    """Generate a URDF from the given xacro path using provided output/initial pose paths."""
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
        print(f"[ocs2_ros2_control] Generated URDF from xacro: {urdf_out}")
    except Exception as exc:  # noqa: BLE001
        print(f"[ocs2_ros2_control] Failed to generate URDF from xacro: {exc}")

    return urdf_out
