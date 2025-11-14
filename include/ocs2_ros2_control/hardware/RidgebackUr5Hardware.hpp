#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>

namespace ocs2_ros2_control {

class RidgebackUr5Hardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RidgebackUr5Hardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  void integrateBase(const rclcpp::Duration &period);
  void integrateArm(const rclcpp::Duration &period);
  void loadInitialPoseFromFile(const std::string &path);

  std::vector<std::string> joint_names_;

  // Arm joint state
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_position_commands_;
  std::vector<double> initial_joint_positions_;

  // Base state
  std::array<double, 3> base_pose_{0.0, 0.0, 0.0};        // x, y, yaw
  std::array<double, 2> base_velocity_state_{0.0, 0.0};   // v, w
  std::array<double, 2> base_velocity_command_{0.0, 0.0}; // v, w command
  std::array<double, 3> initial_base_pose_{0.0, 0.0, 0.0};
  std::string initial_pose_file_;

  double max_joint_velocity_{2.0};
  double max_base_velocity_{1.0};
  double max_yaw_velocity_{1.0};

  // Names used inside ros2_control URDF
  static constexpr const char *BASE_FORWARD_JOINT = "base_forward_joint";
  static constexpr const char *BASE_YAW_JOINT = "base_yaw_joint";
  static constexpr const char *BASE_X_JOINT = "base_x_joint";
  static constexpr const char *BASE_Y_JOINT = "base_y_joint";
};

} // namespace ocs2_ros2_control
